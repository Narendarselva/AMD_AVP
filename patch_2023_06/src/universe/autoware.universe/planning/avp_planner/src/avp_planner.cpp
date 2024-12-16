// Copyright 2021 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "avp_planner/avp_planner.hpp"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;
bool isStopped(
  const std::deque<Odometry::ConstSharedPtr> & odom_buffer, const double th_stopped_velocity_mps)
{
  for (const auto & odom : odom_buffer) {
    if (std::abs(odom->twist.twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);

  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, all_parking_lots, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

bool checkValidIndex(const Pose & p_base, const Pose & p_next, const Pose & p_target)
{
  const Point2d base2target(
    p_target.position.x - p_base.position.x, p_target.position.y - p_base.position.y);
  const Point2d target2next(
    p_next.position.x - p_target.position.x, p_next.position.y - p_target.position.y);
  return base2target.dot(target2next) > 0.0;
}

AvpPlanner::AvpPlanner(const std::string & node_name, const rclcpp::NodeOptions & node_options)
  : rclcpp::Node(node_name, node_options) , tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // NodeParams
  {
    auto & p = node_param_;
    p.update_rate = declare_parameter("update_rate", 10.0);
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
    p.ego_stop_scale_factor = declare_parameter<double>("ego_stop_scale_factor");
    p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    goal_pose_pub_ = create_publisher<PoseStamped>("/planning/mission_planning/goal", qos);
    checkpoint_pub_ = create_publisher<PoseStamped>("/planning/mission_planning/checkpoint", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_checkpoint_array_pub_ = create_publisher<PoseArray>("~/debug/checkpoint_array", qos);
    pub_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  }

  // set vehicle_info
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
    vehicle_shape_.wheel_base = vehicle_info.wheel_base_m;
  }

  // subscriber
  {
    odom_ = nullptr;
    sub_odometry_ = create_subscription<Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
        std::bind(&AvpPlanner::on_odometry, this, std::placeholders::_1));

    // route_handler
    auto qos_transient_local = rclcpp::QoS{1}.transient_local();
    vector_map_subscriber_ = create_subscription<HADMapBin>(
        "/map/vector_map", qos_transient_local, std::bind(&AvpPlanner::onMap, this, _1));
    avp_parking_lot_sub_ = create_subscription<String>(
        "~/avp_selected_parking_lot", 1, std::bind(&AvpPlanner::onAvpParkingLot, this, _1));
    parkinglot_sub_ = create_subscription<ParkingLots>(
        "/motovis_interface/parking_lots", 1,
        std::bind(&AvpPlanner::onParkinglotArray,this, std::placeholders::_1));
    sub_trajectory_ = create_subscription<Trajectory>(
        "~/input/trajectory", 1,
        std::bind(&AvpPlanner::onTrajectory, this, std::placeholders::_1));
    scenario_sub_ = create_subscription<Scenario>(
        "/planning/scenario_planning/scenario", 1,
        std::bind(&AvpPlanner::onScenario, this, _1));
  }

  //Services
  {
    const auto adaptor = component_interface_utils::NodeAdaptor(this);
    adaptor.init_cli(cli_route_);
    adaptor.init_cli(cli_clear_);
  }
}

AvpPlanner::~AvpPlanner()
{
}

void AvpPlanner::onParkinglotArray(ParkingLots::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(parking_lot_mutex_);
  parkinglot_arr_ptr_ = msg;
}

void AvpPlanner::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

void AvpPlanner::onScenario(const Scenario::ConstSharedPtr msg)
{
  scenario_ = msg;
  if((scenario_->current_scenario == tier4_planning_msgs::msg::Scenario::PARKING)
      && isSetStop_ && isParkingGoalSet_)
  {
    is_avp_activated_ = false;
    isSetStop_ = false;
    isParkingGoalSet_ = false;
  }
}

void AvpPlanner::onAvpParkingLot(const String::ConstSharedPtr msg)
{
  PoseStamped goal_pose;
  selected_parkinglot_id_ = msg->data;
  int min_path_distance = INT_MAX;

  if(route_handler_ptr_->isMapMsgReady())
  {
    lanelet::Lanelet start_llt;
    const lanelet::ConstLanelet *final_goal_llt = nullptr;
    route_handler_ptr_->getClosestLaneletFromPose(odom_->pose.pose, &start_llt);
    RCLCPP_INFO_STREAM(
        get_logger(), "Start laneIds:" << start_llt.id()
        << std::endl
        );

    lanelet::ConstPolygons3d pls = route_handler_ptr_->getParkingLots();
    for(const auto &pl : pls)
    {
      const std::string name = pl.attributeOr("name", "none");
      RCLCPP_INFO(get_logger(),"ParkingLot ID: %s",name.c_str());
      if(name == selected_parkinglot_id_)
      {
        lanelet::ConstLanelets llts = route_handler_ptr_->getLinkedLanelets(pl);
        for (const auto & goal_llt : llts)
        {
          RCLCPP_INFO_STREAM(
              get_logger(), "Linked laneId:" << goal_llt.id()
              << std::endl
              );
          //Select shortest path lanelet
          lanelet::Optional<lanelet::routing::Route> optional_route =
            route_handler_ptr_->getRoutingGraphPtr()->getRoute(start_llt, goal_llt, 0);
          if(optional_route)
          {
            int path_distance = 0;
            const auto shortest_path = optional_route->shortestPath();
            for (const auto & llt : shortest_path)
            {
              const lanelet::ConstLineString3d centerline = llt.centerline();
              path_distance += centerline.size();
            }
            RCLCPP_INFO_STREAM(
              get_logger(), "path distance from lanelet:" << goal_llt.id() <<" -> "
              <<path_distance<<""<< std::endl
              );
            if(path_distance < min_path_distance)
            {
              final_goal_llt = &goal_llt;
              min_path_distance = path_distance;
            }
          }
        }

        if (final_goal_llt)
        {
          waypoint_mtx_.lock();
          waypoints_.clear();
          waypoint_count_ = 0;

          RCLCPP_INFO_STREAM(
              get_logger(), "Final goal laneId:" << final_goal_llt->id()
              << std::endl
              );
          const lanelet::ConstLineString3d centerline = final_goal_llt->centerline();
          PoseArray pose_array;
          Pose pos;
          for (size_t i = 0; i < centerline.size() - 1; i++) {
            double angle{0.0};
            if (i + 1 < centerline.size()) {
              angle = tier4_autoware_utils::calcAzimuthAngle(
                  lanelet::utils::conversion::toGeomMsgPt(centerline[i]),
                  lanelet::utils::conversion::toGeomMsgPt(centerline[i+1]));
            } else if (i != 0) {
              angle = tier4_autoware_utils::calcAzimuthAngle(
                  lanelet::utils::conversion::toGeomMsgPt(centerline[i-1]) ,
                  lanelet::utils::conversion::toGeomMsgPt(centerline[i]));
            }

            const auto & pt = centerline[i];
            pose_array.header.stamp = rclcpp::Clock().now();
            pose_array.header.frame_id = "map";
            pos.position = lanelet::utils::conversion::toGeomMsgPt(pt);
            pos.orientation = tier4_autoware_utils::createQuaternionFromYaw(angle);
            pose_array.poses.push_back(pos);

            waypoints_.push_back(pos);
          }
          debug_pose_array_pub_->publish(pose_array);

          goal_pose.header.stamp = rclcpp::Clock().now();
          goal_pose.header.frame_id = "map";
          goal_pose.pose.position = waypoints_[waypoints_.size() -1].position;
          goal_pose.pose.orientation = waypoints_[waypoints_.size() -1].orientation;
          waypoint_count_++;
          goal_pose_pub_->publish(goal_pose);
          is_avp_activated_ = true;
          waypoint_mtx_.unlock();
          return;
        }
      }
    }
  }
}

void AvpPlanner::onMap(const HADMapBin::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_map_);
  map_ptr_ = msg;
  route_handler_ptr_->setMap(*map_ptr_);
}

void AvpPlanner::onTrajectory(const Trajectory::ConstSharedPtr input_msg)
{
  {
    const auto waiting = [this](const auto & str) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "waiting for %s ...",
          str);
    };

    if (!map_ptr_) {
      waiting("Map");
      return;
    }

    if (!odom_) {
      waiting("current velocity");
      return;
    }

    if (!scenario_) {
      waiting("Scenario");
      return;
    }

    if (input_msg->points.empty()) {
      waiting("Trajectory points empty");
      return;
    }
  }

  const auto current_odometry_ptr = odom_;
  const auto current_vel = current_odometry_ptr->twist.twist.linear.x;
  current_pose_ = current_odometry_ptr->pose.pose;
  Trajectory output_trajectory = *input_msg;
  TrajectoryPoints output_trajectory_points =
    motion_utils::convertToTrajectoryPointArray(*input_msg);
  const auto is_stopped = isStopped(odom_buffer_, node_param_.th_stopped_velocity_mps);

  if(is_avp_activated_ &&
      (scenario_->current_scenario == tier4_planning_msgs::msg::Scenario::LANEDRIVING))
  {
    // No stopping point set already check the avp stopping point condition
    // Set the stop point for vehicle which is not stopped
    if(!isSetStop_ && !is_stopped)
    {
      //ego is inside the selected parking lot &&
      //we have best parking lot for parking
      if(isInParkingLot(selected_parkinglot_id_,current_pose_))
      {
        parking_lot_mutex_.lock();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "try to find best parking lot!!!");
        int selected_parking_lot = findBestParkingLot(selected_parkinglot_id_);
        if(selected_parking_lot >= 0)
        {
          RCLCPP_INFO(get_logger(),"Found best parking lot to park && adding stopping point to lane driving trajectory!!!");

          //insert the stop vel
          insertStopVelocity(output_trajectory_points, current_vel, current_pose_);
          stop_trajectory_points_ = output_trajectory_points;
          isSetStop_ = true;

          //Clear route with avp triggered true  to routing API module so that it
          //never changes the operation mode to STOP when route changes from
          //lanedriving to parking
          const auto request = std::make_shared<ClearRoute::Service::Request>();
          request->is_avp_triggered = true;
          cli_clear_->async_send_request(request);
        }
        parking_lot_mutex_.unlock();
      }
    }
    else if(isSetStop_ && !is_stopped)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Stop point is Set , Waiting for ego to stop!!!");
      //Added this for smooth stopping of the ego
      //Saved stop points are used to setstop point until the ego comes to permenant stop
      insertStopVelocity(
         stop_trajectory_points_, current_vel, current_pose_);
      output_trajectory_points = stop_trajectory_points_;
    } 
    else if(isSetStop_ && is_stopped)
    {
      // Already stopping point is set on the trajectory
      // Wait for the vehicle to stop and then update
      // the parking lot goal pose
      if(!isParkingGoalSet_)
      {
        parking_lot_mutex_.lock();
        int selected_parking_lot = findBestParkingLot(selected_parkinglot_id_);
        if(selected_parking_lot >= 0)
        {
          computeParkingGoalPose(selected_parking_lot);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Stop point is Set and ego stopped ,so updating the parking lot goal pose for best parking lot to park ID %d!!!",selected_parking_lot);
          parking_lot_mutex_.unlock();
          isParkingGoalSet_ = true;
        }
      }
      else
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Stop point is Set , ego stopped and Parking lot goal is updated , send stop traj until PARKING scenario kicks in!!!");
        output_trajectory_points = stop_trajectory_points_;
      }
    }
  }

  auto trajectory = motion_utils::convertToTrajectory(output_trajectory_points);
  trajectory.header = input_msg->header;
  pub_trajectory_->publish(trajectory);
}

void AvpPlanner::insertStopVelocity(
    TrajectoryPoints & output,
    const double current_vel, const Pose ego_pose)
{
  (void) current_vel;
  const auto ego_pos_on_path = calcLongitudinalOffsetPose(output, ego_pose.position, 0.0);

  if (ego_pos_on_path) {
    StopPoint current_stop_pos{};
    current_stop_pos.index = findFirstNearestSegmentIndexWithSoftConstraints(
        output, ego_pose, node_param_.ego_nearest_dist_threshold,
        node_param_.ego_nearest_yaw_threshold);
    current_stop_pos.point.pose = ego_pos_on_path.get();

    insertStopPoint(current_stop_pos, output);
  }
}

void AvpPlanner::insertStopPoint(
  const StopPoint & stop_point, TrajectoryPoints & output)
{
  const auto traj_end_idx = output.size() - 1;
  const auto & stop_idx = stop_point.index;

  const auto & p_base = output.at(stop_idx);
  const auto & p_next = output.at(std::min(stop_idx + 1, traj_end_idx));
  const auto & p_insert = stop_point.point;

  constexpr double min_dist = 1e-3;

  const auto is_p_base_and_p_insert_overlap = calcDistance2d(p_base, p_insert) < min_dist;
  const auto is_p_next_and_p_insert_overlap = calcDistance2d(p_next, p_insert) < min_dist;
  const auto is_valid_index = checkValidIndex(p_base.pose, p_next.pose, p_insert.pose);

  auto update_stop_idx = stop_idx;

  if (!is_p_base_and_p_insert_overlap && !is_p_next_and_p_insert_overlap && is_valid_index) {
    // insert: start_idx and end_idx are shifted by one
    output.insert(output.begin() + stop_idx + 1, p_insert);
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  } else if (is_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  }

  //Instead of abruptly stopping the ego(hard stop)
  //we are now adding smooth stop
  double ref_vel = p_base.longitudinal_velocity_mps;
  double scale = ref_vel / node_param_.ego_stop_scale_factor;
  for (size_t i = update_stop_idx; i < output.size(); ++i) {
    if((ref_vel - scale) > 0.0)
    {
      ref_vel = ref_vel - scale;
      output.at(i).longitudinal_velocity_mps = ref_vel;
    }
    else
    {
      output.at(i).longitudinal_velocity_mps = 0.0;
    }
  }
}

bool AvpPlanner::isInParkingLot(
  const std::string parking_lot_id,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(route_handler_ptr_->getLaneletMapPtr(),
            search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }
  
  const std::string name = nearest_parking_lot->attributeOr("name", "none");
  if( name != parking_lot_id)
  {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

bool AvpPlanner::isInParkingLot(
  const std::string parking_lot_id,
  const geometry_msgs::msg::Point & current_point)
{
  const auto & p = current_point;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(route_handler_ptr_->getLaneletMapPtr(),
            search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }
  
  const std::string name = nearest_parking_lot->attributeOr("name", "none");
  if( name != parking_lot_id)
  {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}


bool AvpPlanner::isParkingLotAvailable()
{
  for(int i=0; i < parkinglot_arr_ptr_->detect_slot_num;i++)
  {
    if(parkinglot_arr_ptr_->detect_slots[i].available_state)
    {
      return true;
    }
  }
  return false;
}

Point AvpPlanner::findIntersection(Point A, Point B, Point C,Point D)
{
  // Line AB represented as a1x + b1y = c1
  Point result;
  double a = B.y - A.y;
  double b = A.x - B.x;
  double c = a*(A.x) + b*(A.y);
  // Line CD represented as a2x + b2y = c2
  double a1 = D.y - C.y;
  double b1 = C.x - D.x;
  double c1 = a1*(C.x)+ b1*(C.y);
  double det = a*b1 - a1*b;
  if (det == 0) {
    result.x = FLT_MAX;
    result.y = FLT_MAX;
    return result;
  } else {
    result.x = (b1*c - b*c1)/det;
    result.y = (a*c1 - a1*c)/det;
    return result;
  }
}

double AvpPlanner::getGoalAngleOffset(int parking_direction,
    int parkinglot_side,double slope_angle)
{
  double offset_angle = slope_angle;
  double yaw = tf2::getYaw(current_pose_.orientation);
  RCLCPP_INFO(get_logger(),"base link yaw %f",yaw * 180/3.141);
  RCLCPP_INFO(get_logger(),"parking_direction:%d parkinglot_side:%d",parking_direction,parkinglot_side);

  switch(parkinglot_side)
  {
    case 0://Left Side
      if(slope_angle > 0)
      {
        offset_angle = slope_angle + 1.57;
      }
      else
      {
        offset_angle = slope_angle - 1.57;
      }

      if(parking_direction == 0) // Reverse parking
      {
        offset_angle = offset_angle;
      }
      else // Fwd Parking
      {
        offset_angle = offset_angle + 3.141;
      }
      break;

    case 1://Right Side
      if(slope_angle > 0)
      {
        offset_angle = slope_angle + 1.57;
      }
      else
      {
        offset_angle = slope_angle - 1.57;
      }

      if(parking_direction == 0) // Reverse parking
      {
        offset_angle = offset_angle + 3.141;
      }
      else // Fwd Parking
      {
        offset_angle = offset_angle;
      }
      break;
    case 2: //Front Side
      if(parking_direction == 0) // Reverse parking
      {
        if(slope_angle < 0)
        {
          if(yaw < 0)
          {
            offset_angle = slope_angle + 1.57;
          }
          else
          {
            offset_angle = -(fabs(slope_angle) + 1.57);
          }
        }
        else
        {
          if(yaw < 0)
          {
            offset_angle = slope_angle + 1.57;
          }
          else
          {
            offset_angle = slope_angle + 4.71239;
          }
        }
      }
      else //Forward parking
      {
        if(slope_angle < 0)
        {
          if(yaw < 0)
          {
            offset_angle = -(fabs(slope_angle) + 1.57);
          }
          else
          {
            offset_angle = slope_angle + 1.57;
          }
        }
        else
        {
          if(yaw < 0)
          {
            offset_angle = slope_angle + 4.71239;
          }
          else
          {
            offset_angle = slope_angle + 1.57;
          }
        }
      }
      break;
    case 3://back Side
      if(parking_direction == 0) // Reverse parking
      {
        if(slope_angle < 0)
        {
          if(yaw < 0)
          {
            offset_angle = -(fabs(slope_angle) + 1.57);
          }
          else
          {
            offset_angle = slope_angle + 1.57;
          }
        }
        else
        {
          if(yaw < 0)
          {
            offset_angle = slope_angle + 4.71239;
          }
          else
          {
            offset_angle = slope_angle + 1.57;
          }
        }
      }
      else //Forward parking
      {
        if(slope_angle < 0)
        {
          if(yaw < 0)
          {
            offset_angle = slope_angle + 1.57;
          }
          else
          {
            offset_angle = -(fabs(slope_angle) + 1.57);
          }
        }
        else
        {
          if(yaw < 0)
          {
            offset_angle = slope_angle + 1.57;
          }
          else
          {
            offset_angle = slope_angle + 4.71239;
          }
        }
      }
      break;

    default:
      RCLCPP_ERROR(get_logger(),"Invalid parking lot side!!!");
      break;
  }
  return offset_angle;
}

double AvpPlanner::getSlopeWithRefPlane(Point p1, Point p2)
{
  double m1, m2, angle_radians;
  // Calculate the slopes of the two lines
  // Reference Map axis(x-axis) slope is zero
  m1 = 0;

  //When p2-p1 line is perpendicular to map-axis
  if((p2.x - p1.x) == 0)
  {
    return (90 * 3.141/180);
  }

  m2 = (p2.y - p1.y) / (p2.x - p1.x);

  // Calculate the angle between the two lines in radians
  angle_radians = atan((m2 - m1) / (1 + m1 * m2));
  return angle_radians;
}


int AvpPlanner::pointPosition(Point vehicle, Point referenceVector, Point point)
{
  // Function to determine whether a point is to the left or right of a reference vector
  // Calculate vectors from the vehicle to the reference vector and the vehicle to the point
  double vectorVX = referenceVector.x - vehicle.x;
  double vectorVY = referenceVector.y - vehicle.y;
  double vectorPX = point.x - vehicle.x;
  double vectorPY = point.y - vehicle.y;

  // Calculate the cross product of the vectors
  double crossProduct = vectorVX * vectorPY - vectorVY * vectorPX;

  // Determine the position based on the sign of the cross product
  if (crossProduct > 0) {
    RCLCPP_INFO(get_logger(),"LEFT");
    return 0;
  } else if (crossProduct < 0) {
    RCLCPP_INFO(get_logger(),"RIGHT");
    return 1;
  } else {
    RCLCPP_INFO(get_logger(),"SAME");
    return -1;
  }
}

void AvpPlanner::checkGoalAngleOffset(int parkinglot_side,Point point1,
    Point point4,double *goal_angle_offset)
{
  Point mid_point,check_point;
  Point ego_pose1,ego_pose2;

  mid_point.x = (point1.x + point4.x)/2;
  mid_point.y = (point1.y + point4.y)/2;

  check_point.x = mid_point.x + (100 * cos(*goal_angle_offset));
  check_point.y = mid_point.y + (100 * sin(*goal_angle_offset));

  ego_pose1.x = current_pose_.position.x;
  ego_pose1.y = current_pose_.position.y;

  ego_pose2.x = current_pose_.position.x +
    (5 * cos(tf2::getYaw(current_pose_.orientation)));
  ego_pose2.y = current_pose_.position.y +
    (5 * sin(tf2::getYaw(current_pose_.orientation)));

  int checked_side = pointPosition(ego_pose1,ego_pose2,check_point);
  if(parkinglot_side != checked_side)
  {
    *goal_angle_offset = *goal_angle_offset + 3.141;
    RCLCPP_INFO(get_logger(),"Based on calculated goal_angle checkedside %d is diff from parkinglot_side %d so updated the goal_pose_angle to %lf",checked_side,parkinglot_side,(*goal_angle_offset) * 180/3.141);
  }
}

PoseStamped AvpPlanner::planRightPark(Point point1,Point point2,Point point3,
    Point point4,int parking_direction,double goal_angle_offset)
{
  (void) point2;
  (void) point3;
  PoseStamped inter_pose, end_goal_pose , wp1_pose, wp2_pose;
  Pose pos;
  PoseArray pose_array;
  tf2::Quaternion q;
  Point goal_pose,waypoint1,waypoint2,mid_point;

  //MidPoint of P1 & P4
  mid_point.x = (point1.x + point4.x)/2;
  mid_point.y = (point1.y + point4.y)/2;

  if(parking_direction == 0)//Reverse parking
  {
    //               p1           p2
    //                _____________
    //               |    (10deg)  |
    //               w2    w1   Goal|
    //               *    *   *    |
    // Interpoint    |_____________|            
    //         *     p4           p3

    //Find goal , w1 , w2 point
    {
      //GoalPoint
      goal_pose.x = mid_point.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* cos(goal_angle_offset));
      goal_pose.y = mid_point.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* sin(goal_angle_offset));

      //w1 point
      waypoint1.x = mid_point.x + ( 2 * cos(goal_angle_offset));
      waypoint1.y = mid_point.y + ( 2 * sin(goal_angle_offset));

      //w2 point
      waypoint2.x = mid_point.x + ( -1 * cos(goal_angle_offset));
      waypoint2.y = mid_point.y + ( -1 * sin(goal_angle_offset));
    }
   //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);

    //Waypoint pose 1
    wp1_pose.header.stamp = rclcpp::Clock().now();
    wp1_pose.header.frame_id = "map";
    wp1_pose.pose.position.x = waypoint1.x;
    wp1_pose.pose.position.y = waypoint1.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp1_pose.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose.header.stamp = rclcpp::Clock().now();
    wp2_pose.header.frame_id = "map";
    wp2_pose.pose.position.x = waypoint2.x;
    wp2_pose.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp2_pose.pose.orientation = tf2::toMsg(q);
    
    //Intesection point
    Point ego_pose1,ego_pose2;
    ego_pose1.x = current_pose_.position.x;
    ego_pose1.y = current_pose_.position.y;

    ego_pose2.x = current_pose_.position.x +
      (5 * cos(tf2::getYaw(current_pose_.orientation)));
    ego_pose2.y = current_pose_.position.y +
      (5 * sin(tf2::getYaw(current_pose_.orientation)));
    Point intersection_point = findIntersection(waypoint1,waypoint2,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base ) * cos(tf2::getYaw(current_pose_.orientation) + 3.141));
    intersection_point.y = intersection_point.y +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base ) * sin(tf2::getYaw(current_pose_.orientation) + 3.141));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = current_pose_.orientation;
    
    //Publish way points
    //checkpoint_pub_->publish(inter_pose);
    //checkpoint_pub_->publish(wp1_pose);
    //checkpoint_pub_->publish(wp2_pose); 

    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->waypoints.push_back(inter_pose.pose);
    route->waypoints.push_back(wp1_pose.pose);
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);

    //Debug waypoint array
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;  //intersection pose
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose.pose.position;    //wp1
    pos.orientation = wp1_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose.pose.position;     //wp2
    pos.orientation = wp2_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    debug_checkpoint_array_pub_->publish(pose_array);
  }
  else //Fwd parking
  {
    //    p4           p3
    //     _____________
    //    |             |
    // w1 |   Goal      |
    // *  |   *         |
    //    |_____________|            
    //    p1           p2

    {
      //End goal
      goal_pose.x = mid_point.x + ((vehicle_shape_.base2back + 0.3) * cos(goal_angle_offset ));
      goal_pose.y = mid_point.y + ((vehicle_shape_.base2back + 0.3)* sin(goal_angle_offset ));

      //wp1
      waypoint1.x = mid_point.x + ( (-5) * cos(goal_angle_offset));
      waypoint1.y = mid_point.y + ( (-5) * sin(goal_angle_offset));
      waypoint1.x = waypoint1.x + (3  * cos(goal_angle_offset - 1.57));
      waypoint1.y = waypoint1.y + (3  * sin(goal_angle_offset - 1.57));

      //wp2
      waypoint2.x = mid_point.x + ( (-2 * vehicle_shape_.wheel_base) * cos(goal_angle_offset));
      waypoint2.y = mid_point.y + ( (-2 * vehicle_shape_.wheel_base) * sin(goal_angle_offset));
    }

    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);
    
    //Intesection point
    Point ego_pose1,ego_pose2,mid_point;
    ego_pose1.x = current_pose_.position.x;
    ego_pose1.y = current_pose_.position.y;

    ego_pose2.x = current_pose_.position.x +
      (5 * cos(tf2::getYaw(current_pose_.orientation)));
    ego_pose2.y = current_pose_.position.y +
      (5 * sin(tf2::getYaw(current_pose_.orientation)));
    mid_point.x = (point1.x + point4.x)/2;
    mid_point.y = (point1.y + point4.y)/2;
    Point intersection_point = findIntersection(mid_point,goal_pose,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      (3 * cos(tf2::getYaw(current_pose_.orientation)));
    intersection_point.y = intersection_point.y +
      (3 * sin(tf2::getYaw(current_pose_.orientation)));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = current_pose_.orientation;
    inter_pose.pose.orientation.z = inter_pose.pose.orientation.z;

    //Waypoint pose 1
    wp1_pose.header.stamp = rclcpp::Clock().now();
    wp1_pose.header.frame_id = "map";
    wp1_pose.pose.position.x = waypoint1.x;
    wp1_pose.pose.position.y = waypoint1.y;
    q.setRPY(0, 0, goal_angle_offset + 0.61); //35deg
    q.normalize();
    wp1_pose.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose.header.stamp = rclcpp::Clock().now();
    wp2_pose.header.frame_id = "map";
    wp2_pose.pose.position.x = waypoint2.x;
    wp2_pose.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    wp2_pose.pose.orientation = tf2::toMsg(q);

    //Publish way points
    checkpoint_pub_->publish(inter_pose);
    checkpoint_pub_->publish(wp1_pose);
    //checkpoint_pub_->publish(wp2_pose);
    
    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->waypoints.push_back(inter_pose.pose);
    route->waypoints.push_back(wp1_pose.pose);
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);

    //Debug
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose.pose.position;
    pos.orientation = wp1_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose.pose.position;
    pos.orientation = wp2_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    debug_checkpoint_array_pub_->publish(pose_array);
  }
  return end_goal_pose;
}

PoseStamped AvpPlanner::planLeftPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset)
{
  (void) point2;
  (void) point3;
  PoseStamped inter_pose, end_goal_pose, wp1_pose, wp2_pose;
  Pose pos;
  PoseArray pose_array;
  tf2::Quaternion q;
  Point goal_pose,waypoint1,waypoint2,mid_point;

  //MidPoint
  mid_point.x = (point1.x + point4.x)/2;
  mid_point.y = (point1.y + point4.y)/2;

  if(parking_direction == 0)//Reverse parking
  {
    //               p1           p2
    //                _____________
    //               |    (10deg)  |
    //               w2    w1   Goal|
    //               *    *   *    |
    // Interpoint    |_____________|
    //         *     p4           p3

    //Find goal , w1 , w2 point
    {
      //GoalPoint
      goal_pose.x = mid_point.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* cos(goal_angle_offset));
      goal_pose.y = mid_point.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* sin(goal_angle_offset));

      //w1 point
      waypoint1.x = mid_point.x + ( 2 * cos(goal_angle_offset));
      waypoint1.y = mid_point.y + ( 2 * sin(goal_angle_offset));

      //w2 point
      waypoint2.x = mid_point.x + ( -1 * cos(goal_angle_offset));
      waypoint2.y = mid_point.y + ( -1 * sin(goal_angle_offset));
    }
  
    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);

    //Waypoint pose 1
    wp1_pose.header.stamp = rclcpp::Clock().now();
    wp1_pose.header.frame_id = "map";
    wp1_pose.pose.position.x = waypoint1.x;
    wp1_pose.pose.position.y = waypoint1.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp1_pose.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose.header.stamp = rclcpp::Clock().now();
    wp2_pose.header.frame_id = "map";
    wp2_pose.pose.position.x = waypoint2.x;
    wp2_pose.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp2_pose.pose.orientation = tf2::toMsg(q);
    
    //Intesection point
    Point ego_pose1,ego_pose2;
    ego_pose1.x = current_pose_.position.x;
    ego_pose1.y = current_pose_.position.y;

    ego_pose2.x = current_pose_.position.x +
      (5 * cos(tf2::getYaw(current_pose_.orientation)));
    ego_pose2.y = current_pose_.position.y +
      (5 * sin(tf2::getYaw(current_pose_.orientation)));
    Point intersection_point = findIntersection(waypoint1,waypoint2,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * cos(tf2::getYaw(current_pose_.orientation) + 3.141));
    intersection_point.y = intersection_point.y +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * sin(tf2::getYaw(current_pose_.orientation) + 3.141));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = current_pose_.orientation;

    //Publish way points
    //checkpoint_pub_->publish(inter_pose);
    //checkpoint_pub_->publish(wp1_pose);
    //checkpoint_pub_->publish(wp2_pose);
    
    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->waypoints.push_back(inter_pose.pose);
    route->waypoints.push_back(wp1_pose.pose);
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);

    //Debug waypoint array
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;  //intersection pose
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose.pose.position;    //wp1
    pos.orientation = wp1_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose.pose.position;     //wp2
    pos.orientation = wp2_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    debug_checkpoint_array_pub_->publish(pose_array);
  }
  else //Fwd parking
  {
    //    p4           p3
    //     _____________
    //    |             |
    // w1 |   Goal      |
    // *  |   *         |
    //    |_____________|
    //    p1           p2
    {
      //goalPoint
      goal_pose.x = mid_point.x + ((vehicle_shape_.base2back + 0.3) * cos(goal_angle_offset ));
      goal_pose.y = mid_point.y + ((vehicle_shape_.base2back + 0.3)* sin(goal_angle_offset ));

      //w1 point
      waypoint1.x = mid_point.x + ( (-5 ) * cos(goal_angle_offset));
      waypoint1.y = mid_point.y + ( (-5 ) * sin(goal_angle_offset));
      waypoint1.x = waypoint1.x + (3  * cos(goal_angle_offset + 1.57));
      waypoint1.y = waypoint1.y + (3  * sin(goal_angle_offset + 1.57));

      //w2 point
      waypoint2.x = mid_point.x + ( (-2 * vehicle_shape_.wheel_base) * cos(goal_angle_offset));
      waypoint2.y = mid_point.y + ( (-2 * vehicle_shape_.wheel_base) * sin(goal_angle_offset));
    }
    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);

    //Intesection point
    Point ego_pose1,ego_pose2,mid_point;
    ego_pose1.x = current_pose_.position.x;
    ego_pose1.y = current_pose_.position.y;

    ego_pose2.x = current_pose_.position.x +
      (5 * cos(tf2::getYaw(current_pose_.orientation)));
    ego_pose2.y = current_pose_.position.y +
      (5 * sin(tf2::getYaw(current_pose_.orientation)));
    mid_point.x = (point1.x + point4.x)/2;
    mid_point.y = (point1.y + point4.y)/2;
    Point intersection_point = findIntersection(mid_point,goal_pose,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * cos(tf2::getYaw(current_pose_.orientation)));
    intersection_point.y = intersection_point.y +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * sin(tf2::getYaw(current_pose_.orientation)));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = current_pose_.orientation;

    //Waypoint pose 1
    wp1_pose.header.stamp = rclcpp::Clock().now();
    wp1_pose.header.frame_id = "map";
    wp1_pose.pose.position.x = waypoint1.x;
    wp1_pose.pose.position.y = waypoint1.y;
    q.setRPY(0, 0, goal_angle_offset - 0.61); //35deg
    q.normalize();
    wp1_pose.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose.header.stamp = rclcpp::Clock().now();
    wp2_pose.header.frame_id = "map";
    wp2_pose.pose.position.x = waypoint2.x;
    wp2_pose.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    wp2_pose.pose.orientation = tf2::toMsg(q);

    //Publish way points
    //checkpoint_pub_->publish(inter_pose);
    //checkpoint_pub_->publish(wp1_pose_);
    //checkpoint_pub_->publish(wp2_pose_);

    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->waypoints.push_back(inter_pose.pose);
    route->waypoints.push_back(wp1_pose.pose);
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);

    //Debug
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose.pose.position;
    pos.orientation = wp1_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose.pose.position;
    pos.orientation = wp2_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    debug_checkpoint_array_pub_->publish(pose_array);
  }
  return end_goal_pose;
}

PoseStamped AvpPlanner::planFrontPark(Point point1,Point point2,
    Point point3,Point point4,int parking_direction,double goal_angle_offset)
{
  (void) point2;
  (void) point3;
  PoseStamped inter_pose, end_goal_pose;
  Pose pos;
  PoseArray pose_array;
  tf2::Quaternion q;
  Point goal_pose,waypoint1,waypoint2;

  if(parking_direction == 0)//Reverse parking
  {
    //     p1           p2
    //      _____________
    //     |             |
    //     |         Goal|
    //     |        *    |
    //     |_____________|            
    //     p4           p3

    //Find goal
    {
      //GoalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* cos(goal_angle_offset + 3.141));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* sin(goal_angle_offset + 3.141));
    }

    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);
    
    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);
  }
  else //Fwd parking
  {
    //    p4           p3
    //     _____________
    //    |             |
    // w1 |   Goal      |
    // *  |   *         |
    //    |_____________|            
    //    p1           p2
    {
      //goalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + 0.3) * cos(goal_angle_offset ));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + 0.3)* sin(goal_angle_offset ));
    }

    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);

    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);
  }
  return end_goal_pose;
}

PoseStamped AvpPlanner::planBackPark(Point point1,Point point2,Point point3,
    Point point4,int parking_direction,double goal_angle_offset)
{
  (void) point2;
  (void) point3;
  PoseStamped inter_pose, end_goal_pose;
  Pose pos;
  PoseArray pose_array;
  tf2::Quaternion q;
  Point goal_pose,waypoint1,waypoint2;

  if(parking_direction == 0)//Reverse parking
  {
    //     p1           p2
    //      _____________
    //     |             |
    //     |         Goal|
    //     |        *    |
    //     |_____________|            
    //     p4           p3

    //Find goal
    {
      //GoalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* cos(goal_angle_offset + 3.141));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* sin(goal_angle_offset + 3.141));
    }

    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);

    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);
  }
  else //Fwd parking
  {
    //    p4           p3
    //     _____________
    //    |             |
    // w1 |   Goal      |
    // *  |   *         |
    //    |_____________|            
    //    p1           p2

    {
      //goalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + 0.3) * cos(goal_angle_offset ));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + 0.3)* sin(goal_angle_offset ));
    }

    //End goal pose
    end_goal_pose.header.stamp = rclcpp::Clock().now();
    end_goal_pose.header.frame_id = "map";
    end_goal_pose.pose.position.x = goal_pose.x;
    end_goal_pose.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose.pose.orientation = tf2::toMsg(q);
    //goal_pose_pub_->publish(end_goal_pose);

    //Send updated route to rounting api module
    const auto route = std::make_shared<SetRoutePoints::Service::Request>();
    route->header.stamp = rclcpp::Clock().now();
    route->header.frame_id = "map";
    route->goal = end_goal_pose.pose;
    route->option.allow_goal_modification = false;
    cli_route_->async_send_request(route);
  }
  return end_goal_pose;
}

PoseStamped AvpPlanner::findParkingGoalPose(Point point1,Point point2,Point point3,
    Point point4,int parking_direction,int parkinglot_side)
{
  PoseStamped goal_pose;
  double slope_angle = 0.0,goal_angle_offset = 0.0;

  //End goal pose angle wrt to p1 & p4
  slope_angle = getSlopeWithRefPlane(point1,point4);
  RCLCPP_INFO(get_logger(),"The slope of p1-p4 wrt (x-axis) %0.20f",slope_angle * 180/3.141);

  //Perpendicular parking
  goal_angle_offset = getGoalAngleOffset(parking_direction,parkinglot_side,slope_angle);
  RCLCPP_INFO(get_logger(),"PerpendicularPark goal offset %lf",goal_angle_offset * 180/3.141);
  checkGoalAngleOffset(parkinglot_side,point1,point4,&goal_angle_offset);

  switch(parkinglot_side)
  {
    case 0://left
      goal_pose = planLeftPark(point1,point2,point3,point4,parking_direction,goal_angle_offset);
      break;
    case 1://Right
      goal_pose = planRightPark(point1,point2,point3,point4,parking_direction,goal_angle_offset);
      break;
    case 2://front
      goal_pose = planFrontPark(point1,point2,point3,point4,parking_direction,goal_angle_offset);
      break;
    case 3://back
      goal_pose = planBackPark(point1,point2,point3,point4,parking_direction,goal_angle_offset);
      break;
    default:
      RCLCPP_ERROR(get_logger(),"Invalid parking lot side!!!");
      break;
  }
  return goal_pose;
}

int AvpPlanner::findBestParkingLot(const std::string parking_lot_id)
{
  Point point1,point2,point3,point4,mid_point;
  double min_distance = FLT_MAX;
  int selected_parking_lot_id = -1;

  if(parkinglot_arr_ptr_)
  {
    for(int i=0; i < parkinglot_arr_ptr_->detect_slot_num;i++)
    {
      if(parkinglot_arr_ptr_->detect_slots[i].available_state)
      {
        point1 = parkinglot_arr_ptr_->detect_slots[i].point1;
        point2 = parkinglot_arr_ptr_->detect_slots[i].point2;
        point3 = parkinglot_arr_ptr_->detect_slots[i].point3;
        point4 = parkinglot_arr_ptr_->detect_slots[i].point4;

        mid_point.x = (point1.x + point4.x)/2;
        mid_point.y = (point1.y + point4.y)/2;

        if(isInParkingLot(parking_lot_id,mid_point))
        { 
          double distance = tier4_autoware_utils::calcDistance2d(mid_point , current_pose_);
          min_distance = std::min(min_distance,distance);
          if(min_distance == distance)
          {
            selected_parking_lot_id = i;
          }
        }
      }
    }
  }
  return selected_parking_lot_id;
}

void AvpPlanner::computeParkingGoalPose(int id)
{
  Point point1,point2,point3,point4,mid_point;
  PoseStamped goal_pose;

  int parking_direction = 0; //Reverese Parking by default
  int parkinglot_side = 0;
  
  {
    if(parkinglot_arr_ptr_->detect_slots[id].available_state)
    {
      point1 = parkinglot_arr_ptr_->detect_slots[id].point1;
      point2 = parkinglot_arr_ptr_->detect_slots[id].point2;
      point3 = parkinglot_arr_ptr_->detect_slots[id].point3;
      point4 = parkinglot_arr_ptr_->detect_slots[id].point4;

      parkinglot_side = parkinglot_arr_ptr_->detect_slots[id].direction;
      goal_pose = findParkingGoalPose(point1,point2,point3,point4,
                        parking_direction,parkinglot_side);
    }
  }
}
