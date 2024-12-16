// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "freespace_planner/freespace_planner_node.hpp"

#include "freespace_planning_algorithms/abstract_algorithm.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::PlannerWaypoint;
using freespace_planning_algorithms::PlannerWaypoints;
using freespace_planning_algorithms::RRTStar;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using tier4_planning_msgs::msg::Scenario;

bool isActive(const Scenario::ConstSharedPtr & scenario)
{
  if (!scenario) {
    return false;
  }

  const auto & s = scenario->activating_scenarios;
  if (std::find(std::begin(s), std::end(s), Scenario::PARKING) != std::end(s)) {
    return true;
  }

  return false;
}

PoseArray trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  // NAREN
  if(trajectory.points.size() <= 0)
  {
    return pose_array;
  }

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

std::vector<size_t> getReversingIndices(const Trajectory & trajectory)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    if (
      trajectory.points.at(i).longitudinal_velocity_mps *
        trajectory.points.at(i + 1).longitudinal_velocity_mps <
      0) {
      indices.push_back(i);
    }
  }

  return indices;
}

std::vector<size_t> getZeroVelocityIndices(const Trajectory & trajectory)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i)
  {
    if (trajectory.points.at(i).longitudinal_velocity_mps == 0) 
    {
      indices.push_back(i);
    }
  }

  return indices;
}

void setZeroVelocityIndices(Trajectory & trajectory, size_t index)
{
  while(index < trajectory.points.size())
  {
    trajectory.points.at(index).longitudinal_velocity_mps = 0;
    index++;
  }
}

size_t getNextTargetIndex(
  const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

Trajectory getPartialTrajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t end_index)
{
  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = rclcpp::Clock().now();

  // NAREN
  if(trajectory.points.size() <= 0)
  {
    return partial_trajectory;
  }

  partial_trajectory.points.reserve(trajectory.points.size());
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.points.push_back(trajectory.points.at(i));
  }

  // Modify velocity at start/end point
  if (partial_trajectory.points.size() >= 2) {
    partial_trajectory.points.front().longitudinal_velocity_mps =
      partial_trajectory.points.at(1).longitudinal_velocity_mps;
  }
  if (!partial_trajectory.points.empty()) {
    partial_trajectory.points.back().longitudinal_velocity_mps = 0;
  }

  return partial_trajectory;
}

double calcDistance2d(const Trajectory & trajectory, const Pose & pose)
{
  const auto idx = motion_utils::findNearestIndex(trajectory.points, pose.position);
  return tier4_autoware_utils::calcDistance2d(trajectory.points.at(idx), pose);
}

Pose transformPose(const Pose & pose, const TransformStamped & transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

Trajectory createTrajectory(
  const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  for (const auto & awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.longitudinal_velocity_mps = velocity / 3.6;      // velocity = const

    // switch sign by forward/backward
    point.longitudinal_velocity_mps = (awp.is_back ? -1 : 1) * point.longitudinal_velocity_mps;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

Trajectory createStopTrajectory(const PoseStamped & current_pose)
{
  PlannerWaypoints waypoints;
  PlannerWaypoint waypoint;

  waypoints.header.stamp = rclcpp::Clock().now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(current_pose, waypoints, 0.0);
}

Trajectory createStopTrajectory(const Trajectory & trajectory)
{
  Trajectory stop_trajectory = trajectory;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    stop_trajectory.points.at(i).longitudinal_velocity_mps = 0.0;
  }
  return stop_trajectory;
}

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

}  // namespace

namespace freespace_planner
{
FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_arrived_angle_deg = declare_parameter<double>("th_arrived_angle_deg");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
    p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
  }

  // set vehicle_info
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
    vehicle_wheel_base_ = vehicle_info.wheel_base_m;
    //vehicle_max_steer_angle_ = vehicle_info.max_steer_angle_rad;
    //Hardcoded the max steering to 28.89degs
    vehicle_max_steer_angle_ = 0.50;
  }

  // Planning
  initializePlanningAlgorithm();

  //NAREN: this is used to make route_sub_ subscription receive messages
  //even when the timer is consuming entire process bandwidth
  //We also made this node as mutithreaded excutor to support parallel
  //processing of route subscriber cb and timer cb for that we have modified
  //the launch script parking.launch.py to use container_mt(multithreaded)
  auto reentrant_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = reentrant_callback_group;

  // Subscribers
  {
    route_sub_ = create_subscription<LaneletRoute>(
      "~/input/route", rclcpp::QoS{1}.transient_local(),
      std::bind(&FreespacePlannerNode::onRoute, this, _1),options);
    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
      "~/input/occupancy_grid", 1, std::bind(&FreespacePlannerNode::onOccupancyGrid, this, _1));
    scenario_sub_ = create_subscription<Scenario>(
      "~/input/scenario", 1, std::bind(&FreespacePlannerNode::onScenario, this, _1));
    odom_sub_ = create_subscription<Odometry>(
      "~/input/odometry", 100, std::bind(&FreespacePlannerNode::onOdometry, this, _1));
  }

  // NAREN
  // Subscribers from motovis_interface module
  {
    update_goal_pose_cli_ = create_client<UpdateGoalPose>("/motovis_interface/update_goal_pose");
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
    debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
    parking_state_pub_ = create_publisher<std_msgs::msg::Bool>("is_completed", qos);

    checkpoint_array_pub_ = create_publisher<PoseArray>("/motovis_interface/debug/checkpoint_array", qos);
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&FreespacePlannerNode::onTimer, this));
  }
}

PlannerCommonParam FreespacePlannerNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");
  p.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");
  p.maximum_turning_radius = declare_parameter<double>("maximum_turning_radius");
  p.turning_radius_size = declare_parameter<int>("turning_radius_size");
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

//NAREN
void FreespacePlannerNode::getGoalPose()
{
  const auto callback = [this](rclcpp::Client<UpdateGoalPose>::SharedFuture future) {
      RCLCPP_INFO(get_logger(), "get latest End goal pose from motovis_interface");
      //Update just the goal pose alone not the waypoint
      //End goal pose is stored at the back of waypoints_
      if(waypoints_.size() > 0)
      {
        //waypoints_[waypoints_.size() - 1].pose.position = future.get()->goal_pose.pose.position;
        waypoints_[waypoints_.size() - 1].pose.orientation = future.get()->goal_pose.pose.orientation;
      }
  };

  const auto request = std::make_shared<UpdateGoalPose::Request>();
  update_goal_pose_cli_->async_send_request(request, callback);
}

void FreespacePlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;
  //NAREN : To stop planning whenever we receive a new goal pose
  algo_->stopPlanning();

  route_update_mtx.lock();
  waypoints_.clear();
  waypoint_count_ = 0;
  for (const auto & waypoint : msg->waypoints) {
    PoseStamped pose;
    pose.header = msg->header;
    pose.pose = waypoint;
    waypoints_.push_back(pose);
  }
  PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->goal_pose;
  waypoints_.push_back(pose);

  RCLCPP_INFO(get_logger(),"No of waypoints:%ld",waypoints_.size());
  if(waypoints_.size() > 0)
  {
    goal_pose_ = waypoints_.front();
    RCLCPP_INFO(get_logger(),"Current goal pose x:%f y:%f",goal_pose_.pose.position.x,goal_pose_.pose.position.y);
    waypoint_count_++;
    reset();
  }
  route_update_mtx.unlock();
}

void FreespacePlannerNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void FreespacePlannerNode::onScenario(const Scenario::ConstSharedPtr msg)
{
  isParallelParkingInProgress_ = false;
  scenario_ = msg;
}

void FreespacePlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
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

bool FreespacePlannerNode::isPlanRequired()
{
  if (trajectory_.points.empty()) {
    return true;
  }

  if (partial_trajectory_.points.empty()) {
    return true;
  }

  if (node_param_.replan_when_obstacle_found && !ignoreObstacleReplan_) {
    const size_t nearest_index_partial =
      motion_utils::findNearestIndex(partial_trajectory_.points, current_pose_.pose.position);
    size_t end_index_partial = partial_trajectory_.points.size() - 1;

    const auto forward_trajectory =
      getPartialTrajectory(partial_trajectory_, nearest_index_partial, end_index_partial);

    const bool is_obstacle_found =
      algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));
    if (is_obstacle_found) {
      //If there is obstacle or courseout happended
      //while doing 2 arc parallel parking 
      //we need to rerun parallelparking
      if(isParallelParkingInProgress_ == true)
      {
        isParallelParkingInProgress_ = false;
      }

      RCLCPP_INFO(get_logger(), "Found obstacle");
      return true;
    }
  }

  if (node_param_.replan_when_course_out) {
    const bool is_course_out =
      calcDistance2d(trajectory_, current_pose_.pose) > node_param_.th_course_out_distance_m;
    if (is_course_out) {
      //If there is obstacle or courseout happended
      //while doing 2 arc parallel parking 
      //we need to rerun parallelparking
      if(isParallelParkingInProgress_ == true)
      {
        isParallelParkingInProgress_ = false;
      }

      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

bool FreespacePlannerNode::isCloseTarget()
{
  const double yaw_pose = tf2::getYaw(current_pose_.pose.orientation);
  const double yaw_goal = tf2::getYaw(goal_pose_.pose.orientation);
  const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw_pose - yaw_goal);
  double dis_diff = tier4_autoware_utils::calcDistance2d(goal_pose_ , current_pose_);

  if((std::abs(yaw_diff * 180.0/3.14) <= node_param_.th_arrived_angle_deg) 
      && (dis_diff <= node_param_.th_arrived_distance_m))
  {
    RCLCPP_INFO(get_logger(),"Goal pose is nearer to current pose %fm %frad",dis_diff,yaw_diff);
    if(waypoint_count_ < waypoints_.size())
    {
      RCLCPP_INFO(get_logger(),"Switching to next waypoint");
      reset();
      goal_pose_ = waypoints_[waypoint_count_];
      waypoint_count_++;
    }
    else
    {
      // Finished publishing all partial trajectories
      isParallelParkingInProgress_ = false;
      is_completed_ = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
      std_msgs::msg::Bool is_completed_msg;
      is_completed_msg.data = is_completed_;
      parking_state_pub_->publish(is_completed_msg);

    }
    return true;
  }
  return false;
}

void FreespacePlannerNode::updateTargetIndex()
{
  const auto is_near_target =
    tier4_autoware_utils::calcDistance2d(trajectory_.points.at(target_index_), current_pose_) <
    node_param_.th_arrived_distance_m;

  const auto is_stopped = isStopped(odom_buffer_, node_param_.th_stopped_velocity_mps);
 
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "is_near_target %d, %f < %f",is_near_target,tier4_autoware_utils::calcDistance2d(trajectory_.points.at(target_index_), current_pose_),node_param_.th_arrived_distance_m);

  if (is_near_target && is_stopped)
  {
    const auto new_target_index =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    if (new_target_index == target_index_)
    {
      //NAREN
      if(waypoint_count_ < waypoints_.size())
      {
        RCLCPP_INFO(get_logger(),"Switching to next waypoint");
        reset();
        goal_pose_ = waypoints_[waypoint_count_];
        waypoint_count_++;
      }
      else
      {
        // Check angle at goal pose.
        const double yaw_pose = tf2::getYaw(current_pose_.pose.orientation);
        const double yaw_goal = tf2::getYaw(goal_pose_.pose.orientation);
        const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw_pose - yaw_goal);
        double dis_diff = tier4_autoware_utils::calcDistance2d(goal_pose_ , current_pose_);

        RCLCPP_INFO(get_logger(),"yaw diff %lf & dis diff %f",yaw_diff * 180.0 / 3.14,dis_diff);
        //Rerun to align to the final goal angle & distance in parking scenario
        if((std::abs(yaw_diff * 180.0/3.14) > node_param_.th_arrived_angle_deg) 
            || (dis_diff > node_param_.th_arrived_distance_m))
        {
          //For direct goal pose setting we nvr need to rerun
          if(waypoints_.size() > 0)
          {
            RCLCPP_INFO(get_logger(),"Re-Running the planner");
            reset();
            waypoint_count_--;
            //If the final park position is not aligned move 5 mtr front/back
            // based on scenario to re-align
            if( scenario_->current_scenario == "FWD")
            {
              goal_pose_.pose.position.x = goal_pose_.pose.position.x +
                (-5 * cos(tf2::getYaw(goal_pose_.pose.orientation)));
              goal_pose_.pose.position.y = goal_pose_.pose.position.y +
                (-5 * sin(tf2::getYaw(goal_pose_.pose.orientation)));
            }
            else if(scenario_->current_scenario == "PAR")
            {
              goal_pose_.pose.position.x = goal_pose_.pose.position.x +
                (1 * cos(tf2::getYaw(goal_pose_.pose.orientation)));
              goal_pose_.pose.position.y = goal_pose_.pose.position.y +
                (1 * sin(tf2::getYaw(goal_pose_.pose.orientation)));
            }
            else
            {
              goal_pose_.pose.position.x = goal_pose_.pose.position.x +
                (5 * cos(tf2::getYaw(goal_pose_.pose.orientation)));
              goal_pose_.pose.position.y = goal_pose_.pose.position.y +
                (5 * sin(tf2::getYaw(goal_pose_.pose.orientation)));
            }
          }
        }
        else
        {
          // Finished publishing all partial trajectories
          isParallelParkingInProgress_ = false;
          is_completed_ = true;
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
          std_msgs::msg::Bool is_completed_msg;
          is_completed_msg.data = is_completed_;
          parking_state_pub_->publish(is_completed_msg);
        }
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(),"Switching to next partial trajectory"); 
      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

void FreespacePlannerNode::onTimer()
{
  // Check all inputs are ready
  if (!occupancy_grid_ || !route_ || !scenario_ || !odom_) {
     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "All inputs are not ready!!!");
	 if(!occupancy_grid_)
	 {
       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Occupancy inputs are not ready!!!");
	 }
	 if(!route_)
	 {
       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "route inputs are not ready!!!");
	 }
	 if(!scenario_)
	 {
       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Scenario inputs are not ready!!!");
	 }
	 if(!odom_)
	 {
       RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Odom inputs are not ready!!!");
	 }
    return;
  }

  if (!isActive(scenario_)) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,"Scenario is not ready!!!");
    reset();
    return;
  }

  if (is_completed_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,"Parking completed!!!");
    return;
  }

  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,"Curent pose header frame id is empty!!!");
    return;
  }

  route_update_mtx.lock();

  // Provide robot shape and map for the planner
  algo_->setMap(*occupancy_grid_);

  if (isPlanRequired()) {
    // Stop before planning new trajectory
    const auto stop_trajectory = partial_trajectory_.points.empty()
                                   ? createStopTrajectory(current_pose_)
                                   : createStopTrajectory(partial_trajectory_);
    trajectory_pub_->publish(stop_trajectory);
    debug_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
    debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));

    reset();
    
    //NAREN
    //Check if current pose is close to goal pose
    //if so move to next way poinor if it is the final goal
    //stop planning and make freespace planning completed
    if(isCloseTarget() == true)
    {
      route_update_mtx.unlock();
      return;
    }

    // Plan new trajectory
    planTrajectory();
  }

  // StopTrajectory
  if (trajectory_.points.size() <= 1) {
    route_update_mtx.unlock();
    return;
  }

  // Update partial trajectory
  updateTargetIndex();
  partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);

  //NAREN:Check for obstacle in trajectory and add zero velocity a point before the obstacle
  //index. So Ego can stop at well maintained distance before the obstacle
  size_t obs_index = algo_->getObstacleIndexOnTrajectory(trajectory2PoseArray(partial_trajectory_));
  if(obs_index < partial_trajectory_.points.size())
  {
    RCLCPP_INFO(get_logger(),"obx idx: %ld %ld",obs_index,partial_trajectory_.points.size());
    setZeroVelocityIndices(partial_trajectory_, obs_index - 1);
  }

  // Publish messages
  trajectory_pub_->publish(partial_trajectory_);
  debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory_));
  debug_partial_pose_array_pub_->publish(trajectory2PoseArray(partial_trajectory_));
  route_update_mtx.unlock();
}

std::vector<Trajectory> FreespacePlannerNode::generatePullOverPaths(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
  const bool is_forward, const double end_pose_offset, const double velocity)
{
  std::vector<Trajectory> arc_paths;
  const double arc_path_interval = 0.1;
  
  const auto goal_pose_in_base_link_frame = transformPose(
      goal_pose_.pose, getTransform("base_link", "map"));
  RCLCPP_INFO(get_logger(),"goal_pose_in_base_link_frame x:%lf y:%lf",goal_pose_in_base_link_frame.position.x,goal_pose_in_base_link_frame.position.y);
    
  //    Y
  //    |
  //    | EgoPose
  //    |_____X
  //
  //
  //    *Goal Pose (y<0) Right 

  if(goal_pose_in_base_link_frame.position.y < 0)
  {
    RCLCPP_INFO(get_logger(),"ParkingLot is on Right side , doing planOneTrialRight");
    //Parking lot is on right side
    arc_paths = planOneTrialRight(
    start_pose, goal_pose, R_E_r , is_forward, end_pose_offset,arc_path_interval);
  }
  else
  {
    RCLCPP_INFO(get_logger(),"ParkingLot is on Left side , doing planOneTrialLeft");
    //Parking lot is on left side
    arc_paths = planOneTrialLeft(
    start_pose, goal_pose, R_E_r , is_forward, end_pose_offset,arc_path_interval);
  }

  if (arc_paths.empty()) {
    RCLCPP_ERROR(get_logger(),"Parallel 2 Arc path empty!!!");
    return std::vector<Trajectory>{};
  }
  
  // set parking velocity and stop velocity at the end of the path
  constexpr bool set_stop_end = true;
  setVelocityToArcPaths(arc_paths, velocity, set_stop_end);

  return arc_paths;
}

std::vector<Trajectory> FreespacePlannerNode::planOneTrialRight(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_l,
  const bool is_forward, const double end_pose_offset,
  const double arc_path_interval)
{
  std::vector<Trajectory> paths;
  const Pose arc_end_pose = tier4_autoware_utils::calcOffsetPose(goal_pose, end_pose_offset, 0, 0);
  const double self_yaw = tf2::getYaw(start_pose.orientation);
  const double goal_yaw = tf2::getYaw(arc_end_pose.orientation);
  const double psi = tier4_autoware_utils::normalizeRadian(self_yaw - goal_yaw);

  const Pose Cl = tier4_autoware_utils::calcOffsetPose(arc_end_pose, 0, R_E_l, 0);
  const double d_Cl_Einit = tier4_autoware_utils::calcDistance2d(Cl, start_pose);
 
  const Point Cl_goal_coords = tier4_autoware_utils::inverseTransformPoint(Cl.position, arc_end_pose);
  const Point self_point_goal_coords = tier4_autoware_utils::inverseTransformPoint(start_pose.position, arc_end_pose);

  const double alpha = psi + std::acos((self_point_goal_coords.y - Cl_goal_coords.y) / d_Cl_Einit);

  const double R_E_r =
    (std::pow(d_Cl_Einit, 2) - std::pow(R_E_l, 2)) / (2 * (R_E_l + d_Cl_Einit * std::cos(alpha)));
  if (R_E_r <= 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,"R_E_r %lf < 0, R_E_l=%lf!!!",R_E_r,R_E_l);
    return std::vector<Trajectory>{};
  }

  // Generate arc path(right turn -> left turn)
  const Pose Cr = tier4_autoware_utils::calcOffsetPose(start_pose, 0, -R_E_r, 0);
  double theta_r = std::acos(
    (std::pow(R_E_r, 2) + std::pow(R_E_r + R_E_l, 2) - std::pow(d_Cl_Einit, 2)) /
    (2 * R_E_r * (R_E_r + R_E_l)));

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,"alpha=%lf R_E_r %lf, theta_r=%lf psi=%lf!!!",alpha,R_E_r,theta_r*180/3.141,psi * 180/3.141);
        
  theta_r = is_forward ? -theta_r : theta_r;

  Trajectory path_turn_right = generateArcPath(
      Cr, R_E_r, M_PI_2, tier4_autoware_utils::normalizeRadian(M_PI_2 + theta_r),arc_path_interval, true,
      false);

  Trajectory path_turn_left = generateArcPath(
     Cl, R_E_l, tier4_autoware_utils::normalizeRadian(-M_PI_2 + psi + theta_r), -M_PI_2, arc_path_interval, false,false);

  // Need to add straight path to last right_turning for parking in parallel
  if (std::abs(end_pose_offset) > 0) {
    TrajectoryPoint straight_point{};
    straight_point.pose = goal_pose;
    path_turn_right.points.push_back(straight_point);
  }
  // generate arc path vector
  paths.push_back(path_turn_right);
  paths.push_back(path_turn_left);

  return paths;
}

std::vector<Trajectory> FreespacePlannerNode::planOneTrialLeft(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
  const bool is_forward, const double end_pose_offset,
  const double arc_path_interval)
{
  std::vector<Trajectory> paths;
  const Pose arc_end_pose = tier4_autoware_utils::calcOffsetPose(goal_pose, end_pose_offset, 0, 0);
  const double self_yaw = tf2::getYaw(start_pose.orientation);
  const double goal_yaw = tf2::getYaw(arc_end_pose.orientation);
  const double psi = tier4_autoware_utils::normalizeRadian(self_yaw - goal_yaw);

  const Pose Cr = tier4_autoware_utils::calcOffsetPose(arc_end_pose, 0, -R_E_r, 0);
  const double d_Cr_Einit = tier4_autoware_utils::calcDistance2d(Cr, start_pose);
 
  const Point Cr_goal_coords = tier4_autoware_utils::inverseTransformPoint(Cr.position, arc_end_pose);
  const Point self_point_goal_coords = tier4_autoware_utils::inverseTransformPoint(start_pose.position, arc_end_pose);

  const double alpha =
    M_PI_2 - psi + std::asin((self_point_goal_coords.y - Cr_goal_coords.y) / d_Cr_Einit);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,"arc_end_pose x=%lf y=%lf",arc_end_pose.position.x,arc_end_pose.position.y);

  const double R_E_l =
    (std::pow(d_Cr_Einit, 2) - std::pow(R_E_r, 2)) / (2 * (R_E_r + d_Cr_Einit * std::cos(alpha)));
  if (R_E_l <= 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,"R_E_l %lf < 0, R_E_r=%lf!!!",R_E_l,R_E_r);
    return std::vector<Trajectory>{};
  }

  // Generate arc path(left turn -> right turn)
  const Pose Cl = tier4_autoware_utils::calcOffsetPose(start_pose, 0, R_E_l, 0);
  double theta_l = std::acos(
    (std::pow(R_E_l, 2) + std::pow(R_E_l + R_E_r, 2) - std::pow(d_Cr_Einit, 2)) /
    (2 * R_E_l * (R_E_l + R_E_r)));

  theta_l = is_forward ? theta_l : -theta_l;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,"R_E_r %lf, theta_l=%lf psi=%lf!!!",R_E_l,theta_l*180/3.141,psi * 180/3.141);
  Trajectory path_turn_left = generateArcPath(
    Cl, R_E_l, -M_PI_2, tier4_autoware_utils::normalizeRadian(-M_PI_2 + theta_l), arc_path_interval, is_forward,
    is_forward);

  Trajectory path_turn_right = generateArcPath(
    Cr, R_E_r, tier4_autoware_utils::normalizeRadian(psi + M_PI_2 + theta_l), M_PI_2, arc_path_interval, !is_forward,
    is_forward);

  // Need to add straight path to last right_turning for parking in parallel
  if (std::abs(end_pose_offset) > 0) {
    TrajectoryPoint straight_point{};
    straight_point.pose = goal_pose;
    path_turn_right.points.push_back(straight_point);
  }

  // generate arc path vector
  paths.push_back(path_turn_left);
  paths.push_back(path_turn_right);
  
  return paths;
}

Trajectory FreespacePlannerNode::generateArcPath(
  const Pose & center, const double radius, const double start_yaw, double end_yaw,
  const double arc_path_interval,
  const bool is_left_turn,  // is_left_turn means clockwise around center.
  const bool is_forward)
{
  Trajectory path;
  const double yaw_interval = arc_path_interval / radius;
  double yaw = start_yaw;
  RCLCPP_INFO(get_logger(),"yaw_interval %lf",yaw_interval);
  if (is_left_turn) {
    if (end_yaw < start_yaw) end_yaw += M_PI_2;
    while (yaw < end_yaw) {
      RCLCPP_INFO(get_logger(),"left turn yaw %lf",yaw * 180/3.141);
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw += yaw_interval;
    }
  } else {  // right_turn
    if (end_yaw > start_yaw) end_yaw -= M_PI_2;
    while (yaw > end_yaw) {
      RCLCPP_INFO(get_logger(),"right turn yaw %lf",yaw * 180/3.141);
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw -= yaw_interval;
    }
  }

  // insert the last point exactly
  const auto p = generateArcPathPoint(center, radius, end_yaw, is_left_turn, is_forward);
  constexpr double min_dist = 0.01;
  if (path.points.empty() || tier4_autoware_utils::calcDistance2d(path.points.back(), p) > min_dist) {
    path.points.push_back(p);
  }

  return path;
}

TrajectoryPoint FreespacePlannerNode::generateArcPathPoint(
  const Pose & center, const double radius, const double yaw, const bool is_left_turn,
  const bool is_forward)
{
  // get pose in center_pose coords
  Pose pose_center_coords;
  pose_center_coords.position.x = radius * std::cos(yaw);
  pose_center_coords.position.y = radius * std::sin(yaw);

  // set orientation
  tf2::Quaternion quat;
  if ((is_left_turn && !is_forward) || (!is_left_turn && is_forward)) {
    quat.setRPY(0, 0, tier4_autoware_utils::normalizeRadian(yaw - M_PI_2));
    RCLCPP_INFO(get_logger(),"left final yaw %lf",(yaw - M_PI_2)* 180/3.141);
  } else {
    quat.setRPY(0, 0, tier4_autoware_utils::normalizeRadian(yaw + M_PI_2));
    RCLCPP_INFO(get_logger(),"right final yaw %lf",(yaw + M_PI_2)* 180/3.141);
  }
  pose_center_coords.orientation = tf2::toMsg(quat);

  // get pose in map coords
  TrajectoryPoint p{};
  p.pose = tier4_autoware_utils::transformPose(pose_center_coords, center);

  return p;
}

void FreespacePlannerNode::setVelocityToArcPaths(
  std::vector<Trajectory> & arc_paths, const double velocity, const bool set_stop_end)
{
  for (auto & path : arc_paths) {
    for (size_t i = 0; i < path.points.size(); i++) {
      if (i == path.points.size() - 1 && set_stop_end) {
        // stop point at the end of the path
        path.points.at(i).longitudinal_velocity_mps = 0.0;
      } else {
        path.points.at(i).longitudinal_velocity_mps = velocity;
      }
    }
  }
}

void FreespacePlannerNode::planTrajectory()
{
  if (occupancy_grid_ == nullptr) {
    return;
  }

  //Parallel planning happens only when parallel park scenario is enabled
  //and when we try to plan only for goal pose , if we have waypoints
  //do normal planning via freespace planner
  //And if the parking is not already in progress
  if((scenario_->current_scenario == "PAR")
      && (waypoint_count_ == waypoints_.size()) && !isParallelParkingInProgress_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,"Parallel Parking!!!");
    reset();

    // Calculate poses in costmap frame
    const auto current_pose_in_costmap_frame = transformPose(
        current_pose_.pose,
        getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

    const auto goal_pose_in_costmap_frame = transformPose(
        goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

    double R_E_min = vehicle_wheel_base_/std::tan(vehicle_max_steer_angle_);
    bool is_forward = false;
    const double  end_pose_offset = 0.0;
    const double velocity = -1.1;

    //Geomentric parallel parking
    std::vector<Trajectory> paths = generatePullOverPaths(
        current_pose_in_costmap_frame, goal_pose_in_costmap_frame, R_E_min,
        is_forward, end_pose_offset,velocity);
    if(paths.empty())
    {
      RCLCPP_ERROR(get_logger(),"GeneratePullOverPaths trajectory path is empty!!!");
      return;
    }

    //Trajectory
    trajectory_.header.stamp = rclcpp::Clock().now();
    trajectory_.header.frame_id = current_pose_.header.frame_id;
    for(auto &path : paths)
    {
      for (size_t i = 0; i < path.points.size(); i++)
      {
        trajectory_.points.push_back(path.points.at(i));
      }
    }

    reversing_indices_ = getZeroVelocityIndices(trajectory_);
    prev_target_index_ = 0;
    target_index_ =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

    //Find if there is obstacle in planned route
    //if obstacle found on planned trajectory then reset the tarj
    const bool is_obstacle_found =
      algo_->hasObstacleOnTrajectory(trajectory2PoseArray(trajectory_));
    if (is_obstacle_found) {
      reset();
      RCLCPP_INFO(get_logger(), "Found obstacle");
      return;
    }

    //Set parallel parking in progress
    isParallelParkingInProgress_ = true;
  }
  else
  {
    // Calculate poses in costmap frame
    const auto current_pose_in_costmap_frame = transformPose(
        current_pose_.pose,
        getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

    const auto goal_pose_in_costmap_frame = transformPose(
        goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

    // execute planning
    const rclcpp::Time start = get_clock()->now();
    RCLCPP_INFO(get_logger(),"Makeplan Goal pose x:%f y:%f",goal_pose_.pose.position.x,goal_pose_.pose.position.y);
    const bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
    const rclcpp::Time end = get_clock()->now();

    RCLCPP_INFO(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

    if (result) {
      RCLCPP_INFO(get_logger(), "Found goal!");
      trajectory_ =
        createTrajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
      reversing_indices_ = getReversingIndices(trajectory_);
      prev_target_index_ = 0;
      target_index_ =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

    } else {
      RCLCPP_INFO(get_logger(), "Can't find goal...");
      reset();
      //Fix to park ego when its nearer to a curb while parking
      //check is added for final end goal pose alone
      if(waypoint_count_ == waypoints_.size())
      {
        RCLCPP_INFO(get_logger(), "Retry with empty obstacle consideration....");
        algo_->resetObstacleTable();
        const bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
        if (result) {
          RCLCPP_INFO(get_logger(), "Found goal with no obstacle!!!");
          algo_->setMap(*occupancy_grid_);
          trajectory_ =
            createTrajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
          reversing_indices_ = getReversingIndices(trajectory_);
          prev_target_index_ = 0;
          target_index_ =
            getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);
          //Ignore the Obstacle replan as we are planning the trajectory
          //without any obstacles in consideration
          ignoreObstacleReplan_ = true;
        }
      }
    }
  }
}

void FreespacePlannerNode::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  ignoreObstacleReplan_= false;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);
}

TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else if (algo_name == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}
}  // namespace freespace_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(freespace_planner::FreespacePlannerNode)
