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

#include "motovis_interface/motovis_interface.hpp"
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
using Direction=int;
struct PlotCoordinate
{
  double x;
  double y;
};

struct ParkingLot_s
{
  std::array<PlotCoordinate,4> p;
  Direction d;
};

#ifdef AVP
//For testing the AVP dummy parking lots
//uncomment plot1,2,3 below if testing avp
std::array<PlotCoordinate,4> plot1 = {{{14,-3},{14,-9},{17,-9},{17,-3}}};
std::array<PlotCoordinate,4> plot2 = {{{6,-3},{6,-9},{9,-9},{9,-3}}};
std::array<PlotCoordinate,4> plot3 = {{{0,3},{0,9},{3,9},{3,3}}};
std::array<ParkingLot_s , 3> parking_lots {
  {
    {plot1,1},
      {plot2,1},
      {plot3,0}
  }
};
#else
std::array<PlotCoordinate,4> plot1 = {{{10,-11},{10,-17},{13,-17},{13,-11}}};
std::array<PlotCoordinate,4> plot2 = {{{10,-11},{10,-17},{7,-17},{7,-11}}};
std::array<PlotCoordinate,4> plot3 = {{{7,-11},{7,-17},{4,-17},{4,-11}}};
std::array<PlotCoordinate,4> plot4 = {{{4,-11},{4,-17},{1,-17},{1,-11}}};

std::array<PlotCoordinate,4> plot5 = {{{10,11},{10,17},{13,17},{13,11}}};
std::array<PlotCoordinate,4> plot6 = {{{10,11},{10,17},{7,17},{7,11}}};
std::array<PlotCoordinate,4> plot7 = {{{7,11},{7,17},{4,17},{4,11}}};
std::array<PlotCoordinate,4> plot8 = {{{4,11},{4,17},{1,17},{1,11}}};

std::array<PlotCoordinate,4> plot9 = {{{-10,5},{-10,11},{-13,11},{-13,5}}};
std::array<PlotCoordinate,4> plot10 = {{{-10,5},{-10,11},{-7,11},{-7,5}}};
std::array<PlotCoordinate,4> plot11 = {{{-7,5},{-7,11},{-4,11},{-4,5}}};
std::array<PlotCoordinate,4> plot12 = {{{-4,5},{-4,11},{-1,11},{-1,5}}};

std::array<PlotCoordinate,4> plot13 = {{{-10,-5},{-10,-11},{-13,-11},{-13,-5}}};
std::array<PlotCoordinate,4> plot14 = {{{-10,-5},{-10,-11},{-7,-11},{-7,-5}}};
std::array<PlotCoordinate,4> plot15 = {{{-7,-5},{-7,-11},{-4,-11},{-4,-5}}};
std::array<PlotCoordinate,4> plot16 = {{{-4,-5},{-4,-11},{-1,-11},{-1,-5}}};

//Front and back parking
std::array<PlotCoordinate,4> plot17 = {{{-10,-1.5},{-16,-1.5},{-16,1.5},{-10,1.5}}};
std::array<PlotCoordinate,4> plot18 = {{{10,1.5},{16,1.5},{16,-1.5},{10,-1.5}}};

//Parallel parking
std::array<PlotCoordinate,4> plot19 = {{{3,-2},{3,-5},{-3,-5},{-3,-2}}};
std::array<PlotCoordinate,4> plot20 = {{{13,-2},{13,-5},{7,-5},{7,-2}}};
std::array<PlotCoordinate,4> plot21 = {{{19,-2},{19,-5},{13,-5},{13,-2}}};

std::array<PlotCoordinate,4> plot22 = {{{3,2},{3,5},{-3,5},{-3,2}}};
std::array<PlotCoordinate,4> plot23 = {{{13,2},{13,5},{7,5},{7,2}}};
std::array<PlotCoordinate,4> plot24 = {{{19,2},{19,5},{13,5},{13,2}}};

std::array<ParkingLot_s , 24> parking_lots {
  {
    {plot1,1},
      {plot2,1},
      {plot3,1},
      {plot4,1},
      {plot5,0},
      {plot6,0},
      {plot7,0},
      {plot8,0},
      {plot9,0},
      {plot10,0},
      {plot11,0},
      {plot12,0},
      {plot13,1},
      {plot14,1},
      {plot15,1},
      {plot16,1},
      {plot17,3},
      {plot18,2},
      {plot19,1},
      {plot20,1},
      {plot21,1},
      {plot22,0},
      {plot23,0},
      {plot24,0}
  }
};
#endif

MotovisInterface::MotovisInterface(const std::string & node_name, const rclcpp::NodeOptions & node_options)
  : rclcpp::Node(node_name, node_options) , tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.num_of_cams = declare_parameter("num_of_cams", 4);
  node_param_.use_simulated_parking = declare_parameter("use_simulated_parking", true);
  node_param_.share_bev_to_disti_hmi = declare_parameter("share_bev_to_disti_hmi", false);
  node_param_.use_vehicle_overlay_on_bev = declare_parameter("use_vehicle_overlay_on_bev", false);
  node_param_.background_png_file = declare_parameter("background_png_file", "background.png");
  node_param_.vehicle_png_file = declare_parameter("vehicle_png_file", "vehcile_overlay.png");
 node_param_.publish_bev = declare_parameter("publish_bev", false);
  
  parkinglot_fixed_ = false;
  selected_parking_lot_pose_.x = FLT_MAX;
  selected_parking_lot_pose_.y = FLT_MAX;
  selected_parkinglot_marker_ = Marker();
  goal_angle_offset_ = FLT_MAX;
  end_goal_pose_.pose.position.x = FLT_MAX;
  end_goal_pose_.pose.position.y = FLT_MAX;
  wp1_pose_.pose.position.x = FLT_MAX;
  wp1_pose_.pose.position.y = FLT_MAX;
  wp2_pose_.pose.position.x = FLT_MAX;
  wp2_pose_.pose.position.y = FLT_MAX;

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    debug_freespace_pub_ = create_publisher<PolygonStamped>("~/debug/polygon", qos);
    debug_parkinglot_marker_pub_ = create_publisher<MarkerArray>("~/debug/marker_array", qos);
    parkinglot_pub_ = create_publisher<ParkingLots>("~/parking_lots", qos);
    goal_pose_pub_ = create_publisher<PoseStamped>("/planning/mission_planning/goal", qos);
    checkpoint_pub_ = create_publisher<PoseStamped>("/planning/mission_planning/checkpoint", qos);
    updated_goal_pose_pub_ = create_publisher<PoseStamped>("/planning/mission_planning/updated_goal_pose", qos);
    checkpoint_array_pub_ = create_publisher<PoseArray>("~/debug/checkpoint_array", qos);
    bev_publisher_ = create_publisher<sensor_msgs::msg::Image>("/image",1);

    //Debug
    pub_debug_velocity_report_ = create_publisher<VelocityReport>("/motovis_interface/debug/velocity_report",1);
  }

  // set vehicle_info
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
    vehicle_shape_.wheel_base = vehicle_info.wheel_base_m;
  }

  // Services
  {
    goal_pose_srv_ = create_service<UpdateGoalPose>(
        "/motovis_interface/update_goal_pose",
        std::bind(&MotovisInterface::onGoalPoseRequest, this, _1, _2));
  }

  // subscriber
  {
    odometry_ = nullptr;
    sub_odometry_ = create_subscription<Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
        std::bind(&MotovisInterface::on_odometry, this, std::placeholders::_1));
    parking_lot_pose_sub_ = create_subscription<PoseStamped>(
        "/rviz2/parkinglot/pose", 1, std::bind(&MotovisInterface::onParkingLotPose, this, _1));
    scenario_sub_ = create_subscription<Scenario>(
        "/planning/scenario_planning/scenario", 1, std::bind(&MotovisInterface::onScenario, this, _1));
    show_parking_sub_ = create_subscription<Bool>(
        "/perception/motovis_interface/show_parking_lot", 1, std::bind(&MotovisInterface::onShowParkingLots, this, _1));
  }

  //Status Report from pacmod3   
  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
        this, "/pacmod/steering_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>(
        this, "/pacmod/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
      this, "/pacmod/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
      this, "/pacmod/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
      this, "/pacmod/shift_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>>(
      this, "/pacmod/global_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
      this, "/pacmod/turn_rpt");

  pacmod_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
        PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,*brake_rpt_sub_, *shift_rpt_sub_, *global_rpt_sub_, *turn_rpt_sub_);

  pacmod_feedbacks_sync_->registerCallback(std::bind(&MotovisInterface::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,std::placeholders::_6,std::placeholders::_7));

  //Init Camera
  if(!node_param_.use_simulated_parking)
  {
#ifndef PARKLOT_SIM
    slotDetect_Init();
    display_init(node_param_.num_of_cams);
    camera_init(node_param_.num_of_cams);
    SetWorkMode(0);
    //DisableFreespace();//it costs too much and result is wrong,so disable it before I can fix it.
#endif
  }

  // Timer
  {
    const auto period_cnn_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_cnn_ = rclcpp::create_timer(
        this, get_clock(), period_cnn_ns, std::bind(&MotovisInterface::onCNNTimer, this));

    if(!node_param_.use_simulated_parking)
    {
      //Publish can info to motovis cnn with 20ms freq
      can_pub_thread_ = std::make_shared<std::thread>(std::bind(&MotovisInterface::can_publish, this));

    }
  }

  //Bev Publisher
  {
    //BEV overlays background & vehicle png processing
    {
      cv::Mat tempImage;
      bev_background_ = cv::imread(node_param_.background_png_file);
      cv::cvtColor(bev_background_, tempImage, cv::COLOR_BGR2BGRA);
      bev_background_ = tempImage;
      bev_vehicle_ = cv::imread(node_param_.vehicle_png_file);
      cv::cvtColor(bev_vehicle_, tempImage, cv::COLOR_BGR2BGRA);
      bev_vehicle_ = tempImage;
    }
    if(node_param_.publish_bev)
    {
      bev_pub_thread_ = std::make_shared<std::thread>(std::bind(&MotovisInterface::publish_bev, this));
    }
  }
}

MotovisInterface::~MotovisInterface()
{
#ifndef PARKLOT_SIM
  if(!node_param_.use_simulated_parking)
  {
    camera_deinit(node_param_.num_of_cams);
    slotDetect_UnInit();
  }
#endif
}

void MotovisInterface::publish_bev()
{
  while(true)
  {
    if(!bev_8uc4_.empty())
    {
      std::shared_ptr<cv_bridge::CvImage> cv_ptr = std::make_shared<cv_bridge::CvImage>();
      // Set the header, encoding, and image data
      cv_ptr->header.stamp = rclcpp::Clock().now();
      cv_ptr->header.frame_id = "bev";
      cv_ptr->encoding = sensor_msgs::image_encodings::RGBA8;
      cv_ptr->image = bev_8uc4_;

      sensor_msgs::msg::Image::SharedPtr ros_image = cv_ptr->toImageMsg();
      bev_publisher_->publish(*ros_image);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void MotovisInterface::callbackPacmodRpt(
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
    const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
    const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt)
{
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  gear_cmd_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;
}

void MotovisInterface::onGoalPoseRequest(
    const UpdateGoalPose::Request::SharedPtr request,
    const UpdateGoalPose::Response::SharedPtr response)
{
  if(request)
  {
    //Send back the updated goal pose to freespace_planner_node
    RCLCPP_INFO(this->get_logger(), "Request from planner for updated goal pose");
    response->goal_pose = end_goal_pose_;

    //Publish updated goal pose to mission planner
    //to update the goal pose in arrival_checker module
    updated_goal_pose_pub_->publish(end_goal_pose_);
  }
  return;
}

void MotovisInterface::on_odometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;
}

void MotovisInterface::onShowParkingLots(const Bool::ConstSharedPtr msg)
{
  //Reset selected parking lots
  if(msg->data == true)
  {
    parkinglot_fixed_ = false;
    selected_parking_lot_pose_.x = FLT_MAX;
    selected_parking_lot_pose_.y = FLT_MAX;
    goal_angle_offset_ = FLT_MAX;
    selected_parkinglot_marker_ = Marker();
    end_goal_pose_.pose.position.x = FLT_MAX;
    end_goal_pose_.pose.position.y = FLT_MAX;
    wp1_pose_.pose.position.x = FLT_MAX;
    wp1_pose_.pose.position.y = FLT_MAX;
    wp2_pose_.pose.position.x = FLT_MAX;
    wp2_pose_.pose.position.y = FLT_MAX;

    MarkerArray marker_array_msg;
    Marker marker;
    marker.header.stamp = rclcpp::Clock().now();
    marker.header.frame_id = "map";
    marker.type = 4;
    marker.action = 3;
    marker.id = 0;
    marker_array_msg.markers.push_back(marker);
    debug_parkinglot_marker_pub_->publish(marker_array_msg);
    getTransform();
  } 
}

void MotovisInterface::onParkingLotPose(const PoseStamped::ConstSharedPtr msg)
{
  parkinglot_fixed_ = false;
  selected_parking_lot_pose_.x = FLT_MAX;
  selected_parking_lot_pose_.y = FLT_MAX;
  goal_angle_offset_ = FLT_MAX;
  selected_parkinglot_marker_ = Marker();
  end_goal_pose_.pose.position.x = FLT_MAX;
  end_goal_pose_.pose.position.y = FLT_MAX;
  wp1_pose_.pose.position.x = FLT_MAX;
  wp1_pose_.pose.position.y = FLT_MAX;
  wp2_pose_.pose.position.x = FLT_MAX;
  wp2_pose_.pose.position.y = FLT_MAX;

  selected_parking_lot_pose_.x = msg->pose.position.x;
  selected_parking_lot_pose_.y = msg->pose.position.y;

  RCLCPP_INFO(get_logger(),"!!!!!!Pose from rviz!!!!!");
  //Store the initial selected parking lot marker for
  //reference
  storeInitialSelectedParkingLot();
}

bool MotovisInterface::isParkingLotSelected(int id)
{
  Point parking_lot_point;

  parking_lot_point.x = selected_parking_lot_pose_.x;
  parking_lot_point.y = selected_parking_lot_pose_.y;

  if(node_param_.use_simulated_parking)
  { 
    Point point1,point2,point3,point4;
    //point 1 
    point1.x = (parking_lots.at(id)).p.at(0).x;
    point1.y = (parking_lots.at(id)).p.at(0).y;
    point1.z = 0;
    //point 2 
    point2.x = (parking_lots.at(id)).p.at(1).x;
    point2.y = (parking_lots.at(id)).p.at(1).y;
    point2.z = 0;
    //point 3 
    point3.x = (parking_lots.at(id)).p.at(2).x;
    point3.y = (parking_lots.at(id)).p.at(2).y;
    point3.z = 0;
    //point 4
    point4.x = (parking_lots.at(id)).p.at(3).x;
    point4.y = (parking_lots.at(id)).p.at(3).y;
    point4.z = 0;
    transformPoint(&point1);
    transformPoint(&point2);
    transformPoint(&point3);
    transformPoint(&point4);

    /*RCLCPP_INFO(get_logger(),"%d point1 x %f y %f",i,point1.x,point1.y);
      RCLCPP_INFO(get_logger(),"%d point2 x %f y %f",i,point2.x,point2.y);
      RCLCPP_INFO(get_logger(),"%d point3 x %f y %f",i,point3.x,point3.y);
      RCLCPP_INFO(get_logger(),"%d point4 x %f y %f",i,point4.x,point4.y);*/

    Point polygon[] = {point1, point2, point3, point4};

    bool isInside = false;
    // Iterate over each edge of the polygon
    for (int k = 0, j = 4 - 1; k < 4; j = k++) {
      // Check if the point is on the left side of the edge
      if (((polygon[k].y > parking_lot_point.y) != (polygon[j].y > parking_lot_point.y)) &&
          (parking_lot_point.x < (polygon[j].x - polygon[k].x) * (parking_lot_point.y - polygon[k].y) / (polygon[j].y - polygon[k].y) + polygon[k].x))
      {
        isInside = !isInside;
      }
    }

    if(isInside == true)
    {
      //updateGoalPose(point1,point2,point3,point4);
      return true;
    }
  }
  else
  {
    Point point1,point2,point3,point4;
    //point 0 
    point1.x = tSlot_.tDetectSlot[id].tPoint0.tWorldPoint.x;
    point1.y = tSlot_.tDetectSlot[id].tPoint0.tWorldPoint.y;
    point1.z = 0;
    //point 1 
    point2.x = tSlot_.tDetectSlot[id].tPoint3.tWorldPoint.x;
    point2.y = tSlot_.tDetectSlot[id].tPoint3.tWorldPoint.y;
    point2.z = 0;
    //point 2 
    point3.x = tSlot_.tDetectSlot[id].tPoint2.tWorldPoint.x;
    point3.y = tSlot_.tDetectSlot[id].tPoint2.tWorldPoint.y;
    point3.z = 0;
    //point 3
    point4.x = tSlot_.tDetectSlot[id].tPoint1.tWorldPoint.x;
    point4.y = tSlot_.tDetectSlot[id].tPoint1.tWorldPoint.y;
    point4.z = 0;
    transformPoint(&point1);
    transformPoint(&point2);
    transformPoint(&point3);
    transformPoint(&point4);

    /*RCLCPP_INFO(get_logger(),"%d point1 x %f y %f",i,point1.x,point1.y);
      RCLCPP_INFO(get_logger(),"%d point2 x %f y %f",i,point2.x,point2.y);
      RCLCPP_INFO(get_logger(),"%d point3 x %f y %f",i,point3.x,point3.y);
      RCLCPP_INFO(get_logger(),"%d point4 x %f y %f",i,point4.x,point4.y);*/

    Point polygon[] = {point1, point2, point3, point4};

    bool isInside = false;
    // Iterate over each edge of the polygon
    for (int k = 0, j = 4 - 1; k < 4; j = k++) {
      // Check if the point is on the left side of the edge
      if (((polygon[k].y > parking_lot_point.y) != (polygon[j].y > parking_lot_point.y)) &&
          (parking_lot_point.x < (polygon[j].x - polygon[k].x) * (parking_lot_point.y - polygon[k].y) / (polygon[j].y - polygon[k].y) + polygon[k].x))
      {
        isInside = !isInside;
      }
    }

    if(isInside == true)
    {
      //updateGoalPose(point1,point2,point3,point4);
      return true;
    }
  }
  return false;
}

void MotovisInterface::updateGoalPose(Point point1,Point point2,Point point3,Point point4)
{  
  if(!scenario_)
  {
    return;
  }

  PoseStamped pose;
  Pose pos;
  PoseArray pose_array;
  Point goal_pose;
  tf2::Quaternion q;
  int parking_direction = 0;

  if( scenario_->current_scenario == "FWD")
  {
    parking_direction = 1;
  }

  if(parking_direction == 0)//Reverse parking
  {
    //               p4           p3
    //                _____________
    //               |    (10deg)  |
    //               w2    w1   Goal|
    //               *    *   *    |
    // Interpoint    |_____________|            
    //         *     p1           p2

    //End goal pose
    goal_pose = findGoalPose(point1,point2,point3,point4,true,vehicle_shape_.base2back + 0.2);
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset_);
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
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

    //End goal pose
    goal_pose = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.base2back + 0.3);
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset_);
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
  }

  //Update the selected parking lot pose to end goal pose
  //Since this is use in isParkingLotSelected function everytime
  //to decide which slot is selected
  selected_parking_lot_pose_.x = goal_pose.x;
  selected_parking_lot_pose_.y = goal_pose.y;
}

void MotovisInterface::onScenario(const Scenario::ConstSharedPtr msg)
{ 
  scenario_ = msg;
  {
    double slope_angle = 0.0;
    Point point1,point2,point3,point4,goal_pose;
    int parking_direction = 0;
    int parkinglot_side = 0;
    bool isP1P2 = false;
    bool isParallelParking = false;

    if( msg->current_scenario == "FWD")
    {
      parking_direction = 1;
    }

    int i = getSelectedParkingLotID();
    if(i == 255)
    {
      RCLCPP_ERROR(get_logger(),"No parkinglot selected!!!");
      return;
    }

    if(node_param_.use_simulated_parking)
    {
      //point 1 
      point1.x = (parking_lots.at(i)).p.at(0).x;
      point1.y = (parking_lots.at(i)).p.at(0).y;
      point1.z = 0;
      //point 2 
      point2.x = (parking_lots.at(i)).p.at(1).x;
      point2.y = (parking_lots.at(i)).p.at(1).y;
      point2.z = 0;
      //point 3 
      point3.x = (parking_lots.at(i)).p.at(2).x;
      point3.y = (parking_lots.at(i)).p.at(2).y;
      point3.z = 0;
      //point 4
      point4.x = (parking_lots.at(i)).p.at(3).x;
      point4.y = (parking_lots.at(i)).p.at(3).y;
      point4.z = 0;
      transformPoint(&point1);
      transformPoint(&point2);
      transformPoint(&point3);
      transformPoint(&point4);
      parkinglot_side = (parking_lots.at(i)).d;
    }
    else
    {
      //Note the modification on point assignment to fit motovis output	
      //point 0 
      point1.x = tSlot_.tDetectSlot[i].tPoint0.tWorldPoint.x;
      point1.y = tSlot_.tDetectSlot[i].tPoint0.tWorldPoint.y;
      point1.z = 0;
      //point 1 
      point2.x = tSlot_.tDetectSlot[i].tPoint3.tWorldPoint.x;
      point2.y = tSlot_.tDetectSlot[i].tPoint3.tWorldPoint.y;
      point2.z = 0;
      //point 2 
      point3.x = tSlot_.tDetectSlot[i].tPoint2.tWorldPoint.x;
      point3.y = tSlot_.tDetectSlot[i].tPoint2.tWorldPoint.y;
      point3.z = 0;
      //point 3
      point4.x = tSlot_.tDetectSlot[i].tPoint1.tWorldPoint.x;
      point4.y = tSlot_.tDetectSlot[i].tPoint1.tWorldPoint.y;
      point4.z = 0;

      transformPoint(&point1);
      transformPoint(&point2);
      transformPoint(&point3);
      transformPoint(&point4);

      parkinglot_side = tSlot_.tDetectSlot[i].nDirection;
    }
    
    if(isP1P2)
    {
      //End goal pose angle wrt to p1 & p2
      slope_angle = getSlopeWithRefPlane(point1,point2);
      RCLCPP_INFO(get_logger(),"The slope of p1-p2 wrt (x-axis) %f",slope_angle * 180/3.141);
    }
    else
    {
      //End goal pose angle wrt to p1 & p4
      slope_angle = getSlopeWithRefPlane(point1,point4);
      RCLCPP_INFO(get_logger(),"The slope of p1-p4 wrt (x-axis) %0.20f",slope_angle * 180/3.141);
    }

    //Parallel parking if front edge is larger than the side edges 
    isParallelParking = detectParallelParking(point1,point2,point3,point4);
    if(isParallelParking)
    {
      goal_angle_offset_ = getGoalAngleOffsetParallelPark(parking_direction,parkinglot_side,slope_angle,isP1P2);
      RCLCPP_INFO(get_logger(),"ParallelPark goal offset %f",goal_angle_offset_ * 180/3.141);
      goal_pose = planParallelPark(point1,point2,point3,point4,parking_direction,parkinglot_side,goal_angle_offset_,isP1P2);
    }
    else
    {
      //Perpendicular parking
      goal_angle_offset_ = getGoalAngleOffset(parking_direction,parkinglot_side,slope_angle,isP1P2);
      RCLCPP_INFO(get_logger(),"PerpendicularPark goal offset %lf",goal_angle_offset_ * 180/3.141);
      checkGoalAngleOffset(parkinglot_side,point1,point4,&goal_angle_offset_);

      switch(parkinglot_side)
      {
        case 0://left
          goal_pose = planLeftPark(point1,point2,point3,point4,parking_direction,goal_angle_offset_,isP1P2);
          break;
        case 1://Right
          goal_pose = planRightPark(point1,point2,point3,point4,parking_direction,goal_angle_offset_,isP1P2);
          break;
        case 2://front
          goal_pose = planFrontPark(point1,point2,point3,point4,parking_direction,goal_angle_offset_,isP1P2);
          break;
        case 3://back
          goal_pose = planBackPark(point1,point2,point3,point4,parking_direction,goal_angle_offset_,isP1P2);
          break;
        default:
          RCLCPP_ERROR(get_logger(),"Invalid parking lot side!!!");
          break;
      }
    } 
    //Update the selected parking lot pose to end goal pose
    //Since this is use in isParkingLotSelected function everytime
    //to decide which slot is selected
    selected_parking_lot_pose_.x = goal_pose.x;
    selected_parking_lot_pose_.y = goal_pose.y;
  }
}

Point MotovisInterface::planRightPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2)
{
  PoseStamped inter_pose;
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
    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,true,vehicle_shape_.base2back + 0.2);
      waypoint1 = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.wheel_base);
      waypoint2 = findGoalPose(point1,point2,point3,point4,false,-1);
    }
    else
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
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);

    //Waypoint pose 1
    wp1_pose_.header.stamp = rclcpp::Clock().now();
    wp1_pose_.header.frame_id = "map";
    wp1_pose_.pose.position.x = waypoint1.x;
    wp1_pose_.pose.position.y = waypoint1.y;
    //Tilt the first waypoint 10 degree & move the position 0.15m down or up
    //Based on the direction of the parking lot
    /*wp1_pose_.pose.position.x = wp1_pose_.pose.position.x +
      (0.15 * cos(goal_angle_offset + 1.57));
    wp1_pose_.pose.position.y = wp1_pose_.pose.position.y +
      (0.15 * sin(goal_angle_offset + 1.57));
    q.setRPY(0, 0, goal_angle_offset - 0.20);*/
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp1_pose_.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose_.header.stamp = rclcpp::Clock().now();
    wp2_pose_.header.frame_id = "map";
    wp2_pose_.pose.position.x = waypoint2.x;
    wp2_pose_.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp2_pose_.pose.orientation = tf2::toMsg(q);

    //Intesection point
    Point ego_pose1,ego_pose2;
    ego_pose1.x = odometry_->pose.pose.position.x;
    ego_pose1.y = odometry_->pose.pose.position.y;

    ego_pose2.x = odometry_->pose.pose.position.x +
      (5 * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
    ego_pose2.y = odometry_->pose.pose.position.y +
      (5 * sin(tf2::getYaw(odometry_->pose.pose.orientation)));
    Point intersection_point = findIntersection(waypoint1,waypoint2,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base ) * cos(tf2::getYaw(odometry_->pose.pose.orientation) + 3.141));
    intersection_point.y = intersection_point.y +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base ) * sin(tf2::getYaw(odometry_->pose.pose.orientation) + 3.141));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = odometry_->pose.pose.orientation;

    //Publish way points
    checkpoint_pub_->publish(inter_pose);
    checkpoint_pub_->publish(wp1_pose_);
    //checkpoint_pub_->publish(wp2_pose_); 

    //Debug waypoint array
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;  //intersection pose
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose_.pose.position;    //wp1
    pos.orientation = wp1_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose_.pose.position;     //wp2
    pos.orientation = wp2_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    checkpoint_array_pub_->publish(pose_array);
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

    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.base2back + 0.3);
      waypoint1 = findGoalPose(point1,point2,point3,point4,false, -1 * vehicle_shape_.wheel_base);
    }
    else
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
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);

    //Intesection point
    Point ego_pose1,ego_pose2,mid_point;
    ego_pose1.x = odometry_->pose.pose.position.x;
    ego_pose1.y = odometry_->pose.pose.position.y;

    ego_pose2.x = odometry_->pose.pose.position.x +
      (5 * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
    ego_pose2.y = odometry_->pose.pose.position.y +
      (5 * sin(tf2::getYaw(odometry_->pose.pose.orientation)));
    mid_point.x = (point1.x + point4.x)/2;
    mid_point.y = (point1.y + point4.y)/2;
    Point intersection_point = findIntersection(mid_point,goal_pose,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      (3 * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
    intersection_point.y = intersection_point.y +
      (3 * sin(tf2::getYaw(odometry_->pose.pose.orientation)));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = odometry_->pose.pose.orientation;
    inter_pose.pose.orientation.z = inter_pose.pose.orientation.z;

    //Waypoint pose 1
    wp1_pose_.header.stamp = rclcpp::Clock().now();
    wp1_pose_.header.frame_id = "map";
    wp1_pose_.pose.position.x = waypoint1.x;
    wp1_pose_.pose.position.y = waypoint1.y;
    q.setRPY(0, 0, goal_angle_offset + 0.61); //35deg
    q.normalize();
    wp1_pose_.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose_.header.stamp = rclcpp::Clock().now();
    wp2_pose_.header.frame_id = "map";
    wp2_pose_.pose.position.x = waypoint2.x;
    wp2_pose_.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    wp2_pose_.pose.orientation = tf2::toMsg(q);

    //Publish way points
    checkpoint_pub_->publish(inter_pose);
    checkpoint_pub_->publish(wp1_pose_);
    //checkpoint_pub_->publish(wp2_pose_);

    //Debug
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose_.pose.position;
    pos.orientation = wp1_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose_.pose.position;
    pos.orientation = wp2_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    checkpoint_array_pub_->publish(pose_array);
  }
  return goal_pose;
}

bool MotovisInterface::detectParallelParking(Point point1,Point point2,Point point3,Point point4)
{
  (void)point3;
  double dx,dy,distance_1_2,distance_1_4;

  // p2        p1
  //  _________
  // |         |
  // |         |
  // |         |     ^
  // |         |     |EgoVehicle
  // |         |
  // |         |
  // |_________|            
  // p3        p4

  //Check if p1-p4 > p1-p2 distance
  dx = point2.x - point1.x;
  dy = point2.y - point1.y;
  distance_1_2 = sqrt(dx*dx + dy*dy);

  dx = point4.x - point1.x;
  dy = point4.y - point1.y;
  distance_1_4 = sqrt(dx*dx + dy*dy);
  return (distance_1_4 > distance_1_2);
}

Point MotovisInterface::planParallelPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,int parkinglot_side,double goal_angle_offset,bool isP1P2)
{
  (void)parking_direction;
  (void)point1;
  (void)point2;
  (void)isP1P2;
  PoseStamped wp1_pose;
  Pose pos;
  PoseArray pose_array;
  tf2::Quaternion q;
  Point goal_pose;
  
  //                   *wp1
  //
  // p2        p1
  //  _________
  // |         |
  // |         |
  // |         |  
  // |         |       *EgoVehicle
  // |    *Goal|     
  // |    |    |
  // |____*____|            
  // p3        p4

  //wp1
  //As parallel planner expects the ego to move forward to specfic dis
  //forward (7m) as an optimal dis so that it can work on 2 arc approach to
  //park with angled 10deg
  wp1_pose.header.stamp = rclcpp::Clock().now();
  wp1_pose.header.frame_id = "map";
  wp1_pose.pose.position.x = odometry_->pose.pose.position.x +
    ( 7 * cos(goal_angle_offset));
  wp1_pose.pose.position.y = odometry_->pose.pose.position.y +
    ( 7 * sin(goal_angle_offset));
  if(parkinglot_side)//Right side
  {
    q.setRPY(0, 0, goal_angle_offset + 0.175); //+10deg
  }
  else
  {
    q.setRPY(0, 0, goal_angle_offset - 0.175); //-10deg
  }
  q.normalize();
  wp1_pose.pose.orientation = tf2::toMsg(q);

  //End goal pose
  goal_pose.x = (point4.x + point3.x)/2;
  goal_pose.y = (point4.y + point3.y)/2;
  goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + 0.2)* cos(goal_angle_offset));
  goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + 0.2)* sin(goal_angle_offset));
  end_goal_pose_.header.stamp = rclcpp::Clock().now();
  end_goal_pose_.header.stamp = rclcpp::Clock().now();
  end_goal_pose_.header.frame_id = "map";
  end_goal_pose_.pose.position.x = goal_pose.x;
  end_goal_pose_.pose.position.y = goal_pose.y;
  q.setRPY(0, 0, goal_angle_offset);
  q.normalize();
  end_goal_pose_.pose.orientation = tf2::toMsg(q);
  goal_pose_pub_->publish(end_goal_pose_);
  updated_goal_pose_pub_->publish(end_goal_pose_);
  
  //Publish way points
  checkpoint_pub_->publish(wp1_pose);

  //Debug
  pose_array.header.stamp = rclcpp::Clock().now();
  pose_array.header.frame_id = "map";
  pos.position = wp1_pose.pose.position;
  pos.orientation = wp1_pose.pose.orientation;
  pose_array.poses.push_back(pos);
  checkpoint_array_pub_->publish(pose_array);

  return goal_pose;
}

Point MotovisInterface::planLeftPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2)
{
  PoseStamped inter_pose;
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
    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,true,vehicle_shape_.base2back + 0.2);
      waypoint1 = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.wheel_base);
      waypoint2 = findGoalPose(point1,point2,point3,point4,false,-1);
    }
    else
    {
      //GoalPoint
      goal_pose.x = mid_point.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.4)* cos(goal_angle_offset));
      goal_pose.y = mid_point.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.4)* sin(goal_angle_offset));

      //w1 point
      waypoint1.x = mid_point.x + ( 2 * cos(goal_angle_offset));
      waypoint1.y = mid_point.y + ( 2 * sin(goal_angle_offset)); 

      //w2 point
      waypoint2.x = mid_point.x + ( -1 * cos(goal_angle_offset));
      waypoint2.y = mid_point.y + ( -1 * sin(goal_angle_offset));
    }

    //End goal pose
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);

    //Waypoint pose 1
    wp1_pose_.header.stamp = rclcpp::Clock().now();
    wp1_pose_.header.frame_id = "map";
    wp1_pose_.pose.position.x = waypoint1.x;
    wp1_pose_.pose.position.y = waypoint1.y;
    //Tilt the first waypoint 10 degree & move the position 0.15m down or up
    //Based on the direction of the parking lot
    /*q.setRPY(0, 0, goal_angle_offset + 0.20);
    wp1_pose_.pose.position.x = wp1_pose_.pose.position.x +
      (0.15 * cos(goal_angle_offset - 1.57));
    wp1_pose_.pose.position.y = wp1_pose_.pose.position.y +
      (0.15 * sin(goal_angle_offset - 1.57));*/
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp1_pose_.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose_.header.stamp = rclcpp::Clock().now();
    wp2_pose_.header.frame_id = "map";
    wp2_pose_.pose.position.x = waypoint2.x;
    wp2_pose_.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset + 3.141);
    q.normalize();
    wp2_pose_.pose.orientation = tf2::toMsg(q);

    //Intesection point
    Point ego_pose1,ego_pose2;
    ego_pose1.x = odometry_->pose.pose.position.x;
    ego_pose1.y = odometry_->pose.pose.position.y;

    ego_pose2.x = odometry_->pose.pose.position.x +
      (5 * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
    ego_pose2.y = odometry_->pose.pose.position.y +
      (5 * sin(tf2::getYaw(odometry_->pose.pose.orientation)));
    Point intersection_point = findIntersection(waypoint1,waypoint2,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * cos(tf2::getYaw(odometry_->pose.pose.orientation) + 3.141));
    intersection_point.y = intersection_point.y +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * sin(tf2::getYaw(odometry_->pose.pose.orientation) + 3.141));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = odometry_->pose.pose.orientation;

    //Publish way points
    checkpoint_pub_->publish(inter_pose);
    checkpoint_pub_->publish(wp1_pose_);
    //checkpoint_pub_->publish(wp2_pose_); 

    //Debug waypoint array
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;  //intersection pose
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose_.pose.position;    //wp1
    pos.orientation = wp1_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose_.pose.position;     //wp2
    pos.orientation = wp2_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    checkpoint_array_pub_->publish(pose_array);
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

    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.base2back + 0.3);
      waypoint1 = findGoalPose(point1,point2,point3,point4,false, -1 * vehicle_shape_.wheel_base);
    }
    else
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
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);

    //Intesection point
    Point ego_pose1,ego_pose2,mid_point;
    ego_pose1.x = odometry_->pose.pose.position.x;
    ego_pose1.y = odometry_->pose.pose.position.y;

    ego_pose2.x = odometry_->pose.pose.position.x +
      (5 * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
    ego_pose2.y = odometry_->pose.pose.position.y +
      (5 * sin(tf2::getYaw(odometry_->pose.pose.orientation)));
    mid_point.x = (point1.x + point4.x)/2;
    mid_point.y = (point1.y + point4.y)/2;
    Point intersection_point = findIntersection(mid_point,goal_pose,ego_pose1,ego_pose2);
    RCLCPP_INFO(get_logger(),"intersection Point x:%f y:%f",intersection_point.x,intersection_point.y);
    intersection_point.x = intersection_point.x +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
    intersection_point.y = intersection_point.y +
      ((vehicle_shape_.base2back + vehicle_shape_.wheel_base) * sin(tf2::getYaw(odometry_->pose.pose.orientation)));
    inter_pose.header.stamp = rclcpp::Clock().now();
    inter_pose.header.frame_id = "map";
    inter_pose.pose.position.x = intersection_point.x;
    inter_pose.pose.position.y = intersection_point.y;
    inter_pose.pose.orientation = odometry_->pose.pose.orientation;

    //Waypoint pose 1
    wp1_pose_.header.stamp = rclcpp::Clock().now();
    wp1_pose_.header.frame_id = "map";
    wp1_pose_.pose.position.x = waypoint1.x;
    wp1_pose_.pose.position.y = waypoint1.y;
    q.setRPY(0, 0, goal_angle_offset - 0.61); //35deg
    q.normalize();
    wp1_pose_.pose.orientation = tf2::toMsg(q);

    //Waypoint pose 2
    wp2_pose_.header.stamp = rclcpp::Clock().now();
    wp2_pose_.header.frame_id = "map";
    wp2_pose_.pose.position.x = waypoint2.x;
    wp2_pose_.pose.position.y = waypoint2.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    wp2_pose_.pose.orientation = tf2::toMsg(q);

    //Publish way points
    checkpoint_pub_->publish(inter_pose);
    checkpoint_pub_->publish(wp1_pose_);
    //checkpoint_pub_->publish(wp2_pose_);

    //Debug
    pose_array.header.stamp = rclcpp::Clock().now();
    pose_array.header.frame_id = "map";
    pos.position = inter_pose.pose.position;
    pos.orientation = inter_pose.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp1_pose_.pose.position;
    pos.orientation = wp1_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    pos.position = wp2_pose_.pose.position;
    pos.orientation = wp2_pose_.pose.orientation;
    pose_array.poses.push_back(pos);
    checkpoint_array_pub_->publish(pose_array);
  }
  return goal_pose;
}

Point MotovisInterface::planFrontPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2)
{
  PoseStamped inter_pose;
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
    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,true,vehicle_shape_.base2back + 0.2);
    }
    else
    {
      //GoalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* cos(goal_angle_offset + 3.141));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* sin(goal_angle_offset + 3.141));
    }

    //End goal pose
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);
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

    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.base2back + 0.3);
    }
    else
    {
      //goalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + 0.3) * cos(goal_angle_offset ));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + 0.3)* sin(goal_angle_offset ));
    }

    //End goal pose
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);
  }
  return goal_pose;
}

Point MotovisInterface::planBackPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2)
{
  PoseStamped inter_pose;
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
    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,true,vehicle_shape_.base2back + 0.2);
    }
    else
    {
      //GoalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* cos(goal_angle_offset + 3.141));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + vehicle_shape_.wheel_base + 0.1)* sin(goal_angle_offset + 3.141));
    }

    //End goal pose
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset);
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);
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

    if(isP1P2)
    {
      goal_pose = findGoalPose(point1,point2,point3,point4,false,vehicle_shape_.base2back + 0.3);
    }
    else
    {
      //goalPoint
      goal_pose.x = (point1.x + point4.x)/2;
      goal_pose.y = (point1.y + point4.y)/2;
      goal_pose.x = goal_pose.x + ((vehicle_shape_.base2back + 0.3) * cos(goal_angle_offset ));
      goal_pose.y = goal_pose.y + ((vehicle_shape_.base2back + 0.3)* sin(goal_angle_offset ));
    }

    //End goal pose
    end_goal_pose_.header.stamp = rclcpp::Clock().now();
    end_goal_pose_.header.frame_id = "map";
    end_goal_pose_.pose.position.x = goal_pose.x;
    end_goal_pose_.pose.position.y = goal_pose.y;
    q.setRPY(0, 0, goal_angle_offset );
    q.normalize();
    end_goal_pose_.pose.orientation = tf2::toMsg(q);
    goal_pose_pub_->publish(end_goal_pose_);
    updated_goal_pose_pub_->publish(end_goal_pose_);
  }
  return goal_pose;
}

void MotovisInterface::can_publish()
{
  while(true)
  {
    auto next_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    PushCanInfo_to_motovis();
    std::this_thread::sleep_until(next_time);
  }
}

void MotovisInterface::onCNNTimer()
{
  if(!node_param_.use_simulated_parking)
  {
#ifndef PARKLOT_SIM
    //struct timeval tendcnn, tstartcnn;
    //unsigned long long elapsed;

    //Cam capture
    //gettimeofday(&tstartcnn, NULL);
    check_mipi_overflow();
    capture_frames(node_param_.num_of_cams);
    //gettimeofday(&tendcnn, NULL);
    //elapsed =((tendcnn.tv_sec - tstartcnn.tv_sec) * 1000000) + (tendcnn.tv_usec - tstartcnn.tv_usec);
    //RCLCPP_INFO(get_logger(),"Capture Frames Delay is %llu us",elapsed);

    //CNN
    //gettimeofday(&tstartcnn, NULL);
    MvStartCnn();
    while (GetCnnFinishFlag() == false)
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "!!!CNN Wait!!!");
      usleep(10000);//wait for cnn result
      check_mipi_overflow();
    }
#endif
    //gettimeofday(&tendcnn, NULL);
    //elapsed =((tendcnn.tv_sec - tstartcnn.tv_sec) * 1000000) + (tendcnn.tv_usec - tstartcnn.tv_usec);
    //RCLCPP_INFO(get_logger(),"CNN Delay is %llu us",elapsed);
  }

  //BEV image
  updateBev();
  publish_freespace();
  publish_parkinglot();
}

void MotovisInterface::updateBev()
{
  if(node_param_.use_simulated_parking)
  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    cv::Mat bev_mat;
    bev_background_.copyTo(bev_mat);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
    auto str = oss.str();
    cv::putText(bev_mat,str,cv::Point(50,50),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    // Setup a rectangle to define your region of interest
    cv::Rect myROI(12, 12, 640-12, 640-12);
    bev_8uc4_ = bev_mat(myROI);

    if(node_param_.use_vehicle_overlay_on_bev && !bev_vehicle_.empty())
    {
      // Define the ROI on the background image where the vehicle overlay will be placed
      cv::Mat roi = bev_8uc4_(cv::Rect(280 - 12, 194 - 12, bev_vehicle_.cols, bev_vehicle_.rows));
      bev_vehicle_.copyTo(roi);
    }

    if(node_param_.share_bev_to_disti_hmi)
    {
      SharedMemoryStructure *mem = shm_interface_.GetSharedMemory();
      memcpy(mem->textureBuffer,bev_8uc4_.data,bev_8uc4_.total() * bev_8uc4_.elemSize());
    }
  }
  else
  {
#ifndef PARKLOT_SIM
    //Display Bev
    ulBv2dAddr_ = GetBv2dImageAddr();
    RCLCPP_DEBUG(get_logger(),"Get Bv2d image addr is %llu\n",ulBv2dAddr_);
    if (ulBv2dAddr_)
    {
      bev_8uc4_ = cv::Mat(cv::Size(640,640), CV_8UC4, (uint8_t *)ulBv2dAddr_);

      if(node_param_.use_vehicle_overlay_on_bev && !bev_vehicle_.empty())
      {
        // Define the ROI on the background image where the vehicle overlay will be placed
        cv::Mat roi = bev_8uc4_(cv::Rect(280, 194, bev_vehicle_.cols, bev_vehicle_.rows));
        bev_vehicle_.copyTo(roi);
      }

      if(node_param_.share_bev_to_disti_hmi)
      {
        SharedMemoryStructure *mem = shm_interface_.GetSharedMemory();
        memcpy(mem->textureBuffer,bev_8uc4_.data,bev_8uc4_.total() * bev_8uc4_.elemSize());
      }
    }
#endif
  }
}

void MotovisInterface::PushCanInfo_to_motovis()
{
  CarInfo vehicleInfo;
  memset(&vehicleInfo,0,sizeof(CarInfo));

  if(gear_cmd_rpt_ptr_)
  {
    switch(gear_cmd_rpt_ptr_->output)
    {
      case 0://P
        vehicleInfo.nPulseDirection = (int)StandStill;
        vehicleInfo.nGear = (int)Parking;
        break;
      case 1://R
        vehicleInfo.nPulseDirection = (int)Backward;
        vehicleInfo.nGear = (int)Reverse;
        break;
      case 2://N
        vehicleInfo.nPulseDirection = (int)StandStill;
        vehicleInfo.nGear = (int)Neutral;
        break;
      case 3://D
        vehicleInfo.nPulseDirection = (int)Forward;
        vehicleInfo.nGear = (int)Drive;
        break;
      default:
        vehicleInfo.nPulseDirection = (int)Invalid;
        vehicleInfo.nGear = (int)None;
        break;
    }
  }

  if(steer_wheel_rpt_ptr_)
  {
    vehicleInfo.fSteeingWheelAngle = steer_wheel_rpt_ptr_->output; //rad
  }

  if(turn_rpt_ptr_)
  {
    switch(turn_rpt_ptr_->output)
    {
      case 1: //None
        vehicleInfo.nRLight = 0;
        vehicleInfo.nLLight = 0;
        break;
      case 2://Left
        vehicleInfo.nRLight = 0;
        vehicleInfo.nLLight = 1;
        break;
      case 3://right
        vehicleInfo.nRLight = 1;
        vehicleInfo.nLLight = 0;
        break;
      default:
        vehicleInfo.nRLight = 0;
        vehicleInfo.nLLight = 0;
        break;
    }
  }

  if(wheel_speed_rpt_ptr_ && gear_cmd_rpt_ptr_ && steer_wheel_rpt_ptr_)
  {
    const double tire_radius = 0.383; //[m]
    const double sign = (gear_cmd_rpt_ptr_->output == pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
    double vel = (wheel_speed_rpt_ptr_->rear_left_wheel_speed + wheel_speed_rpt_ptr_->rear_right_wheel_speed) * 0.5 *  tire_radius;


    vehicleInfo.fVelocity = sign * vel  * 3.6; //km/h

    if(vel < 1e-4)
    {
      vehicleInfo.nBrake = 1;
    }
    else
    {
      vehicleInfo.nBrake = 0;
    }
    vehicleInfo.fWheelSpeed = vehicleInfo.fVelocity; //km/h

    vel = vel * sign;
    double steering_offset = 0.0;
    double vgr_coef_a = 15.713;
    double vgr_coef_b = 0.053;
    double vgr_coef_c = 0.042;
    double wheel_base = 2.79;
    const double adaptive_gear_ratio = std::max(
        1e-5, vgr_coef_a + vgr_coef_b * vel * vel - vgr_coef_c * std::fabs( steer_wheel_rpt_ptr_->output));
    const double current_steer = steer_wheel_rpt_ptr_->output / adaptive_gear_ratio + steering_offset;

    vehicleInfo.fYawRate = vel * std::tan(current_steer) / wheel_base; //rad/s
    vehicleInfo.fAlpha = current_steer; //rad
  }

  VelocityReport vr;
  vr.header.stamp = rclcpp::Clock().now();
  vr.header.frame_id = "base_link";
  vr.longitudinal_velocity = vehicleInfo.fVelocity;
  pub_debug_velocity_report_->publish(vr);

  //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "CAN_INFO\nBrake:%d LLight:%d RLight:%d\tVelocity:%f yaw:%f Alpha:%f\tSteerWheelAngle:%f Pulsedir:%d Gear:%d WheelSpeed:%f",vehicleInfo.nBrake,vehicleInfo.nLLight,vehicleInfo.nRLight,vehicleInfo.fVelocity,vehicleInfo.fYawRate,vehicleInfo.fAlpha,vehicleInfo.fSteeingWheelAngle,vehicleInfo.nPulseDirection,vehicleInfo.nGear,vehicleInfo.fWheelSpeed);

  /*printf("nBrake: %d\n",vehicleInfo.nBrake);
  printf("nLLight: %d\n",vehicleInfo.nLLight);
  printf("nRLight: %d\n",vehicleInfo.nRLight);
  printf("fVelocity: %f\n",vehicleInfo.fVelocity);
  printf("fYawRate: %f\n",vehicleInfo.fYawRate);
  printf("fAlpha: %f\n",vehicleInfo.fAlpha);
  printf("fSteeingWheelAngle: %f\n",vehicleInfo.fSteeingWheelAngle);
  printf("nFLWheelSpeedRC: %d\n",vehicleInfo.nFLWheelSpeedRC);
  printf("nFRWheelSpeedRC: %d\n",vehicleInfo.nFRWheelSpeedRC);
  printf("nRLWheelSpeedRC: %d\n",vehicleInfo.nRLWheelSpeedRC);
  printf("nRRWheelSpeedRC: %d\n",vehicleInfo.nRRWheelSpeedRC);
  printf("nPulseDirection: %d\n",vehicleInfo.nPulseDirection);
  printf("nGear: %d\n",vehicleInfo.nGear);
  printf("fWheelSpeed: %f\n",vehicleInfo.fWheelSpeed);*/
#ifndef PARKLOT_SIM
  PushCanInfo(vehicleInfo);
#endif
}

void MotovisInterface::publish_freespace()
{
  PolygonStamped polygon_msg;
  Point32 point;

  polygon_msg.header.stamp = rclcpp::Clock().now();
  polygon_msg.header.frame_id = "base_link";
//#ifdef PARKLOT_SIM
  point.x = 20;
  point.y = 20; 
  polygon_msg.polygon.points.push_back(point);
  point.x = -20;
  point.y = 20; 
  polygon_msg.polygon.points.push_back(point);
  point.x = -20;
  point.y = -20; 
  polygon_msg.polygon.points.push_back(point);
  point.x = 20;
  point.y = -20; 
  polygon_msg.polygon.points.push_back(point);
/*#else
  exFreeSpaceRegionInfo tFreespace;
  GetFreespaceResult( tFreespace);

  for( unsigned int i=0; i < tFreespace.nPointNum ; i++)
  {
    point.x = tFreespace.tPoint[i].tWorldPoint.x;
    point.y = tFreespace.tPoint[i].tWorldPoint.y;
    if(isnan(point.x) || isnan(point.y))
    {
      continue;
    }

    if(std::abs(point.x) > 6.0)
    {
      point.x = point.x * 2;
    }
    if(std::abs(point.y) > 6.0)
    {
      point.y = point.y * 2;
    }

    polygon_msg.polygon.points.push_back(point);
  }
#endif*/
  debug_freespace_pub_->publish(polygon_msg);
}

void MotovisInterface::publish_parkinglot()
{
  MarkerArray marker_array_msg;
  ParkingLots parking_lots_msg;
  Point point1,point2,point3,point4;
  ColorRGBA color;
  static int marker_refresh_counter = 0;

  if(node_param_.use_simulated_parking)
  {
#ifdef AVP
    //For testing the AVP dummy parking lots
    //uncomment plot1,2,3 below if testing avp
    getTransform();
#endif 
    parking_lots_msg.detect_slot_num = parking_lots.size();
    for(unsigned int i =0;i < parking_lots.size();i++)
    {
      Marker marker;
      marker.header.stamp = rclcpp::Clock().now();
      marker.header.frame_id = "map";
      marker.type = 4;
      marker.action = 0;
      //ID 0 is used by selected_parkinglot_marker_
      //so start from 1
      marker.id = i + 1;

      if(isParkingLotSelected(i))
      {
        marker.scale.x = 0.15;
        color.a = 1.0;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
      }
      else
      {
        marker.scale.x = 0.05;
        color.a = 1.0;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
      }

      //point 1 
      point1.x = (parking_lots.at(i)).p.at(0).x;
      point1.y = (parking_lots.at(i)).p.at(0).y;
      point1.z = 0;
      transformPoint(&point1);
      marker.points.push_back(point1);
      marker.colors.push_back(color);

      //point 2 
      point2.x = (parking_lots.at(i)).p.at(1).x;
      point2.y = (parking_lots.at(i)).p.at(1).y;
      point2.z = 0;
      transformPoint(&point2);      
      marker.points.push_back(point2);
      marker.colors.push_back(color);

      //point 3 
      point3.x = (parking_lots.at(i)).p.at(2).x;
      point3.y = (parking_lots.at(i)).p.at(2).y;
      point3.z = 0;
      transformPoint(&point3);      
      marker.points.push_back(point3);
      marker.colors.push_back(color);

      //point 4
      point4.x = (parking_lots.at(i)).p.at(3).x;
      point4.y = (parking_lots.at(i)).p.at(3).y;
      point4.z = 0;
      transformPoint(&point4);      
      marker.points.push_back(point4);
      marker.colors.push_back(color);

      //Connect again to point 0
      marker.points.push_back(marker.points[0]);
      marker.colors.push_back(marker.colors[0]);

      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker_array_msg.markers.push_back(marker);

      ParkingLot parking_lot;
      parking_lot.point1 = point1;
      parking_lot.point2 = point2;
      parking_lot.point3 = point3;
      parking_lot.point4 = point4;
      parking_lot.slot_id = i;
      parking_lot.direction = (parking_lots.at(i)).d;
      parking_lot.available_state = 1;
      parking_lots_msg.detect_slots.push_back(parking_lot);
    }
  }
  else
  {
#ifndef PARKLOT_SIM
    //Get current position of ego everytime
    getTransform();
    GetSlotResult(tSlot_);

    parking_lots_msg.detect_slot_num = tSlot_.nDetectSlotNum;
    for(unsigned int i =0;i < tSlot_.nDetectSlotNum;i++)
    {
      Marker marker;
      Marker marker_available_state_txt;
      marker.header.stamp = rclcpp::Clock().now();
      marker.header.frame_id = "map";
      marker.type = 4;
      marker.ns = "parking_lots";
      marker.action = 0;
      //ID 0 is used by selected_parkinglot_marker_
      //so start from 1
      marker.id = i + 1;

      if(tSlot_.tDetectSlot[i].nAvailableState == 1)
      {
        marker.scale.x = 0.05;
        color.a = 1.0;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
      }
      else
      {
        marker.scale.x = 0.05;
        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
      }

      //For selected parking lot
      if(isParkingLotSelected(i))
      {
        marker.scale.x = 0.15;
        color.a = 1.0;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
      }

      //point 0 
      point1.x = tSlot_.tDetectSlot[i].tPoint0.tWorldPoint.x;
      point1.y = tSlot_.tDetectSlot[i].tPoint0.tWorldPoint.y;
      point1.z = 0;
      transformPoint(&point1);      
      marker.points.push_back(point1);
      marker.colors.push_back(color);
      //point 1 
      point2.x = tSlot_.tDetectSlot[i].tPoint3.tWorldPoint.x;
      point2.y = tSlot_.tDetectSlot[i].tPoint3.tWorldPoint.y;
      point2.z = 0;
      transformPoint(&point2);      
      marker.points.push_back(point2);
      marker.colors.push_back(color);
      //point 2 
      point3.x = tSlot_.tDetectSlot[i].tPoint2.tWorldPoint.x;
      point3.y = tSlot_.tDetectSlot[i].tPoint2.tWorldPoint.y;
      point3.z = 0;
      transformPoint(&point3);      
      marker.points.push_back(point3);
      marker.colors.push_back(color);
      //point 3
      point4.x = tSlot_.tDetectSlot[i].tPoint1.tWorldPoint.x;
      point4.y = tSlot_.tDetectSlot[i].tPoint1.tWorldPoint.y;
      point4.z = 0;
      transformPoint(&point4);      
      marker.points.push_back(point4);
      marker.colors.push_back(color);

      //Connect again to point 0
      marker.points.push_back(marker.points[0]);
      marker.colors.push_back(marker.colors[0]);

      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      //Occupied state txt marker
      marker_available_state_txt.header.stamp = rclcpp::Clock().now();
      marker_available_state_txt.header.frame_id = "map";
      marker_available_state_txt.ns = "parking_lot_state";
      marker_available_state_txt.action = 0;
      marker_available_state_txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker_available_state_txt.scale.x = 0.5;
      marker_available_state_txt.scale.z = 0.5;
      marker_available_state_txt.id = i;
      marker_available_state_txt.pose.position.x = (point1.x + point2.x + point3.x + point4.x)/4;
      marker_available_state_txt.pose.position.y = (point1.y + point2.y + point3.y + point4.y)/4;
      marker_available_state_txt.pose.position.z = (point1.z + point2.z + point3.z + point4.z)/4;
      if(tSlot_.tDetectSlot[i].nAvailableState == 1)
      {
        marker_available_state_txt.color.a = 1.0;
        marker_available_state_txt.color.r = 0.0;
        marker_available_state_txt.color.g = 1.0;
        marker_available_state_txt.color.b = 0.0;
        marker_available_state_txt.text = "Unoccupied";
      }
      else
      {
        marker_available_state_txt.color.a = 1.0;
        marker_available_state_txt.color.r = 1.0;
        marker_available_state_txt.color.g = 0.0;
        marker_available_state_txt.color.b = 0.0;
        marker_available_state_txt.text = "Occupied";
      }
      marker_available_state_txt.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_available_state_txt.frame_locked = true;

      marker_array_msg.markers.push_back(marker);
      marker_array_msg.markers.push_back(marker_available_state_txt);

      ParkingLot parking_lot;
      parking_lot.point1 = point1;
      parking_lot.point2 = point2;
      parking_lot.point3 = point3;
      parking_lot.point4 = point4;
      parking_lot.slot_id = tSlot_.tDetectSlot[i].nSlotId;
      parking_lot.direction = tSlot_.tDetectSlot[i].nDirection;
      parking_lot.type = tSlot_.tDetectSlot[i].nType;
      parking_lot.available_state = tSlot_.tDetectSlot[i].nAvailableState;
      parking_lot.angle = tSlot_.tDetectSlot[i].fAngle;
      parking_lot.slot_theta = tSlot_.tDetectSlot[i].fSlotTheta;
      parking_lots_msg.detect_slots.push_back(parking_lot);
    }
#endif
  }

  //Push the initial selected marker
  if(selected_parkinglot_marker_.points.size() > 0)
  {
    selected_parkinglot_marker_.header.stamp = rclcpp::Clock().now();
    marker_array_msg.markers.push_back(selected_parkinglot_marker_);
  }

  //publish
  parking_lots_msg.header.stamp = rclcpp::Clock().now();
  parking_lots_msg.header.frame_id = "map";
  parkinglot_pub_->publish(parking_lots_msg);
  debug_parkinglot_marker_pub_->publish(marker_array_msg);
  marker_refresh_counter++;

  //Refresh marker every 1sec
  if(marker_refresh_counter == node_param_.update_rate)
  {
    marker_refresh_counter = 0;
    MarkerArray marker_array_msg;
    Marker marker;
    marker.header.stamp = rclcpp::Clock().now();
    marker.header.frame_id = "map";
    marker.type = 4;
    marker.action = 3;
    marker.id = 0;
    marker_array_msg.markers.push_back(marker);
    debug_parkinglot_marker_pub_->publish(marker_array_msg);
  }
}

int MotovisInterface::getSelectedParkingLotID()
{
  if(node_param_.use_simulated_parking)
  {
    for(unsigned int i =0;i < parking_lots.size();i++)
    {
      if(isParkingLotSelected(i))
      {
        return i;
      }
    }
  }
  else
  {
#ifndef PARKLOT_SIM
    GetSlotResult( tSlot_);

    for(unsigned int i =0;i < tSlot_.nDetectSlotNum;i++)
    {
      if(isParkingLotSelected(i))
      {
        return i;
      }
    }
#endif
  }
  return 255;
}

void MotovisInterface::storeInitialSelectedParkingLot()
{
  Point point1,point2,point3,point4;
  ColorRGBA color;

  if(node_param_.use_simulated_parking)
  {
    for(unsigned int i =0;i < parking_lots.size();i++)
    {
      if(isParkingLotSelected(i))
      {
        selected_parkinglot_marker_.header.stamp = rclcpp::Clock().now();
        selected_parkinglot_marker_.header.frame_id = "map";
        selected_parkinglot_marker_.type = 4;
        selected_parkinglot_marker_.action = 0;
        selected_parkinglot_marker_.id = 0;

        selected_parkinglot_marker_.scale.x = 0.15;
        color.a = 1.0;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;

        //point 1 
        point1.x = (parking_lots.at(i)).p.at(0).x;
        point1.y = (parking_lots.at(i)).p.at(0).y;
        point1.z = 0;
        transformPoint(&point1);      
        selected_parkinglot_marker_.points.push_back(point1);
        selected_parkinglot_marker_.colors.push_back(color);
        //point 2 
        point2.x = (parking_lots.at(i)).p.at(1).x;
        point2.y = (parking_lots.at(i)).p.at(1).y;
        point2.z = 0;
        transformPoint(&point2);      
        selected_parkinglot_marker_.points.push_back(point2);
        selected_parkinglot_marker_.colors.push_back(color);
        //point 3 
        point3.x = (parking_lots.at(i)).p.at(2).x;
        point3.y = (parking_lots.at(i)).p.at(2).y;
        point3.z = 0;
        transformPoint(&point3);      
        selected_parkinglot_marker_.points.push_back(point3);
        selected_parkinglot_marker_.colors.push_back(color);
        //point 4
        point4.x = (parking_lots.at(i)).p.at(3).x;
        point4.y = (parking_lots.at(i)).p.at(3).y;
        point4.z = 0;
        transformPoint(&point4);      
        selected_parkinglot_marker_.points.push_back(point4);
        selected_parkinglot_marker_.colors.push_back(color);

        //Connect again to point 0
        selected_parkinglot_marker_.points.push_back(selected_parkinglot_marker_.points[0]);
        selected_parkinglot_marker_.colors.push_back(selected_parkinglot_marker_.colors[0]);

        selected_parkinglot_marker_.color.a = 1.0;
        selected_parkinglot_marker_.color.r = 0.0;
        selected_parkinglot_marker_.color.g = 1.0;
        selected_parkinglot_marker_.color.b = 0.0;

        selected_parkinglot_marker_.pose.position.x = 0.0;
        selected_parkinglot_marker_.pose.position.y = 0.0;
        selected_parkinglot_marker_.pose.position.z = 0.0;
        selected_parkinglot_marker_.pose.orientation.x = 0.0;
        selected_parkinglot_marker_.pose.orientation.y = 0.0;
        selected_parkinglot_marker_.pose.orientation.z = 0.0;
        selected_parkinglot_marker_.pose.orientation.w = 1.0;
        break;
      }
    }
  }
  else
  {
#ifndef PARKLOT_SIM
    GetSlotResult( tSlot_);

    for(unsigned int i =0;i < tSlot_.nDetectSlotNum;i++)
    {
      if(isParkingLotSelected(i))
      {
        //For slot tracking to motovis module
        //SetParkSlotId(tSlot_.tDetectSlot[i].nSlotId);

        selected_parkinglot_marker_.header.stamp = rclcpp::Clock().now();
        selected_parkinglot_marker_.header.frame_id = "map";
        selected_parkinglot_marker_.type = 4;
        selected_parkinglot_marker_.action = 0;
        selected_parkinglot_marker_.id = 0;

        selected_parkinglot_marker_.scale.x = 0.15;
        color.a = 1.0;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;

        //point 0 
        point1.x = tSlot_.tDetectSlot[i].tPoint0.tWorldPoint.x;
        point1.y = tSlot_.tDetectSlot[i].tPoint0.tWorldPoint.y;
        point1.z = 0;
        transformPoint(&point1);      
        selected_parkinglot_marker_.points.push_back(point1);
        selected_parkinglot_marker_.colors.push_back(color);
        //point 1 
        point2.x = tSlot_.tDetectSlot[i].tPoint1.tWorldPoint.x;
        point2.y = tSlot_.tDetectSlot[i].tPoint1.tWorldPoint.y;
        point2.z = 0;
        transformPoint(&point2);      
        selected_parkinglot_marker_.points.push_back(point2);
        selected_parkinglot_marker_.colors.push_back(color);
        //point 2 
        point3.x = tSlot_.tDetectSlot[i].tPoint2.tWorldPoint.x;
        point3.y = tSlot_.tDetectSlot[i].tPoint2.tWorldPoint.y;
        point3.z = 0;
        transformPoint(&point3);      
        selected_parkinglot_marker_.points.push_back(point3);
        selected_parkinglot_marker_.colors.push_back(color);
        //point 3
        point4.x = tSlot_.tDetectSlot[i].tPoint3.tWorldPoint.x;
        point4.y = tSlot_.tDetectSlot[i].tPoint3.tWorldPoint.y;
        point4.z = 0;
        transformPoint(&point4);      
        selected_parkinglot_marker_.points.push_back(point4);
        selected_parkinglot_marker_.colors.push_back(color);

        //Connect again to point 0
        selected_parkinglot_marker_.points.push_back(selected_parkinglot_marker_.points[0]);
        selected_parkinglot_marker_.colors.push_back(selected_parkinglot_marker_.colors[0]);

        selected_parkinglot_marker_.color.a = 1.0;
        selected_parkinglot_marker_.color.r = 0.0;
        selected_parkinglot_marker_.color.g = 1.0;
        selected_parkinglot_marker_.color.b = 0.0;

        selected_parkinglot_marker_.pose.position.x = 0.0;
        selected_parkinglot_marker_.pose.position.y = 0.0;
        selected_parkinglot_marker_.pose.position.z = 0.0;
        selected_parkinglot_marker_.pose.orientation.x = 0.0;
        selected_parkinglot_marker_.pose.orientation.y = 0.0;
        selected_parkinglot_marker_.pose.orientation.z = 0.0;
        selected_parkinglot_marker_.pose.orientation.w = 1.0;
        break;
      }
    }
#endif
  }
}

void MotovisInterface::getTransform()
{
  try {
    tf_ = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
}

void MotovisInterface::transformPoint(Point *point)
{
  geometry_msgs::msg::PoseStamped output_stamped, input_stamped;
  input_stamped.pose.position.x = point->x;
  input_stamped.pose.position.y = point->y;
  input_stamped.pose.position.z = point->z;
  tf2::doTransform(input_stamped, output_stamped, tf_);
  point->x = output_stamped.pose.position.x;
  point->y = output_stamped.pose.position.y;
  point->z = output_stamped.pose.position.z;
}

void MotovisInterface::transformPointToBaseFrame(Point *point)
{
  geometry_msgs::msg::TransformStamped tf;
  geometry_msgs::msg::PoseStamped output_stamped, input_stamped;
  try {
    tf = tf_buffer_.lookupTransform("base_link", "map", rclcpp::Time(0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }

  input_stamped.pose.position.x = point->x;
  input_stamped.pose.position.y = point->y;
  input_stamped.pose.position.z = point->z;
  tf2::doTransform(input_stamped, output_stamped, tf);
  point->x = output_stamped.pose.position.x;
  point->y = output_stamped.pose.position.y;
  point->z = output_stamped.pose.position.z;
}

Point MotovisInterface::findGoalPose(Point point1,Point point2,Point point3,Point point4,bool isFromEnd ,double dis_offset)
{
  (void)point3;
  Point goal_pose;
  double distance,dx,dy;
  double goal_offset = dis_offset;

  // p1            p2
  //  _____________
  // |             |
  // |             |
  // |             |
  // |_____________|            
  // p4 <--dis-->  p3
  // 
  dx = point2.x - point1.x;
  dy = point2.y - point1.y;
  distance = sqrt(dx*dx + dy*dy);
  dx /= distance;
  dy /= distance;

  // p1           p2
  //  _____________
  // |             |
  // |             |
  // * mid point   |
  // |             |
  // |_____________|            
  // p4           p3
  //  	
  goal_pose.x = (point1.x + point4.x)/2;
  goal_pose.y = (point1.y + point4.y)/2;

  // p1           p2
  //  _____________
  // |             |
  // |             |
  // |------* offset_point
  // |             |
  // |_____________|            
  // p4            p3
  // 
  if(isFromEnd == true)
  {
    goal_pose.x += (distance - goal_offset) * dx;
    goal_pose.y += (distance - goal_offset) * dy; 
  }
  else
  {
    goal_pose.x += goal_offset * dx;
    goal_pose.y += goal_offset * dy; 
  }

  /*
  //Old approach for ref
  Point goal_pose;
  double x1,y1,x2,y2,distance,dx,dy;
  double distance,dx,dy;
  double goal_offset = dis_offset;
  // p4           p3
  //  _____________
  // |             |
  // |             |
  // |             |
  // |_____________|            
  // p1        *    p2
  //           |------> point x meters from p1
  //
  //Find a point in a line connecting between p1,p2 using linear interpolation   
  dx = point2.x - point1.x;
  dy = point2.y - point1.y;

  // Find waypoint offset
  // For Forward parking 
  //     p4           p3
  //      _____________
  //     |             |
  //     |             |
  // *---- 4m(if fwd)    --* 2m + distance(if reverse parking)
  //     |             |
  //     |_____________|            
  // p1        *    p2
  // 
  distance = sqrt(dx*dx + dy*dy);

  dx /= distance;
  dy /= distance;

  if( isFromEnd == true) //From End p2 & p3
  {  
  x1 = point1.x + ((distance - goal_offset) * dx);
  y1 = point1.y + ((distance - goal_offset) * dy);
  }
  else//From Start p1 & p4
  {
  x1 = point1.x + (goal_offset * dx);
  y1 = point1.y + (goal_offset * dy);
  }

  // p4           p3
  //  _________*___-----> Point x meters from p4
  // |             |
  // |             |
  // |             |
  // |_____________|            
  // p1            p2
  //Find a point in a line connecting between p2,p3 using linear interpolation   
  dx = point3.x - point4.x;
  dy = point3.y - point4.y;
  distance = sqrt(dx*dx + dy*dy);
  dx /= distance;
  dy /= distance; 

  if( isFromEnd == true) //From End p2 & p3
  {  
  x2 = point4.x + ((distance - goal_offset) * dx);
  y2 = point4.y + ((distance - goal_offset) * dy);
  }
  else//Forward parking p1 & p4
  {
  x2 = point4.x + (goal_offset * dx);
  y2 = point4.y + (goal_offset * dy);
  }

  // p3           p4
  //  _________*____
  // |         |    |
  // |         *---------> mid point
  // |         |    |
  // |_________*____|            
  // p1            p2
  //Find the mid point 
  goal_pose.x = (x2 + x1)/2;
  goal_pose.y = (y2 + y1)/2;
  */

    return goal_pose;
}

Point MotovisInterface::findIntersection(Point A, Point B, Point C,Point D)
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

Point MotovisInterface::calculateIntersection(Point pointA, Point pointB, double angle)
{
  // Calculate the slope
  //double slope = (pointB.y - pointA.y) / (pointB.x - pointA.x);

  // Convert angle to radians
  double angleRadians = angle * (M_PI / 180.0);

  // Determine the direction of the intersection
  int direction = 1;  // Counterclockwise by default
  if (angle < 0) {
    direction = -1;  // Clockwise
  }

  // Calculate R
  double R = sqrt(pow((pointB.x - pointA.x), 2) + pow((pointB.y - pointA.y), 2));

  // Calculate the intersection point
  Point intersectionPoint;
  intersectionPoint.x = pointA.x + (direction * R * cos(angleRadians));
  intersectionPoint.y = pointA.y + (direction * R * sin(angleRadians));

  return intersectionPoint;
}

double MotovisInterface::getGoalAngleOffsetParallelPark(int parking_direction,int parkinglot_side,double slope_angle,bool isP1P2)
{
  (void) parking_direction;
  (void) parkinglot_side;
  double offset_angle = slope_angle;
  double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  RCLCPP_INFO(get_logger(),"base link yaw %f",yaw * 180/3.141);
  RCLCPP_INFO(get_logger(),"parking_direction:%d parkinglot_side:%d",parking_direction,parkinglot_side);

  if(isP1P2)
  {
    if(slope_angle < 0)
    {
      if(yaw < 0)
      {
        offset_angle = slope_angle + 4.7123;
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
        offset_angle = slope_angle + 4.7123;
      }
      else
      {
        offset_angle = slope_angle + 1.57;
      }
    }
  }
  else //p1&p4
  {
    if(slope_angle < 0)
    {
      if(yaw < 0)
      {
        offset_angle = slope_angle;
      }
      else
      {
        offset_angle = slope_angle + 3.141;
      }
    }
    else
    {
      if(yaw < 0)
      {
        offset_angle = slope_angle + 3.141;
      }
      else
      {
        offset_angle = slope_angle;
      }
    }
  }
  return offset_angle; 
}

int MotovisInterface::pointPosition(Point vehicle, Point referenceVector, Point point)
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

void MotovisInterface::checkGoalAngleOffset(int parkinglot_side,Point point1,Point point4,double *goal_angle_offset)
{
  Point mid_point,check_point;
  Point ego_pose1,ego_pose2;

  mid_point.x = (point1.x + point4.x)/2;
  mid_point.y = (point1.y + point4.y)/2;

  check_point.x = mid_point.x + (100 * cos(*goal_angle_offset));
  check_point.y = mid_point.y + (100 * sin(*goal_angle_offset));

  ego_pose1.x = odometry_->pose.pose.position.x;
  ego_pose1.y = odometry_->pose.pose.position.y;

  ego_pose2.x = odometry_->pose.pose.position.x +
    (5 * cos(tf2::getYaw(odometry_->pose.pose.orientation)));
  ego_pose2.y = odometry_->pose.pose.position.y +
    (5 * sin(tf2::getYaw(odometry_->pose.pose.orientation)));

  int checked_side = pointPosition(ego_pose1,ego_pose2,check_point);
  if(parkinglot_side != checked_side)
  {
    *goal_angle_offset = *goal_angle_offset + 3.141;
    RCLCPP_INFO(get_logger(),"Based on calculated goal_angle checkedside %d is diff from parkinglot_side %d so updated the goal_pose_angle to %lf",checked_side,parkinglot_side,(*goal_angle_offset) * 180/3.141);
  }
}

double MotovisInterface::getGoalAngleOffset(int parking_direction,int parkinglot_side,double slope_angle,bool isP1P2)
{
  double offset_angle = slope_angle;
  double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  RCLCPP_INFO(get_logger(),"base link yaw %f",yaw * 180/3.141);
  RCLCPP_INFO(get_logger(),"parking_direction:%d parkinglot_side:%d",parking_direction,parkinglot_side);

  if(isP1P2)
  {
    switch(parkinglot_side)
    {
      case 0://Left side
        if(parking_direction == 0) // Reverse parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
            else
            {
              offset_angle = slope_angle;
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle + 3.141;
            }
            else
            {
              offset_angle = slope_angle;
            }
          }
        }
        else // Fwd Parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;
            }
            else
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;
            }
            else
            {
              offset_angle = slope_angle + 3.141;
            }
          }
        }
        break;

      case 1: //Right Side
        if(parking_direction == 0) // Reverse parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;
            }
            else
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;     
            }
            else
            {
              offset_angle = slope_angle + 3.141;
            }
          }
        }
        else // Fwd Parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
            else
            {
              offset_angle = slope_angle;
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle + 3.141;
            }
            else
            {
              offset_angle = slope_angle;     
            }
          }
        }
        break;

      case 2://Front Side
        if(parking_direction == 0) // Reverse parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
            else
            {
              offset_angle = slope_angle;
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;     
            }
            else
            {
              offset_angle = slope_angle + 3.141;
            }
          }
        }
        else // Fwd Parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;
            }
            else
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle + 3.141;
            }
            else
            {
              offset_angle = slope_angle;     
            }
          }
        }
        break;

      case 3://Back Side
        if(parking_direction == 0) // Reverse parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;
            }
            else
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle + 3.141;
            }
            else
            {
              offset_angle = slope_angle;     
            }
          }
        }
        else // Fwd Parking
        {
          if(slope_angle < 0)
          {
            if(yaw < 0)
            {
              offset_angle = -(fabs(slope_angle) + 3.141);
            }
            else
            {
              offset_angle = slope_angle;
            }
          }
          else
          {
            if(yaw < 0)
            {
              offset_angle = slope_angle;     
            }
            else
            {
              offset_angle = slope_angle + 3.141;
            }
          }
        }
        break;

      default:
        RCLCPP_ERROR(get_logger(),"Invalid parking lot side!!!");
        break;
    }
  }
  else //p1&p4
  {
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
  }
  return offset_angle; 
}

double MotovisInterface::getSlopeWithRefPlane(Point p1, Point p2)
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

float MotovisInterface::get_angle_3points(Point p1, Point p2, Point p3)
{
  double Dir_C_to_A = atan2(p1.y - p2.y, p1.x - p2.x);
  double Dir_C_to_B = atan2(p3.y - p2.y, p3.x - p2.x);
  double Angle_ACB = Dir_C_to_A - Dir_C_to_B;

  // Handle wrap around
  const double Pi = acos(-1);  // or use some π constant
  if (Angle_ACB > Pi) Angle_ACB -= 2*Pi;
  else if (Angle_ACB < -Pi) Angle_ACB += 2*Pi;

  // Answer is in the range of [-pi...pi]
  return  Angle_ACB;
}
