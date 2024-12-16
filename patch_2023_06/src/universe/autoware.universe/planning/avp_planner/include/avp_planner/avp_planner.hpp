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

#ifndef AVP_PLANNER_HPP_
#define AVP_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <autoware_ad_api_specs/routing.hpp>
#include <component_interface_utils/rclcpp.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <motovis_interface_msgs/msg/parking_lots.hpp>

#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std_msgs::msg::Header;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseArray = geometry_msgs::msg::PoseArray;
using Pose = geometry_msgs::msg::Pose;
using PolygonStamped = geometry_msgs::msg::PolygonStamped;
using Point32 = geometry_msgs::msg::Point32;
using Point = geometry_msgs::msg::Point;
using ColorRGBA = std_msgs::msg::ColorRGBA;
using Bool = std_msgs::msg::Bool;
using String = std_msgs::msg::String;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using Scenario = tier4_planning_msgs::msg::Scenario;
using Odometry = nav_msgs::msg::Odometry;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using GearReport = autoware_auto_vehicle_msgs::msg::GearReport; 
using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport; 
using ParkingLots = motovis_interface_msgs::msg::ParkingLots;
using ParkingLot = motovis_interface_msgs::msg::ParkingLot;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using route_handler::RouteHandler;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcLongitudinalOffsetToSegment;
using motion_utils::calcSignedArcLength;
using motion_utils::findFirstNearestIndexWithSoftConstraints;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::Point2d;
using SetRoutePoints = autoware_ad_api::routing::SetRoutePoints;
using ClearRoute = autoware_ad_api::routing::ClearRoute;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct NodeParam
{
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_velocity_mps;
  double th_stopped_time_sec;

  // dist threshold for ego's nearest index
  double ego_nearest_dist_threshold;

  // yaw threshold for ego's nearest index
  double ego_nearest_yaw_threshold;

  //Points in trajectory to be considered for smooth stop
  double ego_stop_scale_factor;
};

struct VehicleShape
{
  double length;     // X [m]
  double width;      // Y [m]
  double base2back;  // base_link to rear [m]
  double wheel_base; // front to rear center
};

struct StopPoint
{
  TrajectoryPoint point{};

  size_t index;
};

class AvpPlanner : public rclcpp::Node
{
public:
  AvpPlanner(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~AvpPlanner();

private:

  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr checkpoint_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_checkpoint_array_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<ParkingLots>::SharedPtr parkinglot_sub_;
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Subscription<String>::SharedPtr avp_parking_lot_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_sub_;

  component_interface_utils::Client<SetRoutePoints>::SharedPtr cli_route_;
  component_interface_utils::Client<ClearRoute>::SharedPtr cli_clear_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Odometry::ConstSharedPtr odom_ {nullptr};
  std::deque<Odometry::ConstSharedPtr> odom_buffer_;
  std::vector<Pose> waypoints_;
  Pose current_pose_;
  bool is_avp_activated_{false};
  bool isSetStop_{false};
  bool isParkingGoalSet_{false};
  bool isSetPointSetAlready_{false};
  TrajectoryPoints stop_trajectory_points_;
  long unsigned int waypoint_count_;
  VehicleShape vehicle_shape_;
  ParkingLots::ConstSharedPtr parkinglot_arr_ptr_;
  Scenario::ConstSharedPtr scenario_;

  std::mutex parking_lot_mutex_;
  std::mutex mutex_map_;      // mutex for has_received_map_ and map_ptr_
  std::mutex mutex_route_;    // mutex for has_received_route_ and route_ptr_
  std::mutex waypoint_mtx_;
  HADMapBin::ConstSharedPtr map_ptr_{nullptr};
  LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  std::string selected_parkinglot_id_;

  std::shared_ptr<RouteHandler> route_handler_ptr_{std::make_shared<RouteHandler>()};
  void onMap(const HADMapBin::ConstSharedPtr map_msg);
  void onAvpParkingLot(const String::ConstSharedPtr parking_lot_msg);
  void onParkinglotArray(ParkingLots::ConstSharedPtr msg);
  void onTimer();
  void onTrajectory(const Trajectory::ConstSharedPtr input_msg);

  // params
  NodeParam node_param_;

  // Get current pose
  geometry_msgs::msg::TransformStamped tf_;

  void computeParkingGoalPose(int id);
  bool isParkingLotAvailable();
  void on_odometry(const Odometry::ConstSharedPtr msg);
  void onParkingLotPose(const PoseStamped::ConstSharedPtr msg);
  void onScenario(const Scenario::ConstSharedPtr msg);
  void insertStopPoint(const StopPoint & stop_point, TrajectoryPoints & output);
  void insertStopVelocity(
    TrajectoryPoints & output,const double current_vel, const Pose ego_pos);
  bool isInParkingLot(const std::string parking_lot_id,const geometry_msgs::msg::Pose & current_pose);
  bool isInParkingLot(const std::string parking_lot_id,const geometry_msgs::msg::Point & current_point);
  void checkGoalAngleOffset(int parkinglot_side,Point point1,Point point4,double *goal_angle_offset);
  double getSlopeWithRefPlane(Point p1, Point p2);
  double getGoalAngleOffset(int parking_direction,int parkinglot_side,double slope_angle);
  PoseStamped findParkingGoalPose(Point point1,Point point2,Point point3,
    Point point4,int parking_direction,int parkinglot_side);
  PoseStamped planBackPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset);
  PoseStamped planFrontPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset);
  PoseStamped planLeftPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset);
  PoseStamped planRightPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset);
 int pointPosition(Point vehicle, Point referenceVector, Point point);
 Point findIntersection(Point A, Point B, Point C,Point D);
 int findBestParkingLot(const std::string pid);
};
#endif
