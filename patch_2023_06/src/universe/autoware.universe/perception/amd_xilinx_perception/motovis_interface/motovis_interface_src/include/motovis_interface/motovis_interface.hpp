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

#ifndef MOTOVIS_INTERFACE_HPP_
#define MOTOVIS_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <motovis_interface_msgs/srv/update_goal_pose.hpp>
#include <motovis_interface_msgs/msg/parking_lots.hpp>

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

#include "slotdetect_wrapper.h"
#include "sensor_msgs/msg/image.hpp"
//For SharedMem
#include "SharedMemoryInterface.h"

#define BEV_X_LOC 800
#define BEV_Y_LOC 300
#define CAR_GRAPHIC_X_LOC BEV_X_LOC+270
#define CAR_GRAPHIC_Y_LOC BEV_Y_LOC+183

struct VehicleShape
{
  double length;     // X [m]
  double width;      // Y [m]
  double base2back;  // base_link to rear [m]
  double wheel_base; // front to rear center
};

struct NodeParam
{
  std::string background_png_file;
  std::string vehicle_png_file;
  double update_rate;         // replanning and publishing rate [Hz]
  int num_of_cams;         // No of cameras
  bool use_simulated_parking;
  bool share_bev_to_disti_hmi;
  bool use_vehicle_overlay_on_bev;
  bool publish_bev;
};

enum button_selection_t
{ 
  B_NONE=0,
  B_REVERSE=1,
  B_FORWARD=2,
  B_CANCEL=3,
  B_CONFIRM=4
};

enum class APAState {
  INIT,
  NO_SLOT_DETECTED,
  SLOT_DETECTED,
  VEHICLE_STOPPED,
  SLOT_SELECTED,
  FORWARD_PARK,
  REVERSE_PARK,
  CONFIRM,
  DONE,
  EMERGENCY,
  CANCELLED,
  UNDEFINED
};

struct button_selected_t {
  cv::Point upper_left;
  cv::Point lower_right;
};

using std::placeholders::_1;
using std::placeholders::_2;

using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseArray = geometry_msgs::msg::PoseArray;
using Pose = geometry_msgs::msg::Pose;
using PolygonStamped = geometry_msgs::msg::PolygonStamped;
using Point32 = geometry_msgs::msg::Point32;
using Point = geometry_msgs::msg::Point;
using ColorRGBA = std_msgs::msg::ColorRGBA;
using Bool = std_msgs::msg::Bool;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using Scenario = tier4_planning_msgs::msg::Scenario;
using Odometry = nav_msgs::msg::Odometry;
using UpdateGoalPose = motovis_interface_msgs::srv::UpdateGoalPose;
using ParkingLots = motovis_interface_msgs::msg::ParkingLots;
using ParkingLot = motovis_interface_msgs::msg::ParkingLot;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using GearReport = autoware_auto_vehicle_msgs::msg::GearReport; 
using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport; 
class MotovisInterface : public rclcpp::Node
{
public:
  MotovisInterface(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~MotovisInterface();

private:
  //For ShmMem
  SharedMemoryInterface shm_interface_;

  std::shared_ptr<std::thread> can_pub_thread_;
  std::shared_ptr<std::thread> bev_pub_thread_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr debug_freespace_pub_;  //!< @brief freespace publisher
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_parkinglot_marker_pub_;  //!< @brief parkinglot publisher
  rclcpp::Publisher<ParkingLots>::SharedPtr parkinglot_pub_;  //!< @brief parkinglot publisher
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pose_pub_;  //!< @brief parkinglot publisher
  rclcpp::Publisher<PoseStamped>::SharedPtr updated_goal_pose_pub_;  //!< @brief parkinglot publisher
  rclcpp::Publisher<PoseStamped>::SharedPtr checkpoint_pub_;  //!< @brief parkinglot publisher
  rclcpp::Publisher<PoseArray>::SharedPtr checkpoint_array_pub_;  //!< @brief parkinglot publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bev_publisher_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_debug_velocity_report_;
  rclcpp::TimerBase::SharedPtr timer_cnn_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_sub_;
  rclcpp::Subscription<Bool>::SharedPtr show_parking_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr parking_lot_pose_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;

  rclcpp::Service<UpdateGoalPose>::SharedPtr goal_pose_srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Odometry::ConstSharedPtr odometry_;
  AutowareState::ConstSharedPtr autoware_state_;
  OperationModeState::ConstSharedPtr operation_mode_;
  GearReport::ConstSharedPtr vehicle_gear_rpt_;

  // params
  NodeParam node_param_;
  Scenario::ConstSharedPtr scenario_;
  bool parkinglot_fixed_;
  VehicleShape vehicle_shape_;
  Point selected_parking_lot_pose_;
  double goal_angle_offset_;
  unsigned long long ulBv2dAddr_;
  Marker selected_parkinglot_marker_;
  PoseStamped end_goal_pose_,wp1_pose_,wp2_pose_;
  cv::Mat  bev_8uc4_;
  cv::Mat  bev_background_;
  cv::Mat  bev_vehicle_;

  std::thread display_thread;
  exSlotResult tSlot_;

  typedef message_filters::sync_policies::ApproximateTime<pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::WheelSpeedRpt, pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::GlobalRpt, pacmod3_msgs::msg::SystemRptInt> PacmodFeedbacksSyncPolicy;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>
	  steer_wheel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>
	  wheel_speed_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>> global_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  std::unique_ptr<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>> pacmod_feedbacks_sync_;

  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt_ptr_;  // [m/s]
  pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;       // [m/s]
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;

  void callbackPacmodRpt(
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt);

  // Get current pose
  geometry_msgs::msg::TransformStamped tf_;

  /*
  //HMI
  cv::Mat hmi_mat_default_;
  cv::Mat hmi_mat_;
  button_selected_t forward_park_button_;
  button_selected_t reverse_park_button_;
  button_selected_t cancel_button_;
  button_selected_t confirm_button_;
  APAState curr_apa_state_;
  APAState prev_apa_state_;
  int hmi_selected_slot_id_;
  exSlotData hmi_selected_slot_;
  std::atomic_bool touchscreen_event_ = false;
  void display_hmi();
  void hmi_Init();
  cv::Point map_coord_to_cv_points(float y, float x);
  void drawSlotsOnBev();
  void drawSelectedSlotOnBev();
  int getSelectedSlot();
  button_selection_t getCurrentButtonSelection();
  double updateHmiState();
  void onScenarioFromHmi(int id,int parking_direction);
  std::string toStr(const APAState s)
  {
    if (s == APAState::INIT) return "INIT";
    if (s == APAState::NO_SLOT_DETECTED) return "NO_SLOT_DETECTED";
    if (s == APAState::SLOT_DETECTED) return "SLOT_DETECTED";
    if (s == APAState::VEHICLE_STOPPED) return "VEHICLE_STOPPED";
    if (s == APAState::SLOT_SELECTED) return "SLOT_SELECTED";
    if (s == APAState::FORWARD_PARK) return "FORWARD_PARK";
    if (s == APAState::REVERSE_PARK) return "REVERSE_PARK";
    if (s == APAState::CONFIRM) return "CONFIRM";
    if (s == APAState::DONE) return "DONE";
    if (s == APAState::EMERGENCY) return "EMERGENCY";
    return "UNDEFINED";
  };

  bool isGearParked()
  {
    if(vehicle_gear_rpt_)
    {
      if(vehicle_gear_rpt_->report == GearReport::PARK)
      {
        return true;
      }
    }
    return false;
  }

  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_stop_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_enable_autoware_control_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_enable_direct_control_;

  template <typename T>
  bool callServiceWithResponse(const typename rclcpp::Client<T>::SharedPtr client)
  {
    const std::chrono::nanoseconds & timeout = std::chrono::seconds(1);
    auto req = std::make_shared<typename T::Request>();

    while(true)
    {
      RCLCPP_INFO(this->get_logger(), "client request");

      if (!client->service_is_ready()) {
        RCLCPP_INFO(this->get_logger(), "client is unavailable");
        return false;
      }

      auto future = client->async_send_request(req);
      if (future.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_INFO(this->get_logger(), "client timeout");
        return false;
      }
      auto result = future.get();
      RCLCPP_INFO(get_logger(), "Status: %d %d, %s", (int) result->status.success,
          result->status.code,result->status.message.c_str());
      if(result->status.success)
      {
        return true;
      }
      else if(result->status.code == 2)
      {
        //If operational mode is in transition try again
        usleep(100);
        continue;
      }
      return false;
    }
  }

  int getSlotCount();
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  */

  void updateBev();
  void publish_bev();
  void on_odometry(const Odometry::ConstSharedPtr msg);
  void onScenario(const Scenario::ConstSharedPtr msg); 
  void onParkingLotPose(const PoseStamped::ConstSharedPtr msg); 
  void onShowParkingLots(const Bool::ConstSharedPtr msg);
  void onGoalPoseRequest(
  const UpdateGoalPose::Request::SharedPtr request,
  const UpdateGoalPose::Response::SharedPtr response);

  void onCNNTimer();
  void can_publish();
  void publish_freespace();
  void publish_parkinglot();
  void PushCanInfo_to_motovis();
  bool isActive(const Scenario::ConstSharedPtr & scenario);
  void transformPoint(Point *point);
  void transformPointToBaseFrame(Point *point);
  Point findGoalPose(Point p1,Point p2,Point p3,Point p4,bool isFromEnd,double dis_offset);
  void display_bev();
  Point calculateIntersection(Point pointA, Point pointB, double angleRadians);
  bool isParkingLotSelected(int id);
  void getTransform();
  void updateGoalPose(Point point1,Point point2,Point point3,Point point4);
  double getGoalAngleOffset(int parking_direction,int parkinglot_side,double slope_angle,bool isP1P2);
  Point findIntersection(Point A, Point B, Point C,Point D);
  double getSlopeWithRefPlane(Point p1, Point p2);
  float get_angle_3points(Point p1, Point p2, Point p3);
  void storeInitialSelectedParkingLot();
  int getSelectedParkingLotID();
  bool detectParallelParking(Point point1,Point point2,Point point3,Point point4);
  Point planBackPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2);
  Point planFrontPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2);
  Point planLeftPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2);
  Point planRightPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,double goal_angle_offset,bool isP1P2);
  Point planParallelPark(Point point1,Point point2,Point point3,Point point4,int parking_direction,int parkinglot_side,double goal_angle_offset,bool isP1P2);
  double getGoalAngleOffsetParallelPark(int parking_direction,int parkinglot_side,double slope_angle,bool isP1P2);
  int pointPosition(Point vehicle, Point referenceVector, Point point); 
  void checkGoalAngleOffset(int parkinglot_side,Point point1,Point point4,double *goal_angle_offset);

};
#endif
