//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef AMD_STATE_PANEL_HPP_
#define AMD_STATE_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <tier4_planning_msgs/msg/scenario.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace rviz_plugins
{
class AmdScenarioPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AmdScenarioPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickFwdParkingMode();
  void onClickRevParkingMode();
  void onClickParallelParkingMode();
  void onClickDummyParkingLot();
  void onClickParkingCoordinate(); 
  void onClickAvpParking(); 
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr nav_sat_fix_msg_ptr);
protected:

   std::shared_ptr<nav_msgs::msg::Odometry> current_odom_msg_;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr pub_scenario_selector_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_avp_parking_lot_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_parking_pose_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dummy_parking_lot_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

  QLineEdit * parking_lot_lineEdit_ptr_;
  QPushButton * set_avp_parking_button_ptr_;
  QPushButton * fwd_parking_button_ptr_;
  QPushButton * rev_parking_button_ptr_;
  QPushButton * par_parking_button_ptr_;
  QPushButton * set_parking_coordinate_button_ptr_;
  QPushButton * dummy_parking_lot_button_ptr_;
  QDoubleSpinBox * set_parking_coordinate_x_input_; 
  QDoubleSpinBox * set_parking_coordinate_y_input_; 
  QDoubleSpinBox * set_parking_coordinate_xo_input_; 
  QDoubleSpinBox * set_parking_coordinate_yo_input_; 
  QDoubleSpinBox * set_parking_coordinate_zo_input_; 
  QDoubleSpinBox * set_parking_coordinate_w_input_; 
};

}  // namespace rviz_plugins

#endif  // AUTOWARE_STATE_PANEL_HPP_
