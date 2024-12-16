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

#include "amd_scenario_panel.hpp"

#include <QLineEdit>
#include <QHBoxLayout>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>
#include <iostream>
inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }

using std::placeholders::_1;

namespace rviz_plugins
{
AmdScenarioPanel::AmdScenarioPanel(QWidget * parent) : rviz_common::Panel(parent)
{

  // Autoware Engage Button
  auto *  parking_layout = new QHBoxLayout();
  fwd_parking_button_ptr_ = new QPushButton("ForwardPark");
  rev_parking_button_ptr_ = new QPushButton("ReversePark");
  par_parking_button_ptr_ = new QPushButton("ParallelPark");
  connect(fwd_parking_button_ptr_, SIGNAL(clicked()), SLOT(onClickFwdParkingMode()));
  connect(rev_parking_button_ptr_, SIGNAL(clicked()), SLOT(onClickRevParkingMode()));
  connect(par_parking_button_ptr_, SIGNAL(clicked()), SLOT(onClickParallelParkingMode()));
  parking_layout->addWidget(fwd_parking_button_ptr_);
  parking_layout->addWidget(rev_parking_button_ptr_);
  parking_layout->addWidget(par_parking_button_ptr_);

  set_parking_coordinate_button_ptr_ = new QPushButton("Set Parking Co-ordinates");
  connect(set_parking_coordinate_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickParkingCoordinate()));

  auto *  set_parking_lot_layout = new QHBoxLayout();
  parking_lot_lineEdit_ptr_ = new QLineEdit();
  set_avp_parking_button_ptr_ = new QPushButton("AVP");
  connect(set_avp_parking_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickAvpParking()));
  set_parking_lot_layout->addWidget(new QLabel("ParkingLotID:"));
  set_parking_lot_layout->addWidget(parking_lot_lineEdit_ptr_);
  set_parking_lot_layout->addWidget(set_avp_parking_button_ptr_);
  
  auto *  set_parking_coordinate_layout = new QHBoxLayout();
  set_parking_coordinate_x_input_ = new QDoubleSpinBox();
  set_parking_coordinate_x_input_->setRange(-75.0, 75.0);
  set_parking_coordinate_x_input_->setValue(0.0);
  set_parking_coordinate_x_input_->setSingleStep(5.0);
  set_parking_coordinate_layout->addWidget(new QLabel("[x (m)]:"));
  set_parking_coordinate_layout->addWidget(set_parking_coordinate_x_input_);

  set_parking_coordinate_y_input_ = new QDoubleSpinBox();
  set_parking_coordinate_y_input_->setRange(-75.0, 75.0);
  set_parking_coordinate_y_input_->setValue(0.0);
  set_parking_coordinate_y_input_->setSingleStep(5.0);
  set_parking_coordinate_layout->addWidget(new QLabel("[y (m)]:"));
  set_parking_coordinate_layout->addWidget(set_parking_coordinate_y_input_);

  auto *  set_parking_coordinate_orientation_layout = new QHBoxLayout();
  set_parking_coordinate_xo_input_ = new QDoubleSpinBox();
  set_parking_coordinate_xo_input_->setRange(-360.0, 360.0);
  set_parking_coordinate_xo_input_->setValue(0.0);
  set_parking_coordinate_xo_input_->setSingleStep(0.001);
  set_parking_coordinate_orientation_layout->addWidget(new QLabel("[x']:"));
  set_parking_coordinate_orientation_layout->addWidget(set_parking_coordinate_xo_input_);

  set_parking_coordinate_yo_input_ = new QDoubleSpinBox();
  set_parking_coordinate_yo_input_->setRange(-360.0, 360.0);
  set_parking_coordinate_yo_input_->setValue(0.0);
  set_parking_coordinate_yo_input_->setSingleStep(0.001);
  set_parking_coordinate_orientation_layout->addWidget(new QLabel("[y']:"));
  set_parking_coordinate_orientation_layout->addWidget(set_parking_coordinate_yo_input_);

  auto *  set_parking_coordinate_orientation_layout_1 = new QHBoxLayout();
  set_parking_coordinate_zo_input_ = new QDoubleSpinBox();
  set_parking_coordinate_zo_input_->setRange(-360.0, 360.0);
  set_parking_coordinate_zo_input_->setValue(0.0);
  set_parking_coordinate_zo_input_->setSingleStep(0.001);
  set_parking_coordinate_orientation_layout_1->addWidget(new QLabel("[z']:"));
  set_parking_coordinate_orientation_layout_1->addWidget(set_parking_coordinate_zo_input_);

  set_parking_coordinate_w_input_ = new QDoubleSpinBox();
  set_parking_coordinate_w_input_->setRange(-360.0, 360.0);
  set_parking_coordinate_w_input_->setValue(0.0);
  set_parking_coordinate_w_input_->setSingleStep(0.001);
  set_parking_coordinate_orientation_layout_1->addWidget(new QLabel("[w]:"));
  set_parking_coordinate_orientation_layout_1->addWidget(set_parking_coordinate_w_input_);

  dummy_parking_lot_button_ptr_ = new QPushButton("ShowParkingLot");
  connect(dummy_parking_lot_button_ptr_, SIGNAL(clicked()), SLOT(onClickDummyParkingLot()));
  
  // Layout
  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(parking_layout);
  v_layout->addLayout(set_parking_lot_layout);
  //v_layout->addLayout(set_parking_coordinate_layout);
  //v_layout->addLayout(set_parking_coordinate_orientation_layout);
  // v_layout->addLayout(set_parking_coordinate_orientation_layout_1);
  //v_layout->addWidget(set_parking_coordinate_button_ptr_);
  v_layout->addWidget(dummy_parking_lot_button_ptr_);
  setLayout(v_layout);
}

void AmdScenarioPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();


  pub_scenario_selector_ = raw_node_->create_publisher<tier4_planning_msgs::msg::Scenario>(
    "/planning/scenario_planning/scenario", rclcpp::QoS{1}.transient_local());
  pub_parking_pose_ = raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", 1);
  pub_avp_parking_lot_ = raw_node_->create_publisher<std_msgs::msg::String>(
    "/planning/avp_planning/avp_planner/avp_selected_parking_lot", 1);
  pub_dummy_parking_lot_ = raw_node_->create_publisher<std_msgs::msg::Bool>(
    "/perception/motovis_interface/show_parking_lot", 1);
  sub_odometry_ = raw_node_->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&AmdScenarioPanel::onOdometry, this, std::placeholders::_1));
}

void AmdScenarioPanel::onOdometry(nav_msgs::msg::Odometry::SharedPtr msg)
{
   current_odom_msg_ = msg;
}

void AmdScenarioPanel::onClickParkingCoordinate()
{
  geometry_msgs::msg::PoseStamped pose;
  double x  = set_parking_coordinate_x_input_->value();
  double y  = set_parking_coordinate_y_input_->value();

  pose.header.stamp = raw_node_->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = x + current_odom_msg_->pose.pose.position.x;
  pose.pose.position.y = y + current_odom_msg_->pose.pose.position.y;
  pose.pose.orientation.x = set_parking_coordinate_xo_input_->value();
  pose.pose.orientation.y = set_parking_coordinate_yo_input_->value();
  pose.pose.orientation.z = set_parking_coordinate_zo_input_->value();
  pose.pose.orientation.w = set_parking_coordinate_w_input_->value();
  pub_parking_pose_->publish(pose);
}

void AmdScenarioPanel::onClickAvpParking()
{
  QString parking_lot = parking_lot_lineEdit_ptr_->text();
  std_msgs::msg::String parking_lot_msg;
  parking_lot_msg.data = parking_lot.toStdString();
  pub_avp_parking_lot_->publish(parking_lot_msg);
}

void AmdScenarioPanel::onClickFwdParkingMode()
{
  tier4_planning_msgs::msg::Scenario scenario;
  scenario.current_scenario = "FWD";
  scenario.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::PARKING);
  pub_scenario_selector_->publish(scenario);
}

void AmdScenarioPanel::onClickRevParkingMode()
{
  tier4_planning_msgs::msg::Scenario scenario;
  scenario.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::PARKING);
  scenario.current_scenario = "REV";
  pub_scenario_selector_->publish(scenario);
}

void AmdScenarioPanel::onClickParallelParkingMode()
{
  tier4_planning_msgs::msg::Scenario scenario;
  scenario.activating_scenarios.push_back(tier4_planning_msgs::msg::Scenario::PARKING);
  scenario.current_scenario = "PAR";
  pub_scenario_selector_->publish(scenario);
}

void AmdScenarioPanel::onClickDummyParkingLot()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  pub_dummy_parking_lot_->publish(msg);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AmdScenarioPanel, rviz_common::Panel)
