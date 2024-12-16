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

#include "module_mode_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }

using std::placeholders::_1;

namespace rviz_plugins
{
ModuleModePanel::ModuleModePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // System
  auto * system_prefix_label_ptr = new QLabel("SYSTEM: ");
  system_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  system_mode_label_ptr_ = new QLabel("INIT");
  system_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * system_layout = new QHBoxLayout;
  system_layout->addWidget(system_prefix_label_ptr);
  system_layout->addWidget(system_mode_label_ptr_);

  // Vehicle
  auto * vehicle_prefix_label_ptr = new QLabel("VEHICLE: ");
  vehicle_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  vehicle_mode_label_ptr_ = new QLabel("INIT");
  vehicle_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * vehicle_layout = new QHBoxLayout;
  vehicle_layout->addWidget(vehicle_prefix_label_ptr);
  vehicle_layout->addWidget(vehicle_mode_label_ptr_);

 // Control
  auto * control_prefix_label_ptr = new QLabel("CONTROL: ");
  control_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  control_mode_label_ptr_ = new QLabel("INIT");
  control_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * control_layout = new QHBoxLayout;
  control_layout->addWidget(control_prefix_label_ptr);
  control_layout->addWidget(control_mode_label_ptr_);

 // Planning
  auto * planning_prefix_label_ptr = new QLabel("PLANNING: ");
  planning_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  planning_mode_label_ptr_ = new QLabel("INIT");
  planning_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * planning_layout = new QHBoxLayout;
  planning_layout->addWidget(planning_prefix_label_ptr);
  planning_layout->addWidget(planning_mode_label_ptr_);

 // Localization
  auto * localization_prefix_label_ptr = new QLabel("LOCALIZATION: ");
  localization_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  localization_mode_label_ptr_ = new QLabel("INIT");
  localization_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * localization_layout = new QHBoxLayout;
  localization_layout->addWidget(localization_prefix_label_ptr);
  localization_layout->addWidget(localization_mode_label_ptr_);

 // Perception
  auto * perception_prefix_label_ptr = new QLabel("PERCEPTION: ");
  perception_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  perception_mode_label_ptr_ = new QLabel("INIT");
  perception_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * perception_layout = new QHBoxLayout;
  perception_layout->addWidget(perception_prefix_label_ptr);
  perception_layout->addWidget(perception_mode_label_ptr_);

  // Sensing
  auto * sensing_prefix_label_ptr = new QLabel("SENSING: ");
  sensing_prefix_label_ptr->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  sensing_mode_label_ptr_ = new QLabel("INIT");
  sensing_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  auto * sensing_layout = new QHBoxLayout;
  sensing_layout->addWidget(sensing_prefix_label_ptr);
  sensing_layout->addWidget(sensing_mode_label_ptr_);



  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(system_layout);
  v_layout->addLayout(vehicle_layout);
  v_layout->addLayout(control_layout);
  v_layout->addLayout(planning_layout);
  v_layout->addLayout(localization_layout);
  v_layout->addLayout(perception_layout);
  v_layout->addLayout(sensing_layout);

  setLayout(v_layout);
}

void ModuleModePanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // System Mode
  sub_system_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/system", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onSystemMode, this, _1));
  // Vehicle Mode
  sub_vehicle_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/vehicle", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onVehicleMode, this, _1));
 
  // Control Mode
  sub_control_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/control", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onControlMode, this, _1));

  // planning Mode
  sub_planning_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/planning", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onPlanningMode, this, _1));

  // localization Mode
  sub_localization_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/localization", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onLocalizationMode, this, _1));
  
  // Perception Mode
  sub_perception_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/perception", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onPerceptionMode, this, _1));
  
  // Sensing Mode
  sub_sensing_mode_ = raw_node_->create_subscription<ModeChangeAvailable>(
    "/system/component_state_monitor/component/autonomous/sensing", rclcpp::QoS{1}.transient_local(),
    std::bind(&ModuleModePanel::onSensingMode, this, _1));
}

void ModuleModePanel::onSystemMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(system_mode_label_ptr_, msg);
}

void ModuleModePanel::onVehicleMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(vehicle_mode_label_ptr_, msg);
}


void ModuleModePanel::onControlMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(control_mode_label_ptr_, msg );
}

void ModuleModePanel::onPlanningMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(planning_mode_label_ptr_, msg );
}

void ModuleModePanel::onLocalizationMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(localization_mode_label_ptr_, msg );
}

void ModuleModePanel::onPerceptionMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(perception_mode_label_ptr_, msg );
}

void ModuleModePanel::onSensingMode(const ModeChangeAvailable::ConstSharedPtr msg)
{
  updateLabel(sensing_mode_label_ptr_, msg );
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ModuleModePanel, rviz_common::Panel)
