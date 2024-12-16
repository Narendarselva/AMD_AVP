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

#ifndef MODULE_MODE_PANEL_HPP_
#define MODULE_MODE_PANEL_HPP_

#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>


#include <memory>

namespace rviz_plugins
{
class ModuleModePanel : public rviz_common::Panel
{
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;
  Q_OBJECT

public:
  explicit ModuleModePanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:

  rclcpp::Node::SharedPtr raw_node_;

  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_system_mode_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_vehicle_mode_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_control_mode_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_planning_mode_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_localization_mode_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_perception_mode_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_sensing_mode_;

  //// Lable Mode
  QLabel * system_mode_label_ptr_{nullptr};
  QLabel * vehicle_mode_label_ptr_{nullptr};
  QLabel * control_mode_label_ptr_{nullptr};
  QLabel * planning_mode_label_ptr_{nullptr};
  QLabel * localization_mode_label_ptr_{nullptr};
  QLabel * perception_mode_label_ptr_{nullptr};
  QLabel * sensing_mode_label_ptr_{nullptr};


  //// Functions
  void onSystemMode(const ModeChangeAvailable::ConstSharedPtr msg);
  void onVehicleMode(const ModeChangeAvailable::ConstSharedPtr msg);
  void onControlMode(const ModeChangeAvailable::ConstSharedPtr msg);
  void onPlanningMode(const ModeChangeAvailable::ConstSharedPtr msg);
  void onLocalizationMode(const ModeChangeAvailable::ConstSharedPtr msg);
  void onPerceptionMode(const ModeChangeAvailable::ConstSharedPtr msg);
  void onSensingMode(const ModeChangeAvailable::ConstSharedPtr msg);

  static void updateLabel(QLabel * label, const ModeChangeAvailable::ConstSharedPtr msg)
  {

	  QString text = "";
	  QString style_sheet = "";

	  if(msg->available)
	  {
		  text = "OK";
		  style_sheet = "background-color: #00FF00;";  // green

	  }
	  else
	  {
		  text = "ERROR";
		  style_sheet = "background-color: #FF0000;";  // red
	  }

	  label->setText(text);
	  label->setStyleSheet(style_sheet);
  }

};

}  // namespace rviz_plugins

#endif  // Module_mode_PANEL_HPP_
