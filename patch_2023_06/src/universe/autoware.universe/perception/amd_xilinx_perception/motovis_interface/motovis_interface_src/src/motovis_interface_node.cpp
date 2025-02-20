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

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<MotovisInterface>("motovis_interface", node_options);
  executor.add_node(node);
  executor.spin();
  //rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
