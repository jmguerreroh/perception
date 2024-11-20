/*
  Copyright (c) 2024 José Miguel Guerrero Hernández

  Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      https://creativecommons.org/licenses/by-sa/4.0/

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include "perception_system/PerceptionListener.hpp"

using pl = perception_system::PerceptionListener;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    "node_test");

  while (rclcpp::ok()) {
    pl::getInstance(node)->set_interest("person", true);
    pl::getInstance(node)->update(true);
    rclcpp::spin_some(node->get_node_base_interface());

    std::vector<perception_system_interfaces::msg::Detection> detections;
    detections = pl::getInstance(node)->get_by_type("person");

    if (detections.empty()) {
      RCLCPP_ERROR(node->get_logger(), "No detections");
    }
  }
  

  rclcpp::shutdown();
  return 0;
}
