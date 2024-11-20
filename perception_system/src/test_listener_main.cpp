/*
The MIT License (MIT)

Copyright (c) 2024 Jose Miguel Guerrero Hernandez

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "perception_system/PerceptionListener.hpp"

using pl = perception_system::PerceptionListener;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    "node_test");
  node->add_activation("/perception_system/perception_people_detection");
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  while (rclcpp::ok()) {
    pl::getInstance(node)->set_interest("person", true);
    pl::getInstance(node)->update(true);
    rclcpp::spin_some(node->get_node_base_interface());

    std::vector<perception_system_interfaces::msg::Detection> detections;
    detections = pl::getInstance(node)->get_by_type("person");

    if (detections.empty()) {
      // RCLCPP_ERROR(node->get_logger(), "No detections");
    }
  }


  rclcpp::shutdown();
  return 0;
}
