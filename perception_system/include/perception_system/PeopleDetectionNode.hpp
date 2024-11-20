/*
MIT License

Copyright (c) 2024 José Miguel Guerrero Hernández

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

#ifndef PERCEPTION_SYSTEM__PEOPLE_DETECTION_NODE_HPP_
#define PERCEPTION_SYSTEM__PEOPLE_DETECTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "yolo_msgs/msg/detection_array.hpp"
#include "yolo_msgs/msg/key_point2_d_array.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "cascade_lifecycle_msgs/msg/activation.hpp"
#include "cascade_lifecycle_msgs/msg/state.hpp"

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include "perception_system/PerceptionUtils.hpp"

namespace perception_system
{

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PeopleDetectionNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  PeopleDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

private:
  void callback(const yolo_msgs::msg::DetectionArray::ConstSharedPtr & msg);

  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<perception_system_interfaces::msg::DetectionArray>::SharedPtr
    pub_;

  std::string frame_id_;
};

}  // namespace perception_system

#endif  // PERCEPTION_SYSTEM__PEOPLE_DETECTION_NODE_HPP_
