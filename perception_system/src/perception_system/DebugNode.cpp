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

#include "perception_system/DebugNode.hpp"

namespace perception_system
{

DebugNode::DebugNode(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("perception_debug", "perception_system", options)
{
  this->declare_parameter("target_frame", "head_front_camera_link");
}

CallbackReturnT DebugNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(),
    state.label().c_str());

  this->get_parameter("target_frame", frame_id_);

  std::string topic_name = "bb_objects";
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    topic_name, 10);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT DebugNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(),
    state.label().c_str());

  std::string topic_name = "all_perceptions";
  sub_ = this->create_subscription<perception_system_interfaces::msg::DetectionArray>(
    topic_name, 10,
    std::bind(&DebugNode::callback, this, std::placeholders::_1));

  markers_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT DebugNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(),
    state.label().c_str());

  sub_ = nullptr;

  markers_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT DebugNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleaning up from [%s] state...", get_name(),
    state.label().c_str());

  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT DebugNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting down from [%s] state...", get_name(),
    state.label().c_str());

  markers_pub_.reset();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT DebugNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Erroring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

void DebugNode::callback(
  const perception_system_interfaces::msg::DetectionArray::ConstSharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (auto detection : msg->detections) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = detection.header.stamp;
    marker.ns = "perception_system";
    marker.id = detection.class_id;
    marker.text = detection.class_name;
    marker.frame_locked = false;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = detection.center3d;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale = detection.bbox3d;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_array.markers.push_back(marker);
  }

  markers_pub_->publish(marker_array);
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::DebugNode)
