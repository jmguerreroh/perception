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

#ifndef PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_
#define PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "perception_system_interfaces/msg/detection.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"
#include "perception_system/PerceptionUtils.hpp"


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace perception_system
{

struct PerceptionData
{
  std::string type;
  perception_system_interfaces::msg::Detection msg;
  rclcpp::Time time;
};

struct PerceptionInterest
{
  bool status;
  rclcpp::Time time;
};

class PerceptionListener
{
public:
  static std::shared_ptr<PerceptionListener> getInstance(
    std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node)
  {
    if (uniqueInstance_ == nullptr) {
      uniqueInstance_ = std::make_shared<PerceptionListener>(parent_node);
    }
    return uniqueInstance_;
  }

  explicit PerceptionListener(
    std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node);

  virtual ~PerceptionListener() {}

  void update(double hz = 30);

  void set_interest(const std::string & type, bool status = true);
  std::vector<perception_system_interfaces::msg::Detection> get_by_id(const std::string & id);
  std::vector<perception_system_interfaces::msg::Detection> get_by_type(const std::string & type);
  std::vector<perception_system_interfaces::msg::Detection> get_by_features(
    const perception_system_interfaces::msg::Detection & object, const float confidence = 0.4);
  std::vector<perception_system_interfaces::msg::Detection> get_detection_at(
    const geometry_msgs::msg::TransformStamped & position);
  // directly publish the TF
  int publishTF_suffix(
    const perception_system_interfaces::msg::Detection & detected_object,
    const std::string & custom_suffix = "");
  int publishTF_EKF(
    const perception_system_interfaces::msg::Detection & detected_object,
    const std::string & frame_name = "", const bool useEKF = true);
  void publishTFinterest();
  // publish tfs with custom sorting
  void publishSortedTFinterest(
    std::function<bool(const perception_system_interfaces::msg::Detection &,
    const perception_system_interfaces::msg::Detection &)> comp = [] (const
    perception_system_interfaces::msg::Detection & a,
    const perception_system_interfaces::msg::Detection & b) {
      // Default sorting behavior
      return a.center3d.position.z < b.center3d.position.z;
    });

private:
  static std::shared_ptr<PerceptionListener> uniqueInstance_;
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node_;

  rclcpp::Subscription<perception_system_interfaces::msg::DetectionArray>::SharedPtr percept_sub_;
  std::map<std::string, PerceptionInterest> interests_;
  std::map<std::string, PerceptionData> perceptions_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  perception_system_interfaces::msg::DetectionArray::UniquePtr last_msg_;

  void perception_callback(perception_system_interfaces::msg::DetectionArray::UniquePtr msg);


  double max_time_perception_;
  double max_time_interest_;
  rclcpp::Time last_update_;
  rclcpp::Time last_msg_time_;

  std::string tf_frame_camera_;
  std::string tf_frame_map_;

  tf2::Transform previousPosition_EKF_;
  bool firstEKF_ = true;
};

}  // namespace perception_system

std::shared_ptr<perception_system::PerceptionListener> perception_system::PerceptionListener::
uniqueInstance_;


#endif  // PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_
