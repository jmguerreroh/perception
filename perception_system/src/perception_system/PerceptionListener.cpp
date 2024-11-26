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


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "perception_system_interfaces/msg/detection.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "perception_system/PerceptionListener.hpp"


namespace perception_system
{
using namespace std::chrono_literals;

PerceptionListener::PerceptionListener(
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node)
: parent_node_(parent_node)
{
  parent_node_->declare_parameter("max_time_perception", 0.01);
  parent_node_->declare_parameter("max_time_interest", 0.01);
  parent_node_->declare_parameter("debug", false);
  parent_node_->declare_parameter("tf_frame_camera", "oak_rgb_camera_optical_frame");
  parent_node_->declare_parameter("tf_frame_map", "base_footprint");

  parent_node_->get_parameter("max_time_perception", max_time_perception_);
  parent_node_->get_parameter("max_time_interest", max_time_interest_);
  parent_node_->get_parameter("tf_frame_camera", tf_frame_camera_);
  parent_node_->get_parameter("tf_frame_map", tf_frame_map_);
  if (parent_node_->get_parameter("debug").as_bool()) {
    parent_node_->add_activation("yolo_debug_node");
  }

  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_node_->get_clock(), 120ms); // 30 Hz liveliness so it is always updated

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(parent_node_.get());

  std::string topic_name = "all_perceptions";
  percept_sub_ = parent_node_->create_subscription<perception_system_interfaces::msg::DetectionArray>(
    topic_name, 10,
    std::bind(&PerceptionListener::perception_callback, this, std::placeholders::_1));
}

void PerceptionListener::perception_callback(
  perception_system_interfaces::msg::DetectionArray::UniquePtr msg)
{
  last_msg_ = std::move(msg);
}

// Check if last_perceptions_ elements are too old, and remove it
// Check if interests are too old, and call set_interest( , false)
void PerceptionListener::update(double hz)
{
  // Check if last_update_ is less than hz_callback, and return
  auto now_update = rclcpp::Clock(RCL_STEADY_TIME).now();
  auto diff_update = now_update - last_update_;
  auto seconds = 1.0 / hz;
  if (diff_update.seconds() < seconds) {
    return;
  }

  if (last_msg_ != nullptr) {
    for (auto & detection : last_msg_->detections) {
      // If it is in last_perceptions_ update, if not, create a new element in the vector
      auto det = perceptions_.find(detection.unique_id);
      if (det != perceptions_.end()) {
        det->second.msg = detection;

        det->second.time = rclcpp::Clock(RCL_STEADY_TIME).now();
      } else {
        perceptions_.emplace(
          std::pair<std::string, PerceptionData>(
            detection.unique_id,
            {detection.type, detection, rclcpp::Clock(RCL_STEADY_TIME).now()}));
      }
    }
  }

  // Check if last_perceptions_ elements are too old, and remove it
  std::vector<std::string> to_remove_perceptions;
  for (auto & perception : perceptions_) {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - perception.second.time;
    if (diff.seconds() > max_time_perception_) {
      to_remove_perceptions.push_back(perception.first);
    }
  }

  for (auto & id : to_remove_perceptions) {
    perceptions_.erase(id);
  }

  // Check if interests are too old, and call set_interest( , false)
  std::vector<std::string> to_remove_interests;
  for (auto & interest : interests_) {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - interest.second.time;
    if (diff.seconds() > max_time_interest_) {
      to_remove_interests.push_back(interest.first);
    }
  }

  for (auto & id : to_remove_interests) {
    set_interest(id, false);
  }

  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();
}

// if status == true, add to interests_ and call add_activation
// if status == false, remove to interests_ and call remove_activation
void PerceptionListener::set_interest(const std::string & id, bool status)
{
  if (status) {
    if (interests_.find(id) == interests_.end()) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      interests_.insert(
        std::pair<std::string, PerceptionInterest>(
          id,
          {status, steady_clock.now()}));
      if (id.find("person") != std::string::npos) {
        parent_node_->add_activation("perception_people_detection");
      } else {
        parent_node_->add_activation("perception_objects_detection");
      }
      RCLCPP_INFO(parent_node_->get_logger(), "Added interest: %s, %d", id.c_str(), status);
    } else {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      interests_.find(id)->second.time = steady_clock.now();
    }
  } else {
    interests_.erase(id);
    if (id.find("person") != std::string::npos) {
      parent_node_->remove_activation("perception_people_detection");
    } else {
      parent_node_->remove_activation("perception_objects_detection");
    }
    RCLCPP_DEBUG(parent_node_->get_logger(), "Removed interest: %s, %d", id.c_str(), status);
  }

}

// Create a vector from perceptions_ whose id match
std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_id(const std::string & id)
{
  std::vector<perception_system_interfaces::msg::Detection> result;
  for (auto & perception : perceptions_) {
    if (perception.first == id) {
      result.push_back(perception.second.msg);
    }
  }
  return result;
}

// Create a vector from perceptions_ whose type match
std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_type(const std::string & type)
{
  std::vector<perception_system_interfaces::msg::Detection> result;
  for (auto & perception : perceptions_) {
    if (type.empty() || perception.second.type == type) {
      result.push_back(perception.second.msg);
    }
  }
  return result;
}

// Create a transform from map to object and send it
int
PerceptionListener::publishTF(
  const perception_system_interfaces::msg::Detection & detected_object,
  const std::string & custom_suffix)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;
  try {
    map2camera_msg = tf_buffer_->lookupTransform(
      tf_frame_map_, tf_frame_camera_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      parent_node_->get_logger(), "Could not transform %s to %s: %s",
      tf_frame_map_.c_str(), tf_frame_camera_.c_str(), ex.what());
    return -1;
  }

  tf2::Transform camera2object;
  camera2object.setOrigin(
    tf2::Vector3(
      detected_object.center3d.position.x,
      detected_object.center3d.position.y,
      detected_object.center3d.position.z));
  camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2object = map2camera * camera2object;

  // create a transform message from tf2::Transform
  geometry_msgs::msg::TransformStamped map2object_msg;
  map2object_msg.header.stamp = detected_object.header.stamp;
  map2object_msg.header.frame_id = tf_frame_map_;
  map2object_msg.child_frame_id =
    (custom_suffix.empty()) ? detected_object.unique_id : (detected_object.class_name + "_" +
    custom_suffix);
  map2object_msg.transform = tf2::toMsg(map2object);

  tf_broadcaster_->sendTransform(map2object_msg);
  return 0;
}

// Publish all the transforms in interests_ whose status is true
void PerceptionListener::publishTFinterest()
{
  for (auto & interest : interests_) {
    if (interest.second.status) {
      auto detections = get_by_type(interest.first);
      for (auto & detection : detections) {
        publishTF(detection);
      }
    }
  }
}

void PerceptionListener::publishSortedTFinterest(
  std::function<bool(
    const perception_system_interfaces::msg::Detection &,
    const perception_system_interfaces::msg::Detection &)> comp)
{
  for (auto & interest : interests_) {
    if (interest.second.status) {
      auto detections = get_by_type(interest.first);
      std::sort(detections.begin(), detections.end(), comp);
      int entity_counter = 0;
      for (auto & detection : detections) {
        publishTF(detection, std::to_string(entity_counter));
        entity_counter++;
      }
    }
  }
}

// void PerceptionListener::set_features(perception_system_interfaces::msg::Detection & person){}

// Create a vector from perceptions_ whose type match with the features of the person
std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_features(
  const perception_system_interfaces::msg::Detection & object,
  float confidence)
{
  std::vector<perception_system_interfaces::msg::Detection> result;
  std::vector<perception_system_interfaces::msg::Detection> detections =
    get_by_type(object.class_name);

  for (auto & detection : detections) {
    if (detection.unique_id == object.unique_id) {
      detection.confidence = 1.0;
      result.push_back(detection);
    }
  }

  std::sort(
      result.begin(), result.end(), [this](const auto & a, const auto & b) {
      return a.confidence < b.confidence;
      });

  return result;
}

}  // namespace perception_system
