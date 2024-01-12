// Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

#include "hri/feature_tracker.hpp"

#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"

namespace hri
{

FeatureTracker::FeatureTracker(
  ID id,
  std::string feature_ns,
  std::string feature_tf_prefix,
  NodeInterfaces node_interfaces,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: kId_(id), kNs_(feature_ns + "/" + id), kFrame_(feature_tf_prefix + id),
  node_interfaces_(node_interfaces), callback_group_(callback_group), tf_buffer_(tf_buffer),
  reference_frame_(reference_frame)
{
  valid_ = true;
}

std::optional<geometry_msgs::msg::TransformStamped> FeatureTracker::transformFromReference(
  std::string frame_name) const
{
  try {
    auto transform = tf_buffer_.lookupTransform(reference_frame_, frame_name, tf2::TimePointZero);
    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "failed to transform " << frame_name << " to " << reference_frame_ << ". " << ex.what());
    return {};
  }
}

}  // namespace hri
