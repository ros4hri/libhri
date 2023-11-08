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

#include "hri/body.hpp"

#include <functional>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/skeleton2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{

Body::Body(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/bodies", "body_", node, callback_group, tf_buffer, reference_frame}
{
}

Body::~Body()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting body " << kId_);
}

void Body::init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New body detected: " << kNs_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();
  auto latched_qos = rclcpp::SystemDefaultsQoS().transient_local().reliable();

  roi_subscriber_ = node_->create_subscription<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    kNs_ + "/roi", latched_qos,
    bind(&Body::onRoI, this, std::placeholders::_1), options);

  cropped_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    kNs_ + "/cropped", latched_qos,
    bind(&Body::onCropped, this, std::placeholders::_1), options);

  skeleton_subscriber_ = node_->create_subscription<hri_msgs::msg::Skeleton2D>(
    kNs_ + "/skeleton2d", default_qos,
    bind(&Body::onSkeleton, this, std::placeholders::_1), options);
}

void Body::onRoI(const hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr msg)
{
  roi_ = *msg;
}

void Body::onCropped(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cropped_ = cv_bridge::toCvShare(msg)->image;
}

void Body::onSkeleton(const hri_msgs::msg::Skeleton2D::ConstSharedPtr msg)
{
  skeleton_ = msg->skeleton;
}

}  // namespace hri
