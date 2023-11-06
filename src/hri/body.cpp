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

#include <cv_bridge/cv_bridge.h>


namespace hri
{

Body::Body(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/bodies", node, callback_group}
  , _reference_frame(reference_frame)
  , tf_buffer_(tf_buffer)
{
}


Body::~Body()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting body " << id_);
}

void Body::init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New body detected: " << ns_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto qos = rclcpp::SystemDefaultsQoS();

  roi_subscriber_ = node_->create_subscription<RegionOfInterest>(
    ns_ + "/roi", qos, bind(&Body::onRoI, this, std::placeholders::_1), options);

  cropped_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    ns_ + "/cropped", qos, bind(&Body::onCropped, this, std::placeholders::_1), options);

  skeleton_subscriber_ = node_->create_subscription<hri_msgs::msg::Skeleton2D>(
    ns_ + "/skeleton2d", qos, bind(&Body::onSkeleton, this, std::placeholders::_1), options);
}

void Body::onRoI(const hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr roi)
{
  roi_ = *roi;
}

Body::RegionOfInterest Body::roi() const
{
  return roi_;
}

void Body::onCropped(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cropped_ = cv_bridge::toCvShare(msg)->image;
}

cv::Mat Body::cropped() const
{
  return cropped_;
}

void Body::onSkeleton(const hri_msgs::msg::Skeleton2D::ConstSharedPtr msg)
{
  skeleton_ = msg->skeleton;
}

Body::SkeletonPoints Body::skeleton() const
{
  return skeleton_;
}

std::optional<geometry_msgs::msg::TransformStamped> Body::transform() const
{
  try {
    auto transform = tf_buffer_.lookupTransform(
      _reference_frame, frame(),
      tf2::TimePointZero);

    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
      "failed to transform person frame " << frame()
                                          << " to " << _reference_frame <<
        ex.what());
    return std::optional<geometry_msgs::msg::TransformStamped>();
  }
}
}  // namespace hri
