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

#include "hri/face.hpp"

#include <cv_bridge/cv_bridge.h>
#include <hri_msgs/msg/soft_biometrics.hpp>


namespace hri
{

Face::Face(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/faces", node, callback_group}
  , softbiometrics_(nullptr)
  , _reference_frame(reference_frame)
  , tf_buffer_(tf_buffer)
{
}

Face::~Face()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting face " << id_);
}

void Face::init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New face detected: " << ns_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto qos = rclcpp::SystemDefaultsQoS();

  roi_subscriber_ = node_->create_subscription<RegionOfInterest>(
    ns_ + "/roi", qos,
    bind(&Face::onRoI, this, std::placeholders::_1), options);

  cropped_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    ns_ + "/cropped", qos,
    bind(&Face::onCropped, this, std::placeholders::_1), options);


  aligned_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    ns_ + "/aligned", qos,
    bind(&Face::onAligned, this, std::placeholders::_1), options);

  landmarks_subscriber_ = node_->create_subscription<hri_msgs::msg::FacialLandmarks>(
    ns_ + "/landmarks", qos,
    bind(&Face::onLandmarks, this, std::placeholders::_1), options);

  softbiometrics_subscriber_ = node_->create_subscription<hri_msgs::msg::SoftBiometrics>(
    ns_ + "/softbiometrics", qos,
    bind(&Face::onSoftBiometrics, this, std::placeholders::_1), options);
}


void Face::onRoI(const hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr roi)
{
  roi_ = *roi;
}

Face::RegionOfInterest Face::roi() const
{
  return roi_;
}

void Face::onCropped(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // if using toCvShare, the image ends up shared with aligned_!
  cropped_ = cv_bridge::toCvCopy(msg)->image;
}

cv::Mat Face::cropped() const
{
  return cropped_;
}

void Face::onAligned(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // if using toCvShare, the image ends up shared with cropped_!
  aligned_ = cv_bridge::toCvCopy(msg)->image;
}

cv::Mat Face::aligned() const
{
  return aligned_;
}

void Face::onLandmarks(const hri_msgs::msg::FacialLandmarks::ConstSharedPtr msg)
{
  int i = 0;

  for (auto landmark : msg->landmarks) {
    facial_landmarks_[i].x = landmark.x;
    facial_landmarks_[i].y = landmark.y;
    facial_landmarks_[i].c = landmark.c;
    ++i;
  }
}

void Face::onSoftBiometrics(const hri_msgs::msg::SoftBiometrics::ConstSharedPtr biometrics)
{
  softbiometrics_ = biometrics;
}


std::optional<float> Face::age() const
{
  if (!softbiometrics_) {
    return std::optional<float>();
  }

  return softbiometrics_->age;
}

std::optional<Gender> Face::gender() const
{
  if (!softbiometrics_) {
    return std::optional<Gender>();
  }
  if (softbiometrics_->gender == 0) {  // UNDEFINED
    return std::optional<Gender>();
  }

  return static_cast<Gender>(softbiometrics_->gender);
}

std::optional<geometry_msgs::msg::TransformStamped> Face::transform() const
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

std::optional<geometry_msgs::msg::TransformStamped> Face::gazeTransform() const
{
  try {
    auto transform = tf_buffer_.lookupTransform(
      _reference_frame, gazeFrame(),
      tf2::TimePointZero);
    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
      "failed to transform person frame " << gazeFrame()
                                          << " to " << _reference_frame <<
        ex.what());
    return std::optional<geometry_msgs::msg::TransformStamped>();
  }
}

}  // namespace hri
