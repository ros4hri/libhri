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

#include <functional>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/facial_action_units.hpp"
#include "hri_msgs/msg/facial_landmarks.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace hri
{

Face::Face(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/faces", "face_", node, callback_group, tf_buffer, reference_frame},
  kGazeFrame_("gaze_" + kId_)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New face detected: " << kNs_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();

  roi_subscriber_ = node_->create_subscription<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    kNs_ + "/roi", default_qos,
    bind(&Face::onRoI, this, std::placeholders::_1), options);

  cropped_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    kNs_ + "/cropped", default_qos,
    bind(&Face::onCropped, this, std::placeholders::_1), options);

  aligned_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
    kNs_ + "/aligned", default_qos,
    bind(&Face::onAligned, this, std::placeholders::_1), options);

  landmarks_subscriber_ = node_->create_subscription<hri_msgs::msg::FacialLandmarks>(
    kNs_ + "/landmarks", default_qos,
    bind(&Face::onLandmarks, this, std::placeholders::_1), options);

  softbiometrics_subscriber_ = node_->create_subscription<hri_msgs::msg::SoftBiometrics>(
    kNs_ + "/softbiometrics", default_qos,
    bind(&Face::onSoftBiometrics, this, std::placeholders::_1), options);

  facial_action_units_subscriber_ = node_->create_subscription<hri_msgs::msg::FacialActionUnits>(
    kNs_ + "/facs", default_qos,
    bind(&Face::onFacs, this, std::placeholders::_1), options);
}

Face::~Face()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting face " << kId_);
}

void Face::onRoI(const hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr msg)
{
  roi_.emplace(
    RegionOfInterest{cv::Point2f{msg->xmin, msg->ymin}, cv::Point2f{msg->xmax, msg->ymax}});
}

void Face::onCropped(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // if using toCvShare, the image ends up shared with aligned_!
  cropped_ = cv_bridge::toCvCopy(msg)->image;
}

void Face::onAligned(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // if using toCvShare, the image ends up shared with cropped_!
  aligned_ = cv_bridge::toCvCopy(msg)->image;
}

void Face::onLandmarks(const hri_msgs::msg::FacialLandmarks::ConstSharedPtr msg)
{
  if (!landmarks_) {
    landmarks_ = FacialLandmarks();
  }
  for (size_t i = 0; i < msg->landmarks.size(); ++i) {
    auto & lm = msg->landmarks[i];
    (*landmarks_)[static_cast<FacialLandmark>(i)] = PointOfInterest{lm.x, lm.y, lm.c};
  }
}

void Face::onSoftBiometrics(const hri_msgs::msg::SoftBiometrics::ConstSharedPtr msg)
{
  age_ = msg->age;
  if (msg->gender == hri_msgs::msg::SoftBiometrics::UNDEFINED) {
    gender_.reset();
  } else {
    gender_ = static_cast<Gender>(msg->gender);
  }
}

void Face::onFacs(hri_msgs::msg::FacialActionUnits::ConstSharedPtr msg)
{
  if (!facial_action_units_) {
    facial_action_units_ = FacialActionUnits();
  }
  for (size_t i = 0; i < msg->intensity.size(); ++i) {
    (*facial_action_units_)[static_cast<FacialActionUnit>(i)] =
      IntensityConfidence{msg->intensity[i], msg->confidence[i]};
  }
}

std::optional<Transform> Face::gazeTransform() const
{
  return transform(gazeFrame());
}

}  // namespace hri
