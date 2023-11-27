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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hri_msgs/msg/facial_action_units.hpp"
#include "hri_msgs/msg/facial_landmarks.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"

namespace hri
{

Face::Face(
  ID id,
  NodeInterfaces node_interfaces,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{
    id, "/humans/faces", "face_", node_interfaces, callback_group, tf_buffer, reference_frame},
  kGazeFrame_("gaze_" + kId_)
{
  RCLCPP_DEBUG_STREAM(
    node_interfaces_.get_node_logging_interface()->get_logger(), "New face detected: " << kNs_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();

  roi_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/roi", default_qos, bind(&Face::onRoI, this, std::placeholders::_1), options);

  cropped_subscriber_ = rclcpp::create_subscription<sensor_msgs::msg::Image>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/cropped", default_qos, bind(&Face::onCropped, this, std::placeholders::_1), options);

  aligned_subscriber_ = rclcpp::create_subscription<sensor_msgs::msg::Image>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/aligned", default_qos, bind(&Face::onAligned, this, std::placeholders::_1), options);

  landmarks_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::FacialLandmarks>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/landmarks", default_qos,
    bind(&Face::onLandmarks, this, std::placeholders::_1), options);

  softbiometrics_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::SoftBiometrics>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/softbiometrics", default_qos,
    bind(&Face::onSoftBiometrics, this, std::placeholders::_1), options);

  facial_action_units_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::FacialActionUnits>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/facs", default_qos, bind(&Face::onFacs, this, std::placeholders::_1), options);
}

Face::~Face()
{
  RCLCPP_DEBUG_STREAM(
    node_interfaces_.get_node_logging_interface()->get_logger(), "Deleting face " << kId_);
}

void Face::onRoI(const hri_msgs::msg::NormalizedRegionOfInterest2D::ConstSharedPtr msg)
{
  roi_.emplace(
    cv::Rect2f{cv::Point2f{msg->xmin, msg->ymin}, cv::Point2f{msg->xmax, msg->ymax}});
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

std::optional<geometry_msgs::msg::TransformStamped> Face::gazeTransform() const
{
  return transformFromReference(gazeFrame());
}

void Face::invalidate()
{
  roi_subscriber_.reset();
  cropped_subscriber_.reset();
  aligned_subscriber_.reset();
  landmarks_subscriber_.reset();
  softbiometrics_subscriber_.reset();
  facial_action_units_subscriber_.reset();
  roi_.reset();
  cropped_.reset();
  aligned_.reset();
  landmarks_.reset();
  age_.reset();
  gender_.reset();
  facial_action_units_.reset();
  FeatureTracker::invalidate();
}

}  // namespace hri
