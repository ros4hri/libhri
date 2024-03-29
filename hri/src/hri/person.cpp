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

#include "hri/person.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/hri.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"

namespace hri
{

Person::Person(
  ID id,
  NodeInterfaces node_interfaces,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  std::weak_ptr<const HRIListener> listener,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{
    id, "/humans/persons", "person_", node_interfaces, callback_group, tf_buffer, reference_frame},
  listener_(listener)
{
  RCLCPP_DEBUG_STREAM(
    node_interfaces_.get_node_logging_interface()->get_logger(), "New person detected: " << kNs_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();
  auto latched_qos = rclcpp::SystemDefaultsQoS().transient_local().reliable();

  face_id_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/face_id", latched_qos, bind(&Person::onFaceId, this, std::placeholders::_1), options);

  body_id_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/body_id", latched_qos, bind(&Person::onBodyId, this, std::placeholders::_1), options);

  voice_id_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/voice_id", latched_qos,
    bind(&Person::onVoiceId, this, std::placeholders::_1), options);

  anonymous_subscriber_ = rclcpp::create_subscription<std_msgs::msg::Bool>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/anonymous", latched_qos,
    bind(&Person::onAnonymous, this, std::placeholders::_1), options);

  alias_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/alias", latched_qos, bind(&Person::onAlias, this, std::placeholders::_1), options);

  engagement_subscriber_ = rclcpp::create_subscription<hri_msgs::msg::EngagementLevel>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/engagement_status", default_qos,
    bind(&Person::onEngagementStatus, this, std::placeholders::_1), options);

  loc_confidence_subscriber_ = rclcpp::create_subscription<std_msgs::msg::Float32>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    kNs_ + "/location_confidence",
    default_qos, bind(&Person::onLocationConfidence, this, std::placeholders::_1), options);
}

Person::~Person()
{
  RCLCPP_DEBUG_STREAM(
    node_interfaces_.get_node_logging_interface()->get_logger(), "Deleting person " << kId_);
}

ConstFacePtr Person::face() const
{
  auto ret = ConstFacePtr();
  if (auto locked_listener = listener_.lock()) {
    if (face_id_ && locked_listener->getFaces().count(face_id_.value())) {
      ret = locked_listener->getFaces()[*face_id_];
    }
  } else {
    RCLCPP_WARN_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Person " << kId_ << " lost connection to the HRI listener!");
  }
  return ret;
}

ConstBodyPtr Person::body() const
{
  auto ret = ConstBodyPtr();
  if (auto locked_listener = listener_.lock()) {
    if (body_id_ && locked_listener->getBodies().count(body_id_.value())) {
      ret = locked_listener->getBodies()[*body_id_];
    }
  } else {
    RCLCPP_WARN_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Person " << kId_ << " lost connection to the HRI listener!");
  }
  return ret;
}

ConstVoicePtr Person::voice() const
{
  auto ret = ConstVoicePtr();
  if (auto locked_listener = listener_.lock()) {
    if (voice_id_ && locked_listener->getVoices().count(voice_id_.value())) {
      ret = locked_listener->getVoices()[*voice_id_];
    }
  } else {
    RCLCPP_WARN_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Person " << kId_ << " lost connection to the HRI listener!");
  }
  return ret;
}


void Person::onFaceId(std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    face_id_.reset();
  } else {
    face_id_ = msg->data;
  }
}

void Person::onBodyId(std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    body_id_.reset();
  } else {
    body_id_ = msg->data;
  }
}

void Person::onVoiceId(std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    voice_id_.reset();
  } else {
    voice_id_ = msg->data;
  }
}

void Person::onAlias(std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    alias_.reset();
  } else {
    alias_ = msg->data;
  }
}

void Person::onAnonymous(std_msgs::msg::Bool::ConstSharedPtr msg)
{
  anonymous_ = msg->data;
}

void Person::onEngagementStatus(hri_msgs::msg::EngagementLevel::ConstSharedPtr msg)
{
  if (msg->level == hri_msgs::msg::EngagementLevel::UNKNOWN) {
    engagement_status_.reset();
  } else {
    engagement_status_ = static_cast<EngagementLevel>(msg->level);
  }
}

void Person::onLocationConfidence(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  loc_confidence_ = msg->data;
}

std::optional<geometry_msgs::msg::TransformStamped> Person::transform() const
{
  if (std::abs(loc_confidence_.value_or(0.f)) < 1e-2f) {
    return std::optional<geometry_msgs::msg::TransformStamped>();
  } else {
    return FeatureTracker::transform();
  }
}

void Person::invalidate()
{
  face_id_subscriber_.reset();
  body_id_subscriber_.reset();
  voice_id_subscriber_.reset();
  anonymous_subscriber_.reset();
  alias_subscriber_.reset();
  engagement_subscriber_.reset();
  loc_confidence_subscriber_.reset();
  face_id_.reset();
  body_id_.reset();
  voice_id_.reset();
  alias_.reset();
  anonymous_.reset();
  engagement_status_.reset();
  loc_confidence_.reset();
  FeatureTracker::invalidate();
}

}  // namespace hri
