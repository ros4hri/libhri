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

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/hri.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{

Person::Person(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  std::weak_ptr<const HRIListener> listener,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{
    id, "/humans/persons", "person_", node, callback_group, tf_buffer, reference_frame},
  listener_(listener)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New person detected: " << kNs_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();
  auto latched_qos = rclcpp::SystemDefaultsQoS().transient_local().reliable();

  face_id_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    kNs_ + "/face_id", default_qos,
    bind(&Person::onFaceId, this, std::placeholders::_1), options);

  body_id_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    kNs_ + "/body_id", default_qos,
    bind(&Person::onBodyId, this, std::placeholders::_1), options);

  voice_id_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    kNs_ + "/voice_id", default_qos,
    bind(&Person::onVoiceId, this, std::placeholders::_1), options);

  anonymous_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
    kNs_ + "/anonymous", latched_qos,
    bind(&Person::onAnonymous, this, std::placeholders::_1), options);

  alias_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    kNs_ + "/alias", default_qos,
    bind(&Person::onAlias, this, std::placeholders::_1), options);

  engagement_subscriber_ = node_->create_subscription<hri_msgs::msg::EngagementLevel>(
    kNs_ + "/engagement_status", default_qos,
    bind(&Person::onEngagementStatus, this, std::placeholders::_1), options);

  loc_confidence_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
    kNs_ + "/location_confidence", default_qos,
    bind(&Person::onLocationConfidence, this, std::placeholders::_1), options);
}

Person::~Person()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting person " << kId_);
}

void assignStringOptional(std::optional<std::string> & op, std::string val)
{
  if (val.empty()) {
    op.reset();
  } else {
    op = val;
  }
}

FacePtr Person::face() const
{
  auto ret = FacePtr();
  if (auto locked_listener = listener_.lock()) {
    if (face_id_ && locked_listener->getFaces().count(face_id_.value())) {
      ret = locked_listener->getFaces()[*face_id_];
    }
  } else {
    RCLCPP_WARN_STREAM(
      node_->get_logger(), "Person " << id() << " lost connection to the HRI listener!");
  }
  return ret;
}

BodyPtr Person::body() const
{
  auto ret = BodyPtr();
  if (auto locked_listener = listener_.lock()) {
    if (body_id_ && locked_listener->getBodies().count(body_id_.value())) {
      ret = locked_listener->getBodies()[*body_id_];
    }
  } else {
    RCLCPP_WARN_STREAM(
      node_->get_logger(), "Person " << id() << " lost connection to the HRI listener!");
  }
  return ret;
}

VoicePtr Person::voice() const
{
  auto ret = VoicePtr();
  if (auto locked_listener = listener_.lock()) {
    if (voice_id_ && locked_listener->getVoices().count(voice_id_.value())) {
      ret = locked_listener->getVoices()[*voice_id_];
    }
  } else {
    RCLCPP_WARN_STREAM(
      node_->get_logger(), "Person " << id() << " lost connection to the HRI listener!");
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

std::optional<Transform> Person::transform() const
{
  if (std::abs(loc_confidence_.value_or(0.f)) < 1e-2f) {
    return std::optional<Transform>();
  } else {
    return FeatureTracker::transform();
  }
}

}  // namespace hri
