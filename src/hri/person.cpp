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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "hri/person.hpp"
#include "hri/hri.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include <std_msgs/msg/float32.hpp>

namespace hri
{

Person::Person(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  HRIListener * const listener,
  tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/persons", "person_", node, callback_group, tf_buffer, reference_frame}
  , listener_(listener)
  , _alias("")
  , _engagement_status(nullptr)
  , _loc_confidence(0.)
{
}

Person::~Person()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting person " << id_);
}

void Person::init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New person detected: " << ns_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto qos = rclcpp::SystemDefaultsQoS();

  face_id_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    ns_ + "/face_id", qos,
    [&](const std_msgs::msg::String::SharedPtr msg) {face_id = msg->data;}, options);

  body_id_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    ns_ + "/body_id", qos,
    [&](const std_msgs::msg::String::SharedPtr msg) {body_id = msg->data;}, options);

  voice_id_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    ns_ + "/voice_id", qos,
    [&](const std_msgs::msg::String::SharedPtr msg) {voice_id = msg->data;}, options);

  anonymous_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
    ns_ + "/anonymous", qos,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {_anonymous = msg->data;}, options);

  alias_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    ns_ + "/alias", qos,
    [&](const std_msgs::msg::String::SharedPtr msg) {_alias = msg->data;}, options);

  engagement_subscriber_ = node_->create_subscription<hri_msgs::msg::EngagementLevel>(
    ns_ + "/engagement_status", qos,
    [&](const hri_msgs::msg::EngagementLevel::SharedPtr msg) {_engagement_status = msg;}, options);

  loc_confidence_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
    ns_ + "/location_confidence", qos,
    [&](const std_msgs::msg::Float32::SharedPtr msg) {_loc_confidence = msg->data;}, options);
}

FacePtr Person::face() const
{
  if (listener_->getFaces().count(face_id) != 0) {
    return listener_->getFaces()[face_id];
  } else {
    return FacePtr();
  }
}

BodyPtr Person::body() const
{
  if (listener_->getBodies().count(body_id) != 0) {
    return listener_->getBodies()[body_id];
  } else {
    return BodyPtr();
  }
}

VoicePtr Person::voice() const
{
  if (listener_->getVoices().count(voice_id) != 0) {
    return listener_->getVoices()[voice_id];
  } else {
    return VoicePtr();
  }
}

std::optional<EngagementLevel> Person::engagement_status() const
{
  if (!_engagement_status) {
    return std::optional<EngagementLevel>();
  }

  switch (_engagement_status->level) {
    case hri_msgs::msg::EngagementLevel::UNKNOWN:
      return std::optional<EngagementLevel>();
    case hri_msgs::msg::EngagementLevel::ENGAGING:
      return EngagementLevel::ENGAGING;
    case hri_msgs::msg::EngagementLevel::ENGAGED:
      return EngagementLevel::ENGAGED;
    case hri_msgs::msg::EngagementLevel::DISENGAGING:
      return EngagementLevel::DISENGAGING;
    case hri_msgs::msg::EngagementLevel::DISENGAGED:
      return EngagementLevel::DISENGAGED;
    default:
      // we should handle all the possible engagement values
      assert(false);
      return std::optional<EngagementLevel>();
  }
}

std::optional<geometry_msgs::msg::TransformStamped> Person::transform() const
{
  if (abs(_loc_confidence) < 1e-2) {
    return std::optional<geometry_msgs::msg::TransformStamped>();
  }

  return FeatureTracker::transform();
}

}  // namespace hri
