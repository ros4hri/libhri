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

#include "hri/voice.hpp"

#include <functional>
#include <string>

#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/live_speech.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{

Voice::Voice(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  const tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/voices", "voice_", node, callback_group, tf_buffer, reference_frame}
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New voice detected: " << kNs_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto default_qos = rclcpp::SystemDefaultsQoS();

  is_speaking_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
    kNs_ + "/is_speaking", default_qos,
    bind(&Voice::onIsSpeaking, this, std::placeholders::_1), options);

  speech_subscriber_ = node_->create_subscription<hri_msgs::msg::LiveSpeech>(
    kNs_ + "/speech", default_qos,
    bind(&Voice::_onSpeech, this, std::placeholders::_1), options);
}

Voice::~Voice()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting voice " << kId_);
}

void Voice::_onSpeech(hri_msgs::msg::LiveSpeech::ConstSharedPtr msg)
{
  if (msg->incremental.size() > 0) {
    incremental_speech_ = msg->incremental;
    for (auto cb : incremental_speech_callbacks) {
      cb(msg->incremental);
    }
  }

  if (msg->final.size() > 0) {
    speech_ = msg->final;
    for (auto cb : speech_callbacks) {
      cb(msg->final);
    }
  }
}

void Voice::onIsSpeaking(std_msgs::msg::Bool::ConstSharedPtr msg)
{
  is_speaking_ = msg->data;
  for (auto & cb : is_speaking_callbacks) {
    cb(msg->data);
  }
}

}  // namespace hri
