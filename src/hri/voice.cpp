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
#include "std_msgs/msg/bool.hpp"

namespace hri
{

Voice::Voice(
  ID id,
  rclcpp::Node::SharedPtr node,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: FeatureTracker{id, "/humans/voices", "voice_", node, callback_group, tf_buffer, reference_frame}
{
}

Voice::~Voice()
{
  // executor_->cancel();
  // dedicated_listener_thread_->join();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Deleting voice " << id_);
}

void Voice::init()
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "New voice detected: " << ns_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  auto qos = rclcpp::SystemDefaultsQoS();

  is_speaking_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
    ns_ + "/is_speaking", qos,
    [&](std_msgs::msg::Bool::SharedPtr msg) {
      _is_speaking = msg->data;
      for (auto & cb : is_speaking_callbacks) {
        cb(msg->data);
      }
    });

  speech_subscriber_ = node_->create_subscription<hri_msgs::msg::LiveSpeech>(
    ns_ + "/speech", qos, bind(&Voice::_onSpeech, this, std::placeholders::_1));
}

void Voice::_onSpeech(const hri_msgs::msg::LiveSpeech::ConstSharedPtr msg)
{
  if (msg->incremental.size() > 0) {
    _incremental_speech = msg->incremental;
    for (auto cb : incremental_speech_callbacks) {
      cb(_incremental_speech);
    }
  }

  if (msg->final.size() > 0) {
    _speech = msg->final;
    for (auto cb : speech_callbacks) {
      cb(_speech);
    }
  }
}

}  // namespace hri
