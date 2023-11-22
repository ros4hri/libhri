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

#ifndef HRI__VOICE_HPP_
#define HRI__VOICE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hri/feature_tracker.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/live_speech.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

namespace hri
{
class Voice : public FeatureTracker
// TODO(LJU): possibly subscribe also to the /audio and the /features sub-topics
{
public:
  Voice(
    ID id,
    NodeInterfaces & node_interfaces,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    const tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Voice();

  /** \brief Returns speech is currently detected in this voice, ie, whether the person is
   * currently speaking.
   */
  std::optional<bool> isSpeaking() const {return is_speaking_;}

  /** \brief Returns the last recognised final sentence (or an empty string
   * if no speech was recognised yet).
   */
  std::optional<std::string> speech() const {return speech_;}

  /** \brief Returns the last recognised incremental sentence (or an empty
   * string if no speech was recognised yet).
   */
  std::optional<std::string> incrementalSpeech() const {return incremental_speech_;}

  /** \brief Registers a callback function, to be invoked everytime speech is
   * detected (ie, the person is speaking).
   *
   * See also:
   * * Voice::onSpeech and Voice::onIncrementalSpeech to register a callback
   * ot get the actual recognised speech
   * * Voice::speech and Voice::incrementalSpeech for the last recognised speech
   */
  void onSpeaking(std::function<void(bool)> callback)
  {
    is_speaking_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime speech is
   * recognised from this voice. Only *final* sentences are returned, eg for instance at
   * the end of a sentece.
   *
   * See also: Voice::onIncrementalSpeech for incremental feedback
   */
  void onSpeech(std::function<void(const std::string &)> callback)
  {
    speech_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime speech is
   * recognised from this voice. The callback will be triggered every time the
   * speech recogniser returns a result, *even if it is not the final result*.
   */
  void onIncrementalSpeech(std::function<void(const std::string &)> callback)
  {
    incremental_speech_callbacks_.push_back(callback);
  }

private:
  void onSpeech_(hri_msgs::msg::LiveSpeech::ConstSharedPtr msg);
  void onIsSpeaking(std_msgs::msg::Bool::ConstSharedPtr msg);

  std::optional<bool> is_speaking_;
  std::optional<std::string> speech_;
  std::optional<std::string> incremental_speech_;

  std::vector<std::function<void(bool)>> is_speaking_callbacks_;
  std::vector<std::function<void(const std::string &)>> speech_callbacks_;
  std::vector<std::function<void(const std::string &)>> incremental_speech_callbacks_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_speaking_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::LiveSpeech>::SharedPtr speech_subscriber_;
};

typedef std::shared_ptr<Voice> VoicePtr;

}  // namespace hri

#endif  // HRI__VOICE_HPP_
