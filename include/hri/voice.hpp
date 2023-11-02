// Copyright 2022 PAL Robotics
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the PAL Robotics S.L. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef HRI__VOICE_HPP_
#define HRI__VOICE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <optional>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "hri_msgs/msg/live_speech.hpp"

#include "std_msgs/msg/bool.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "feature_tracker.hpp"

namespace hri
{
// the tf prefix follows REP-155
static const char VOICE_TF_PREFIX[] = "voice_";
static const rclcpp::Duration VOICE_TF_TIMEOUT(rclcpp::Duration::from_seconds(0.01));

class Voice : public FeatureTracker
{
public:
  Voice(
    ID id,
    rclcpp::Node::SharedPtr node,
    tf2::BufferCore & tf_buffer,
    const std::string & reference_frame);

  virtual ~Voice();

  /** \brief the name of the tf frame that correspond to this body
   */
  std::string frame() const
  {
    return VOICE_TF_PREFIX + id_;
  }

  /** \brief Returns the estimated (stamped) 3D transform of the voice (if
   * available).
   */
  std::optional<geometry_msgs::msg::TransformStamped> transform() const;

  /** \brief returns speech is currently detected in this voice, ie, whether the person is
   * currently speaking.
   */
  bool is_speaking() const
  {
    return _is_speaking;
  }

  /** \brief returns the last recognised final sentence (or an empty string
   * if no speech was recognised yet).
   */
  std::string speech() const
  {
    return _speech;
  }

  /** \brief returns the last recognised incremental sentence (or an empty
   * string if no speech was recognised yet).
   */
  std::string incremental_speech() const
  {
    return _incremental_speech;
  }

  /** \brief Registers a callback function, to be invoked everytime speech is
   * detected (ie, the person is speaking).
   *
   * See also:
   * * Voice::onSpeech and Voice::onIncrementalSpeech to register a callback
   * ot get the actual recognised speech
   * * Voice::speech and Voice::incremental_speech for the last recognised speech
   */
  void onSpeaking(std::function<void(bool)> callback)
  {
    is_speaking_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime speech is
   * recognised from this voice. Only *final* sentences are returned, eg for instance at
   * the end of a sentece.
   *
   * See also: Voice::onIncrementalSpeech for incremental feedback
   */
  void onSpeech(std::function<void(const std::string &)> callback)
  {
    speech_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime speech is
   * recognised from this voice. The callback will be triggered every time the
   * speech recogniser returns a result, *even if it is not the final result*.
   */
  void onIncrementalSpeech(std::function<void(const std::string &)> callback)
  {
    incremental_speech_callbacks.push_back(callback);
  }

  void init() override;

private:
  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Node::SharedPtr node_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  std::string _reference_frame;
  tf2::BufferCore & tf_buffer_;

  bool _is_speaking;
  std::string _speech;
  std::string _incremental_speech;

  std::vector<std::function<void(bool)>> is_speaking_callbacks;
  std::vector<std::function<void(const std::string &)>> speech_callbacks;
  std::vector<std::function<void(const std::string &)>> incremental_speech_callbacks;

  void _onSpeech(hri_msgs::msg::LiveSpeech::ConstSharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_speaking_subscriber_{nullptr};
  rclcpp::Subscription<hri_msgs::msg::LiveSpeech>::SharedPtr speech_subscriber_{nullptr};
};

typedef std::shared_ptr<Voice> VoicePtr;
typedef std::shared_ptr<const Voice> VoiceConstPtr;

}  // namespace hri
#endif  // HRI__VOICE_HPP_
