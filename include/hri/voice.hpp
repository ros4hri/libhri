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
#include <boost/optional.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "FeatureTracker.hpp"

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
  boost::optional<geometry_msgs::msg::TransformStamped> transform() const;

  void init() override;

private:
  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Node::SharedPtr node_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  std::string _reference_frame;
  tf2::BufferCore & tf_buffer_;
};

typedef std::shared_ptr<Voice> VoicePtr;
typedef std::shared_ptr<const Voice> VoiceConstPtr;
typedef std::weak_ptr<Voice> VoiceWeakPtr;
typedef std::weak_ptr<const Voice> VoiceWeakConstPtr;

}  // namespace hri
#endif  // HRI__VOICE_HPP_
