// Copyright 2022 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <geometry_msgs/msg/transform_stamped.hpp>
#include "hri/person.hpp"

#include "hri/hri.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include <std_msgs/msg/float32.hpp>

using namespace std;
using namespace hri;

Person::Person(ID id, const HRIListener* listener, tf2_ros::Buffer* tf_buffer_ptr,
                const std::string& reference_frame)
  : FeatureTracker{ id }
  , listener_(listener)
  , _anonymous(false)
  , _engagement_status(nullptr)
  , _alias("")
  , _loc_confidence(0.)
  , _tf_buffer_ptr(tf_buffer_ptr)
  , _reference_frame(reference_frame)
{
}


Person::~Person()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Deleting person " << id_);
}

void Person::init()
{
  ns_ = "/humans/persons/" + id_;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "New person detected: " << ns_);

  auto face_id_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      ns_ + "/face_id", 1, [&](const std_msgs::msg::String::SharedPtr msg) { face_id = msg->data; });

  auto body_id_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      ns_ + "/body_id", 1, [&](const std_msgs::msg::String::SharedPtr msg) { body_id = msg->data; });

  auto voice_id_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      ns_ + "/voice_id", 1,
      [&](const std_msgs::msg::String::SharedPtr msg) { voice_id = msg->data; });

  auto anonymous_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      ns_ + "/anonymous", 1,
      [&](const std_msgs::msg::Bool::SharedPtr msg) { _anonymous = msg->data; });

  auto alias_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      ns_ + "/alias", 1, [&](const std_msgs::msg::String::SharedPtr msg) { _alias = msg->data; });

  auto engagement_subscriber_ = this->create_subscription<hri_msgs::msg::EngagementLevel>(
      ns_ + "/engagement_status", 1,
      [&](const hri_msgs::msg::EngagementLevel::SharedPtr msg) { _engagement_status = msg; });

  auto loc_confidence_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      ns_ + "/location_confidence", 1,
      [&](const std_msgs::msg::Float32::SharedPtr msg) { _loc_confidence = msg->data; });
}

FaceWeakConstPtr Person::face() const
{
  if (listener_->getFaces().count(face_id) != 0)
    return listener_->getFaces()[face_id];
  else
    return FaceWeakConstPtr();
}

BodyWeakConstPtr Person::body() const
{
  if (listener_->getBodies().count(body_id) != 0)
    return listener_->getBodies()[body_id];
  else
    return BodyWeakConstPtr();
}

VoiceWeakConstPtr Person::voice() const
{
  if (listener_->getVoices().count(voice_id) != 0)
    return listener_->getVoices()[voice_id];
  else
    return VoiceWeakConstPtr();
}

boost::optional<EngagementLevel> Person::engagement_status() const
{
  if (!_engagement_status)
  {
    return boost::optional<EngagementLevel>();
  }

  switch (_engagement_status->level)
  {
    case hri_msgs::msg::EngagementLevel::UNKNOWN:
      return boost::optional<EngagementLevel>();
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
      return boost::optional<EngagementLevel>();
  }
}

boost::optional<geometry_msgs::msg::TransformStamped> Person::transform() const
{
  if (_loc_confidence == 0)
  {
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }

  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, frame(),
                                                     rclcpp::Time(0), PERSON_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "failed to transform person frame " << frame()
                                << " to " << _reference_frame << ". Are the frames published?");
    return boost::optional<geometry_msgs::msg::TransformStamped>();
  }
}

