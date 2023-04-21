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

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "geometry_msgs/TransformStamped.h"
#include "hri/person.h"

#include "hri/hri.h"
#include "hri_msgs/EngagementLevel.h"
#include "std_msgs/Float32.h"

using namespace std;
using namespace hri;

Person::Person(ID id, const HRIListener* listener, ros::NodeHandle& nh,
               tf2_ros::Buffer* tf_buffer_ptr, const std::string& reference_frame)
  : FeatureTracker{ id, nh }
  , listener_(listener)
  , _engagement_status(nullptr)
  , _alias("")
  , _loc_confidence(0.)
  , _tf_buffer_ptr(tf_buffer_ptr)
  , _reference_frame(reference_frame)
{
}


Person::~Person()
{
  ROS_DEBUG_STREAM("Deleting person " << id_);
}

void Person::init()
{
  ns_ = "/humans/persons/" + id_;
  ROS_DEBUG_STREAM("New person detected: " << ns_);

  face_id_subscriber_ = node_.subscribe<std_msgs::String>(
      ns_ + "/face_id", 1, [&](const std_msgs::StringConstPtr msg) { face_id = msg->data; });

  body_id_subscriber_ = node_.subscribe<std_msgs::String>(
      ns_ + "/body_id", 1, [&](const std_msgs::StringConstPtr msg) { body_id = msg->data; });

  voice_id_subscriber_ = node_.subscribe<std_msgs::String>(
      ns_ + "/voice_id", 1,
      [&](const std_msgs::StringConstPtr msg) { voice_id = msg->data; });

  anonymous_subscriber_ = node_.subscribe<std_msgs::Bool>(
      ns_ + "/anonymous", 1,
      [&](const std_msgs::BoolConstPtr msg) { _anonymous = msg->data; });

  alias_subscriber_ = node_.subscribe<std_msgs::String>(
      ns_ + "/alias", 1, [&](const std_msgs::StringConstPtr msg) { _alias = msg->data; });

  engagement_subscriber_ = node_.subscribe<hri_msgs::EngagementLevel>(
      ns_ + "/engagement_status", 1,
      [&](const hri_msgs::EngagementLevelConstPtr msg) { _engagement_status = msg; });

  loc_confidence_subscriber_ = node_.subscribe<std_msgs::Float32>(
      ns_ + "/location_confidence", 1,
      [&](const std_msgs::Float32ConstPtr msg) { _loc_confidence = msg->data; });
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
    case hri_msgs::EngagementLevel::UNKNOWN:
      return boost::optional<EngagementLevel>();
    case hri_msgs::EngagementLevel::ENGAGING:
      return EngagementLevel::ENGAGING;
    case hri_msgs::EngagementLevel::ENGAGED:
      return EngagementLevel::ENGAGED;
    case hri_msgs::EngagementLevel::DISENGAGING:
      return EngagementLevel::DISENGAGING;
    case hri_msgs::EngagementLevel::DISENGAGED:
      return EngagementLevel::DISENGAGED;
    default:
      // we should handle all the possible engagement values
      assert(false);
      return boost::optional<EngagementLevel>();
  }
}

boost::optional<geometry_msgs::TransformStamped> Person::transform() const
{
  if (_loc_confidence == 0)
  {
    return boost::optional<geometry_msgs::TransformStamped>();
  }

  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, frame(),
                                                     ros::Time(0), PERSON_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    ROS_WARN_STREAM("failed to transform person frame " << frame() << " to " << _reference_frame
                                                        << ". Are the frames published?");
    return boost::optional<geometry_msgs::TransformStamped>();
  }
}

