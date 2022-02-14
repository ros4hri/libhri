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

#include "hri/person.h"

#include "hri/hri.h"

using namespace std;
using namespace hri;

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

