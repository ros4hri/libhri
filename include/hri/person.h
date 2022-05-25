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

#ifndef HRI_PERSON_H
#define HRI_PERSON_H

#include <geometry_msgs/TransformStamped.h>
#include <memory>

#include "base.h"
#include "face.h"
#include "body.h"
#include "voice.h"

#include <hri_msgs/EngagementLevel.h>
#include <std_msgs/Float32.h>


namespace hri
{
enum EngagementLevel
{
  // disengaged: the human has not looked in the direction of the robot
  DISENGAGED = 1,
  // engaging: the human has started to look in the direction of the robot
  ENGAGING = 2,
  // engaged: the human is fully engaged with the robot
  ENGAGED = 3,
  // disengaging: the human has started to look away from the robot
  DISENGAGING = 4
};


class HRIListener;


class Person : public FeatureTracker
{
public:
  Person(ID id, const HRIListener* listener, const ros::NodeHandle& nh)
    : FeatureTracker{ id, nh }
    , listener_(listener)
    , _anonymous(false)
    , _engagement_status(nullptr)
    , _alias("")
    , _loc_confidence(-1)
  {
  }

  virtual ~Person();

  /* returns a (weak, constant) pointer to the face of this person, or
   * a nullptr if this person is currently not associated to any detected face.
   */
  FaceWeakConstPtr face() const;

  /* returns a (weak, constant) pointer to the body of this person, or
   * a nullptr if this person is currently not associated to any detected body.
   */
  BodyWeakConstPtr body() const;

  /* returns a (weak, constant) pointer to the voice of this person, or
   * a nullptr if this person is currently not associated to any detected voice.
   */
  VoiceWeakConstPtr voice() const;


  bool anonymous() const
  {
    return _anonymous;
  }

  ID alias() const
  {
    return _alias;
  }

  boost::optional<EngagementLevel> engagement_status() const;

  boost::optional<float> location_confidence() const;

  geometry_msgs::TransformStamped getTransform() const;

  void init() override;

protected:
  // we use a raw pointer here. `this` is owned by the pointed HRIListener, so
  // `this` would normally be destroyed before HRIListener (in reality, a
  // pointer to `this` *might* outlive `HRIListener` -- make sure HRIListener
  // is destroyed after all pointers to this person are released.
  const HRIListener* listener_;

  ID face_id;
  ID body_id;
  ID voice_id;

  // if non-empty, this person 'does not exist' and is instead an alias to
  // another person.  hri::getPersons and hri::getTrackedPersons will returns
  // pointers to the aliased person.
  ID _alias;

  bool _anonymous;

  hri_msgs::EngagementLevelConstPtr _engagement_status;

  float _loc_confidence;

  ros::Subscriber face_id_subscriber_;
  ros::Subscriber body_id_subscriber_;
  ros::Subscriber voice_id_subscriber_;
  ros::Subscriber anonymous_subscriber_;
  ros::Subscriber alias_subscriber_;
  ros::Subscriber engagement_subscriber_;
  ros::Subscriber loc_confidence_subscriber_;
};

typedef std::shared_ptr<Person> PersonPtr;
typedef std::shared_ptr<const Person> PersonConstPtr;
typedef std::weak_ptr<Person> PersonWeakPtr;
typedef std::weak_ptr<const Person> PersonWeakConstPtr;


}  // namespace hri

#endif
