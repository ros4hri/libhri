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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <functional>
#include <memory>

#include "FeatureTracker.hpp"
#include "face.hpp"
#include "body.hpp"
#include "voice.hpp"

#include "hri_msgs/msg/engagement_level.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <tf2_ros/buffer.h>
#include "message_filters/subscriber.h"

namespace hri
{
const static std::string PERSON_TF_PREFIX("person_");
const static rclcpp::Duration PERSON_TF_TIMEOUT(rclcpp::Duration::from_seconds(0.01));

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
  Person(ID id, const HRIListener* listener, tf2_ros::Buffer* tf_buffer_ptr,
          const std::string& reference_frame);

  virtual ~Person();

  std::string frame() const
  {
    return PERSON_TF_PREFIX + id_;
  }

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

  float location_confidence() const
  {
    return _loc_confidence;
  }

  boost::optional<geometry_msgs::msg::TransformStamped> transform() const;

  void init() override;
  ID face_id;
  ID body_id;
  ID voice_id;


protected:
  // we use a raw pointer here. `this` is owned by the pointed HRIListener, so
  // `this` would normally be destroyed before HRIListener (in reality, a
  // pointer to `this` *might* outlive `HRIListener` -- make sure HRIListener
  // is destroyed after all pointers to this person are released.
  const HRIListener* listener_;

  void tfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr& transform_ptr)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "got tf transform!");
  }

  // if non-empty, this person 'does not exist' and is instead an alias to
  // another person.  hri::getPersons and hri::getTrackedPersons will returns
  // pointers to the aliased person.
  ID _alias;

  bool _anonymous;

  hri_msgs::msg::EngagementLevel::ConstPtr _engagement_status;

  float _loc_confidence;

  std::string _reference_frame;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr face_id_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr body_id_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_id_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr anonymous_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alias_subscriber_;
  rclcpp::Subscription<hri_msgs::msg::EngagementLevel>::SharedPtr engagement_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr loc_confidence_subscriber_;

  tf2_ros::Buffer* _tf_buffer_ptr;
};

typedef std::shared_ptr<Person> PersonPtr;
typedef std::shared_ptr<const Person> PersonConstPtr;
typedef std::weak_ptr<Person> PersonWeakPtr;
typedef std::weak_ptr<const Person> PersonWeakConstPtr;

}  // namespace hri

#endif
