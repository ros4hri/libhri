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

#ifndef HRI__HRI_HPP_
#define HRI__HRI_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/feature_tracker.hpp"
#include "hri/person.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace hri
{

/** \brief Main entry point to libhri. This is most likely what you want to use.
 *
 * See src/node_show_faces.cpp for a minimal usage example
 */
class HRIListener : public std::enable_shared_from_this<HRIListener>
{
public:
  /** \brief Factory function for the libhri main class.
   *
   * The class can subscribe to topics and print logs, using the node argument.
   */
  [[nodiscard]] static std::shared_ptr<HRIListener> create(rclcpp::Node::SharedPtr node)
  {
    return std::shared_ptr<HRIListener>(new HRIListener(node));
  }

  ~HRIListener();

  /** \brief Returns the list of currently detected faces, mapped to their IDs
   *
   * Faces are returned as constant std::shared_ptr as they may disappear at any point.
   */
  std::map<ID, FacePtr> getFaces() const;

  /** \brief Registers a callback function, to be invoked everytime a new face
   * is detected.
   */
  void onFace(std::function<void(FacePtr)> callback)
  {
    face_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked face is lost (eg, not detected anymore)
   */
  void onFaceLost(std::function<void(ID)> callback)
  {
    face_lost_callbacks_.push_back(callback);
  }


  /** \brief Returns the list of currently detected bodies, mapped to their IDs
   *
   * Bodies are returned as constant std::shared_ptr as they may disappear at any point.
   */
  std::map<ID, BodyPtr> getBodies() const;

  /** \brief Registers a callback function, to be invoked everytime a new body
   * is detected.
   */
  void onBody(std::function<void(BodyPtr)> callback)
  {
    body_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked body is lost (eg, not detected anymore)
   */
  void onBodyLost(std::function<void(ID)> callback)
  {
    body_lost_callbacks_.push_back(callback);
  }


  /** \brief Returns the list of currently detected voices, mapped to their IDs
   *
   * Voices are returned as constant std::shared_ptr as they may disappear at any point.
   */
  std::map<ID, VoicePtr> getVoices() const;

  /** \brief Registers a callback function, to be invoked everytime a new voice
   * is detected.
   */
  void onVoice(std::function<void(VoicePtr)> callback)
  {
    voice_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked voice is lost (eg, not detected anymore)
   */
  void onVoiceLost(std::function<void(ID)> callback)
  {
    voice_lost_callbacks_.push_back(callback);
  }


  /** \brief Returns the list of all known persons, whether or not they are
   * currently actively detected (eg, seen). The persons are mapped to their
   * IDs.
   *
   * Persons are returned as constant std::shared_ptr: while person do *not*
   * disappear in general, *anonymous* persons (created because, eg, a face has
   * been detected, and we can infer a yet-to-be-recognised person does exist)
   * can disappear.
   */
  std::map<ID, PersonPtr> getPersons() const;

  /** \brief Registers a callback function, to be invoked everytime a new person
   * is detected.
   */
  void onPerson(std::function<void(PersonPtr)> callback)
  {
    person_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a person
   * is lost. This can *only* happen for anonymous persons. Identified persons
   * will never be removed from the list of all known persons.
   */
  void onPersonLost(std::function<void(ID)> callback)
  {
    person_lost_callbacks_.push_back(callback);
  }

  /** \brief Returns the list of currently detected persons, mapped to their IDs
   *
   * Persons are returned as constant std::shared_ptr: while person do *not* disappear in
   * general, *anonymous* persons (created because, eg, a face has been detected, and we
   * can infer a yet-to-be-recognised person does exist) can disappear.
   */
  std::map<ID, PersonPtr> getTrackedPersons() const;

  /** \brief Registers a callback function, to be invoked everytime a new person
   * is detected and actively tracked (eg, currently seen).
   */
  void onTrackedPerson(std::function<void(PersonPtr)> callback)
  {
    person_tracked_callbacks_.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a previously tracked
   * person is lost.
   */
  void onTrackedPersonLost(std::function<void(ID)> callback)
  {
    person_tracked_lost_callbacks_.push_back(callback);
  }


  /** \brief Sets the reference frame from which the TF transformations of the persons will
   * be returned (via `Person::transform()`).
   *
   * By default, `base_link`.
   */
  void setReferenceFrame(const std::string & frame)
  {
    reference_frame_ = frame;
  }

private:
  explicit HRIListener(rclcpp::Node::SharedPtr node);
  void onTrackedFeature(FeatureType feature, hri_msgs::msg::IdsList::ConstSharedPtr tracked);

  std::map<FeatureType,
    rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr> feature_subscribers_;
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  std::map<ID, FacePtr> faces_;
  std::vector<std::function<void(FacePtr)>> face_callbacks_;
  std::vector<std::function<void(ID)>> face_lost_callbacks_;

  std::map<ID, BodyPtr> bodies_;
  std::vector<std::function<void(BodyPtr)>> body_callbacks_;
  std::vector<std::function<void(ID)>> body_lost_callbacks_;

  std::map<ID, VoicePtr> voices_;
  std::vector<std::function<void(VoicePtr)>> voice_callbacks_;
  std::vector<std::function<void(ID)>> voice_lost_callbacks_;

  std::map<ID, PersonPtr> persons_;
  std::vector<std::function<void(PersonPtr)>> person_callbacks_;
  std::vector<std::function<void(ID)>> person_lost_callbacks_;
  std::map<ID, PersonPtr> tracked_persons_;
  std::vector<std::function<void(PersonPtr)>> person_tracked_callbacks_;
  std::vector<std::function<void(ID)>> person_tracked_lost_callbacks_;

  std::string reference_frame_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace hri

#endif  // HRI__HRI_HPP_
