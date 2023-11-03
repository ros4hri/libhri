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
#include <vector>
#include <string>

#include "hri_msgs/msg/ids_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "body.hpp"
#include "face.hpp"
#include "feature_tracker.hpp"
#include "person.hpp"
#include "voice.hpp"


namespace hri
{
/** \brief Main entry point to libhri. This is most likely what you want to use.
 * Use example:
 *
 * ```cpp
 * ros::init(argc, argv, "test_libhri");
 * ros::NodeHandle nh;
 *
 * HRIListener hri_listener;
 *
 * while (ros::ok()) {
 *
 *   auto faces = hri_listener.getFaces();
 *
 *   for (auto const& face : faces)
 *   {
 *     cout << "Face " << face.first << " seen!";
 *   }
 * }
 * ```
 */
class HRIListener
{
public:
  HRIListener();

  ~HRIListener();

  /** \brief Returns the list of currently detected faces, mapped to their IDs
   *
   * Faces are returned as constant std::shared_ptr as they may disappear at any point.
   */
  // std::map<ID, FacePtr> getFaces() const;
  std::map<ID, FacePtr> getFaces() const;

  /** \brief Registers a callback function, to be invoked everytime a new face
   * is detected.
   */
  void onFace(std::function<void(FacePtr)> callback)
  {
    face_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked face is lost (eg, not detected anymore)
   */
  void onFaceLost(std::function<void(ID)> callback)
  {
    face_lost_callbacks.push_back(callback);
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
    body_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked body is lost (eg, not detected anymore)
   */
  void onBodyLost(std::function<void(ID)> callback)
  {
    body_lost_callbacks.push_back(callback);
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
    voice_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a
   * previously tracked voice is lost (eg, not detected anymore)
   */
  void onVoiceLost(std::function<void(ID)> callback)
  {
    voice_lost_callbacks.push_back(callback);
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
    person_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a person
   * is lost. This can *only* happen for anonymous persons. Identified persons
   * will never be removed from the list of all known persons.
   */
  void onPersonLost(std::function<void(ID)> callback)
  {
    person_lost_callbacks.push_back(callback);
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
    person_tracked_callbacks.push_back(callback);
  }

  /** \brief Registers a callback function, to be invoked everytime a previously tracked
   * person is lost.
   */
  void onTrackedPersonLost(std::function<void(ID)> callback)
  {
    person_tracked_lost_callbacks.push_back(callback);
  }


  /** sets the reference frame from which the TF transformations of the persons will
   * be returned (via `Person::transform()`).
   *
   * By default, `base_link`.
   */
  void setReferenceFrame(const std::string & frame)
  {
    _reference_frame = frame;
  }

private:
  void init();

  void onTrackedFeature(FeatureType feature, hri_msgs::msg::IdsList::SharedPtr tracked);

  std::map<FeatureType,
    rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr> feature_subscribers_;
  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Node::SharedPtr node_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  std::map<ID, FacePtr> faces;
  std::vector<std::function<void(FacePtr)>> face_callbacks;
  std::vector<std::function<void(ID)>> face_lost_callbacks;

  std::map<ID, BodyPtr> bodies;
  std::vector<std::function<void(BodyPtr)>> body_callbacks;
  std::vector<std::function<void(ID)>> body_lost_callbacks;

  std::map<ID, VoicePtr> voices;
  std::vector<std::function<void(VoicePtr)>> voice_callbacks;
  std::vector<std::function<void(ID)>> voice_lost_callbacks;

  std::map<ID, PersonPtr> persons;
  std::vector<std::function<void(PersonPtr)>> person_callbacks;
  std::vector<std::function<void(ID)>> person_lost_callbacks;
  std::map<ID, PersonPtr> tracked_persons;
  std::vector<std::function<void(PersonPtr)>> person_tracked_callbacks;
  std::vector<std::function<void(ID)>> person_tracked_lost_callbacks;

  std::string _reference_frame;
  tf2::BufferCore _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
};

}  // namespace hri


#endif  // HRI__HRI_HPP_
