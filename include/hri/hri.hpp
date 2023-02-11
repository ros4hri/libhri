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


#ifndef HRI__HRI_HPP_
#define HRI__HRI_HPP_

#include "hri_msgs/msg/ids_list.hpp"

#include <functional>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "FeatureTracker.hpp"
#include "person.hpp"
#include "face.hpp"
#include "body.hpp"
#include "voice.hpp"
#include "tf2_ros/buffer.h"

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
   * Faces are returned as constant std::weak_ptr as they may disappear at any point.
   */
  // std::map<ID, FacePtr> getFaces() const;
  std::map<ID, FaceWeakConstPtr> getFaces() const;  

  /** \brief Registers a callback function, to be invoked everytime a new face
   * is detected.
   */
  void onFace(std::function<void(FaceWeakConstPtr)> callback)
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
   * Bodies are returned as constant std::weak_ptr as they may disappear at any point.
   */
  std::map<ID, BodyWeakConstPtr> getBodies() const;

  /** \brief Registers a callback function, to be invoked everytime a new body
   * is detected.
   */
  void onBody(std::function<void(BodyWeakConstPtr)> callback)
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
   * Voices are returned as constant std::weak_ptr as they may disappear at any point.
   */
  std::map<ID, VoiceWeakConstPtr> getVoices() const;

  /** \brief Registers a callback function, to be invoked everytime a new voice
   * is detected.
   */
  void onVoice(std::function<void(VoiceWeakConstPtr)> callback)
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
   * Persons are returned as constant std::weak_ptr: while person do *not*
   * disappear in general, *anonymous* persons (created because, eg, a face has
   * been detected, and we can infer a yet-to-be-recognised person does exist)
   * can disappear.
   */
  std::map<ID, PersonWeakConstPtr> getPersons() const;

  /** \brief Registers a callback function, to be invoked everytime a new person
   * is detected.
   */
  void onPerson(std::function<void(PersonWeakConstPtr)> callback)
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
   * Persons are returned as constant std::weak_ptr: while person do *not* disappear in
   * general, *anonymous* persons (created because, eg, a face has been detected, and we
   * can infer a yet-to-be-recognised person does exist) can disappear.
   */
  std::map<ID, PersonWeakConstPtr> getTrackedPersons() const;


  /** \brief Registers a callback function, to be invoked everytime a new person
   * is detected and actively tracked (eg, currently seen).
   */
  void onTrackedPerson(std::function<void(PersonWeakConstPtr)> callback)
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
  void setReferenceFrame(const std::string& frame)
  {
    _reference_frame = frame;
  }

private:
  void init();

  void onTrackedFeature(FeatureType feature,hri_msgs::msg::IdsList::SharedPtr tracked);
  
  std::map<FeatureType, rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr> feature_subscribers_;
  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Node::SharedPtr node_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr}; 
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  std::map<ID, FaceConstPtr> faces;
  std::vector<std::function<void(FaceWeakConstPtr)>> face_callbacks;
  std::vector<std::function<void(ID)>> face_lost_callbacks;

  std::map<ID, BodyConstPtr> bodies;
  std::vector<std::function<void(BodyWeakConstPtr)>> body_callbacks;
  std::vector<std::function<void(ID)>> body_lost_callbacks;

  std::map<ID, VoiceConstPtr> voices;
  std::vector<std::function<void(VoiceWeakConstPtr)>> voice_callbacks;
  std::vector<std::function<void(ID)>> voice_lost_callbacks;

  std::map<ID, PersonConstPtr> persons;
  std::vector<std::function<void(PersonConstPtr)>> person_callbacks;
  std::vector<std::function<void(ID)>> person_lost_callbacks;
  std::map<ID, PersonConstPtr> tracked_persons;
  std::vector<std::function<void(PersonConstPtr)>> person_tracked_callbacks;
  std::vector<std::function<void(ID)>> person_tracked_lost_callbacks;

  std::string _reference_frame;
  tf2::BufferCore _tf_buffer;
  tf2_ros::TransformListener _tf_listener;
};

}  // namespace hri



#endif  // HRI__HRI_HPP_
