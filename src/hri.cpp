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

#include "hri/hri.h"
#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <tuple>
#include <utility>
#include "hri/base.h"
#include "hri/body.h"
#include "hri/face.h"
#include "hri/person.h"
#include "hri_msgs/EngagementLevel.h"
#include "hri_msgs/FacialLandmarks.h"
#include "hri_msgs/IdsList.h"
#include "hri_msgs/BoolHRI.h"
#include "hri_msgs/SoftBiometrics.h"
#include "hri_msgs/StringHRI.h"
#include "hri_msgs/Float32HRI.h"
#include "hri_msgs/ImageHRI.h"
#include "hri_msgs/NormalizedRegionOfInterest2D.h"
#include "ros/subscriber.h"

#define CHECK_ID_AND_DO(dict, msg, func, param, debug_msg)                               \
  if (dict.count(msg->header.id))                                                        \
  {                                                                                      \
    dict.at(msg->header.id)->func(param);                                                \
  }                                                                                      \
  else                                                                                   \
  {                                                                                      \
    ROS_WARN_STREAM(debug_msg << msg->header.id);                                        \
  }

#define CHECK_PERSON_AND_SET(msg, param, value, debug_msg)                               \
  if (persons.count(msg->header.id))                                                     \
  {                                                                                      \
    persons.at(msg->header.id)->param = value;                                           \
  }                                                                                      \
  else                                                                                   \
  {                                                                                      \
    ROS_WARN_STREAM(debug_msg << msg->header.id);                                        \
  }                                                                                      \
  /* if the person is *also* currently tracked, update that value as well */             \
  if (tracked_persons.count(msg->header.id))                                             \
  {                                                                                      \
    tracked_persons.at(msg->header.id)->param = value;                                   \
  }


using namespace std;
using namespace hri;

HRIListener::HRIListener()
  : _reference_frame("base_link")
  , _tf_listener(_tf_buffer)
  , hri_subscribers_({ {

        //////////////////////////////////////////////////////////////////
        // FACES
        //

        { "face_tracked", node_.subscribe<hri_msgs::IdsList>(
                              "/humans/faces/tracked", 1,
                              bind(&HRIListener::onTrackedFeature, this, FeatureType::face, _1)) },

        { "face_roi", node_.subscribe<hri_msgs::NormalizedRegionOfInterest2D>(
                          "/humans/faces/roi", 1,
                          [&](auto msg) {
                            CHECK_ID_AND_DO(faces, msg, onRoI, msg,
                                            "Trying to update RoI of unknown face ");
                          }) },


        { "face_cropped", node_.subscribe<hri_msgs::ImageHRI>(
                              "/humans/faces/cropped", 1,
                              [&](auto msg) {
                                CHECK_ID_AND_DO(faces, msg, onCropped, msg->image,
                                                "Trying to update cropped image for unknown face ");
                              }) },

        { "face_aligned", node_.subscribe<hri_msgs::ImageHRI>(
                              "/humans/faces/aligned", 1,
                              [&](auto msg) {
                                CHECK_ID_AND_DO(faces, msg, onAligned, msg->image,
                                                "Trying to update aligned image for unknown face ");
                              }) },

        { "face_landmarks", node_.subscribe<hri_msgs::FacialLandmarks>(
                                "/humans/faces/landmarks", 1,
                                [&](auto msg) {
                                  CHECK_ID_AND_DO(faces, msg, onLandmarks, msg,
                                                  "Trying to update facial landmarks for unknown face ");
                                }) },

        { "face_softbiometrics", node_.subscribe<hri_msgs::SoftBiometrics>(
                                     "/humans/faces/softbiometrics", 1,
                                     [&](auto msg) {
                                       CHECK_ID_AND_DO(faces, msg, onSoftBiometrics, msg,
                                                       "Trying to update soft biometrics for unknown face ");
                                     }) },

        //////////////////////////////////////////////////////////////////
        // BODIES
        //

        { "body_tracked", node_.subscribe<hri_msgs::IdsList>(
                              "/humans/bodies/tracked", 1,
                              bind(&HRIListener::onTrackedFeature, this, FeatureType::body, _1)) },

        { "body_roi", node_.subscribe<hri_msgs::NormalizedRegionOfInterest2D>(
                          "/humans/bodies/roi", 1,
                          [&](auto msg) {
                            CHECK_ID_AND_DO(bodies, msg, onRoI, msg,
                                            "Trying to update RoI for unknown body ");
                          }) },

        { "body_cropped", node_.subscribe<hri_msgs::ImageHRI>(
                              "/humans/bodies/cropped", 1,
                              [&](auto msg) {
                                CHECK_ID_AND_DO(bodies, msg, onCropped, msg->image,
                                                "Trying to update cropped image for unknown body ");
                              }) },

        //////////////////////////////////////////////////////////////////
        // VOICES
        //

        { "voice_tracked", node_.subscribe<hri_msgs::IdsList>(
                               "/humans/voices/tracked", 1,
                               bind(&HRIListener::onTrackedFeature, this, FeatureType::voice, _1)) },

        //////////////////////////////////////////////////////////////////
        // PERSONS
        //

        { "person_known", node_.subscribe<hri_msgs::IdsList>(
                              "/humans/persons/known", 1,
                              bind(&HRIListener::onTrackedFeature, this, FeatureType::person, _1)) },

        { "person_tracked",
          node_.subscribe<hri_msgs::IdsList>("/humans/persons/tracked", 1,
                                             bind(&HRIListener::onTrackedFeature, this,
                                                  FeatureType::tracked_person, _1)) },

        { "person_anonymous",
          node_.subscribe<hri_msgs::BoolHRI>("/humans/persons/anonymous", 1,
                                             [&](auto msg) {
                                               CHECK_PERSON_AND_SET(
                                                   msg, _anonymous, msg->data,
                                                   "Trying to update anonymous status for unknown person ");
                                             }) },

        { "person_face_id",
          node_.subscribe<hri_msgs::StringHRI>("/humans/persons/face_id", 1,
                                               [&](const auto msg) {
                                                 CHECK_PERSON_AND_SET(
                                                     msg, face_id, msg->data,
                                                     "Trying to update face_id for unknown person ");
                                               }) },

        { "person_body_id",
          node_.subscribe<hri_msgs::StringHRI>("/humans/persons/body_id", 1,
                                               [&](const auto msg) {
                                                 CHECK_PERSON_AND_SET(
                                                     msg, body_id, msg->data,
                                                     "Trying to update body_id for unknown person ");
                                               }) },

        { "person_voice_id",
          node_.subscribe<hri_msgs::StringHRI>("/humans/persons/voice_id", 1,
                                               [&](const auto msg) {
                                                 CHECK_PERSON_AND_SET(
                                                     msg, voice_id, msg->data,
                                                     "Trying to update voice_id for unknown person ");
                                               }) },

        { "person_alias", node_.subscribe<hri_msgs::StringHRI>("/humans/persons/alias", 1,
                                                               [&](auto msg) {
                                                                 CHECK_PERSON_AND_SET(
                                                                     msg, _alias, msg->data,
                                                                     "Trying to update alias for unknown person ");
                                                               }) },

        { "person_engagement_status",
          node_.subscribe<
              hri_msgs::EngagementLevel>("/humans/persons/engagement_status", 1,
                                         [&](auto msg) {
                                           CHECK_PERSON_AND_SET(
                                               msg, _engagement_status, msg,
                                               "Trying to update engagement status for unknown person ");
                                         }) },

        { "person_location_confidence",
          node_.subscribe<hri_msgs::Float32HRI>("/humans/persons/location_confidence", 1,
                                                [&](auto msg) {
                                                  CHECK_PERSON_AND_SET(
                                                      msg, _loc_confidence, msg->data,
                                                      "Trying to update location confidence for unknown person ");
                                                }) },
    } })
{
  ROS_DEBUG("Initialising the HRI Listener");
}

HRIListener::~HRIListener()
{
  ROS_DEBUG("Closing the HRI Listener");


  for (auto& sub : hri_subscribers_)
  {
    sub.second.shutdown();
  }

  persons.clear();
  faces.clear();
  bodies.clear();
  voices.clear();
}


map<ID, FaceWeakConstPtr> HRIListener::getFaces() const
{
  map<ID, FaceWeakConstPtr> result;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : faces)
  {
    result[f.first] = f.second;
  }

  return result;
}

map<ID, BodyWeakConstPtr> HRIListener::getBodies() const
{
  map<ID, BodyWeakConstPtr> result;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : bodies)
  {
    result[f.first] = f.second;
  }

  return result;
}

map<ID, VoiceWeakConstPtr> HRIListener::getVoices() const
{
  map<ID, VoiceWeakConstPtr> result;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : voices)
  {
    result[f.first] = f.second;
  }

  return result;
}

map<ID, PersonWeakConstPtr> HRIListener::getPersons() const
{
  map<ID, PersonWeakConstPtr> result;

  vector<PersonConstPtr> aliased;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : persons)
  {
    if (f.second->alias().empty())
    {
      result[f.first] = f.second;
    }
    else
    {
      aliased.push_back(f.second);
    }
  }

  for (auto const& p : aliased)
  {
    if (result.count(p->alias()) != 0)
    {
      result[p->id()] = result[p->alias()];
    }
    else  // ouch! the person points to an inexistant alias! this should not happen
    {
      assert(false);
    }
  }

  return result;
}

map<ID, PersonWeakConstPtr> HRIListener::getTrackedPersons() const
{
  map<ID, PersonWeakConstPtr> result;

  vector<PersonConstPtr> aliased;

  // creates a map of *weak* pointers from the internally managed list of
  // shared pointers
  for (auto const& f : tracked_persons)
  {
    if (f.second->alias().empty())
    {
      result[f.first] = f.second;
    }
    else
    {
      aliased.push_back(f.second);
    }
  }

  for (auto const& p : aliased)
  {
    if (result.count(p->alias()) != 0)
    {
      result[p->id()] = result[p->alias()];
    }
    else  // ouch! the person points to an inexistant alias! this should not happen.
    {
      assert(false);
    }
  }

  return result;
}

void HRIListener::onTrackedFeature(FeatureType feature, hri_msgs::IdsListConstPtr tracked)
{
  // update the current list of tracked feature (face, body...) with
  // what has just been received on the respective /tracked topic.

  set<ID> new_ids;
  for (auto const& id : tracked->ids)
  {
    new_ids.insert(ID(id));
  }

  set<ID> to_remove;
  set<ID> to_add;

  set<ID> current_ids;

  switch (feature)
  {
    case FeatureType::face:
      for (auto const& kv : faces)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::body:
      for (auto const& kv : bodies)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::voice:
      for (auto const& kv : voices)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::person:
      for (auto const& kv : persons)
      {
        current_ids.insert(kv.first);
      }
      break;
    case FeatureType::tracked_person:
      for (auto const& kv : tracked_persons)
      {
        current_ids.insert(kv.first);
      }
      break;
  }



  for (auto id : new_ids)
  {
    if (current_ids.find(id) == current_ids.end())
    {
      to_add.insert(id);
    }
  }

  for (auto id : current_ids)
  {
    if (new_ids.find(id) == new_ids.end())
    {
      to_remove.insert(id);
    }
  }

  switch (feature)
  {
    case FeatureType::face:
      for (auto id : to_remove)
      {
        faces.erase(id);

        // invoke all the callbacks
        for (auto& cb : face_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::body:
      for (auto id : to_remove)
      {
        bodies.erase(id);

        // invoke all the callbacks
        for (auto& cb : body_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::voice:
      for (auto id : to_remove)
      {
        voices.erase(id);

        // invoke all the callbacks
        for (auto& cb : voice_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::tracked_person:
      for (auto id : to_remove)
      {
        tracked_persons.erase(id);

        // also erase the *aliases* of this ID
        vector<ID> aliases;
        for (const auto& p : tracked_persons)
        {
          if (p.second->alias() == id)
          {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases)
        {
          tracked_persons.erase(alias);
        }

        // invoke all the callbacks
        for (auto& cb : person_tracked_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
    case FeatureType::person:
      for (auto id : to_remove)
      {
        persons.erase(id);

        // also erase the *aliases* of this ID
        vector<ID> aliases;
        for (const auto& p : persons)
        {
          if (p.second->alias() == id)
          {
            aliases.push_back(p.first);
          }
        }
        for (auto alias : aliases)
        {
          persons.erase(alias);
        }

        // invoke all the callbacks
        for (auto& cb : person_lost_callbacks)
        {
          cb(id);
        }
      }
      break;
  }

  switch (feature)
  {
    case FeatureType::face:
      for (auto id : to_add)
      {
        auto face = make_shared<Face>(id, node_);
        face->init();
        faces.insert({ id, face });

        // invoke all the callbacks
        for (auto& cb : face_callbacks)
        {
          cb(face);
        }
      }
      break;
    case FeatureType::body:
      for (auto id : to_add)
      {
        auto body = make_shared<Body>(id, node_);
        body->init();
        bodies.insert({ id, body });

        // invoke all the callbacks
        for (auto& cb : body_callbacks)
        {
          cb(body);
        }
      }
      break;
    case FeatureType::voice:
      for (auto id : to_add)
      {
        auto voice = make_shared<Voice>(id, node_);
        voice->init();
        voices.insert({ id, voice });

        // invoke all the callbacks
        for (auto& cb : voice_callbacks)
        {
          cb(voice);
        }
      }
      break;
    case FeatureType::person:
      for (auto id : to_add)
      {
        auto person = make_shared<Person>(id, this, node_, &_tf_buffer, _reference_frame);
        person->init();
        persons.insert({ id, person });

        // invoke all the callbacks
        for (auto& cb : person_callbacks)
        {
          cb(person);
        }
      }
      break;
    case FeatureType::tracked_person:
      for (auto id : to_add)
      {
        auto person = make_shared<Person>(id, this, node_, &_tf_buffer, _reference_frame);
        person->init();
        tracked_persons.insert({ id, person });

        // invoke all the callbacks
        for (auto& cb : person_tracked_callbacks)
        {
          cb(person);
        }
      }
      break;
  }
}

