// Copyright 2021 PAL Robotics S.L.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <hri/hri.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <memory>
#include "hri/face.h"
#include "hri/person.h"
#include "hri/voice.h"
#include "hri_msgs/EngagementLevel.h"
#include "hri_msgs/IdsList.h"
#include "hri_msgs/LiveSpeech.h"
#include "hri_msgs/NormalizedRegionOfInterest2D.h"
#include "hri_msgs/SoftBiometrics.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace ros;
using namespace hri;

// waiting time for the libhri callback to process their inputs
#define WAIT std::this_thread::sleep_for(std::chrono::milliseconds(50))
#define WAIT_LONG std::this_thread::sleep_for(std::chrono::milliseconds(100))
#define WAIT_DEBUG                                                                       \
  {                                                                                      \
    WAIT;                                                                                \
    cout << "waiting..." << endl;                                                        \
  }

TEST(libhri, GetFaces)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);

    ASSERT_EQ(pub.getNumSubscribers(), 1U);

    ASSERT_EQ(hri_listener.getFaces().size(), 0);

    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto faces = hri_listener.getFaces();
    EXPECT_EQ(faces.size(), 1U);
    ASSERT_TRUE(faces.find("A") != faces.end());
    EXPECT_TRUE(faces["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    faces = hri_listener.getFaces();
    EXPECT_EQ(faces.size(), 2U);
    EXPECT_TRUE(faces.find("A") != faces.end());
    EXPECT_TRUE(faces.find("B") != faces.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    faces = hri_listener.getFaces();
    EXPECT_EQ(faces.size(), 1U);
    EXPECT_TRUE(faces.find("A") == faces.end());
    ASSERT_TRUE(faces.find("B") != faces.end());

    weak_ptr<const Face> face_b = faces["B"];
    EXPECT_FALSE(face_b.expired());  // face B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 0U);

    EXPECT_TRUE(face_b.expired());  // face B does not exist anymore!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

TEST(libhri, GetFacesRoi)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  auto pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);

  auto pub_r1 = nh.advertise<hri_msgs::NormalizedRegionOfInterest2D>("/humans/faces/A/roi", 1, true);  // /roi topic is latched
  auto pub_r2 = nh.advertise<hri_msgs::NormalizedRegionOfInterest2D>("/humans/faces/B/roi", 1, true);  // /roi topic is latched

  auto ids = hri_msgs::IdsList();

  ids.ids = { "A" };
  pub.publish(ids);
  WAIT;

  EXPECT_EQ(pub_r1.getNumSubscribers(), 1U)
      << "Face A should have subscribed to /humans/faces/A/roi";


  ids.ids = { "B" };
  pub.publish(ids);
  WAIT;

  EXPECT_EQ(pub_r1.getNumSubscribers(), 0U)
      << "Face A is deleted. No one should be subscribed to /humans/faces/A/roi anymore";
  EXPECT_EQ(pub_r2.getNumSubscribers(), 1U)
      << "Face B should have subscribed to /humans/faces/B/roi";


  auto faces = hri_listener.getFaces();
  ASSERT_FALSE(faces["B"].expired());  // face B still exists!

  auto roi = hri_msgs::NormalizedRegionOfInterest2D();

  {
    auto face = faces["B"].lock();
    EXPECT_FALSE(face == nullptr);

    EXPECT_EQ(face->ns(), "/humans/faces/B");

    EXPECT_FLOAT_EQ(face->roi().xmax, 0.);


    roi.xmax = 0.3;
    pub_r2.publish(roi);
    WAIT;
    EXPECT_FLOAT_EQ(face->roi().xmax, 0.3);

    roi.xmax = 0.6;
    pub_r2.publish(roi);
    WAIT;
    EXPECT_FLOAT_EQ(face->roi().xmax, 0.6);
  }

  // RoI of face A published *before* face A is published in /faces/tracked,
  // but should still get its RoI, as /roi is latched.
  pub_r1.publish(roi);
  ids.ids = { "B", "A" };
  pub.publish(ids);
  WAIT;

  faces = hri_listener.getFaces();
  {
    auto face_a = faces["A"].lock();
    ASSERT_FALSE(face_a == nullptr);
    auto face_b = faces["B"].lock();
    ASSERT_FALSE(face_b == nullptr);

    EXPECT_EQ(face_a->ns(), "/humans/faces/A");
    EXPECT_FLOAT_EQ(face_a->roi().xmax, 0.6);
    EXPECT_EQ(face_b->ns(), "/humans/faces/B");
    EXPECT_FLOAT_EQ(face_b->roi().xmax, 0.6);
  }

  spinner.stop();
}

TEST(libhri, GetBodies)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/bodies/tracked", 1);

    ASSERT_EQ(pub.getNumSubscribers(), 1U);


    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto bodies = hri_listener.getBodies();
    EXPECT_EQ(bodies.size(), 1U);
    ASSERT_TRUE(bodies.find("A") != bodies.end());
    EXPECT_TRUE(bodies["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getBodies().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    bodies = hri_listener.getBodies();
    EXPECT_EQ(bodies.size(), 2U);
    EXPECT_TRUE(bodies.find("A") != bodies.end());
    EXPECT_TRUE(bodies.find("B") != bodies.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getBodies().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    bodies = hri_listener.getBodies();
    EXPECT_EQ(bodies.size(), 1U);
    EXPECT_TRUE(bodies.find("A") == bodies.end());
    ASSERT_TRUE(bodies.find("B") != bodies.end());

    weak_ptr<const Body> body_b = bodies["B"];
    EXPECT_FALSE(body_b.expired());  // body B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getBodies().size(), 0U);

    EXPECT_TRUE(body_b.expired());  // body B does not exist anymore!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

TEST(libhri, GetVoices)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/voices/tracked", 1);

    ASSERT_EQ(pub.getNumSubscribers(), 1U);


    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto voices = hri_listener.getVoices();
    EXPECT_EQ(voices.size(), 1U);
    ASSERT_TRUE(voices.find("A") != voices.end());
    EXPECT_TRUE(voices["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getVoices().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    voices = hri_listener.getVoices();
    EXPECT_EQ(voices.size(), 2U);
    EXPECT_TRUE(voices.find("A") != voices.end());
    EXPECT_TRUE(voices.find("B") != voices.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getVoices().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    voices = hri_listener.getVoices();
    EXPECT_EQ(voices.size(), 1U);
    EXPECT_TRUE(voices.find("A") == voices.end());
    ASSERT_TRUE(voices.find("B") != voices.end());

    weak_ptr<const Voice> voice_b = voices["B"];
    EXPECT_FALSE(voice_b.expired());  // voice B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getVoices().size(), 0U);

    EXPECT_TRUE(voice_b.expired());  // voice B does not exist anymore!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

TEST(libhri, GetKnownPersons)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1);

    ASSERT_EQ(pub.getNumSubscribers(), 1U);


    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto persons = hri_listener.getPersons();
    EXPECT_EQ(persons.size(), 1U);
    ASSERT_TRUE(persons.find("A") != persons.end());
    EXPECT_TRUE(persons["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getPersons().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    persons = hri_listener.getPersons();
    EXPECT_EQ(persons.size(), 2U);
    EXPECT_TRUE(persons.find("A") != persons.end());
    EXPECT_TRUE(persons.find("B") != persons.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getPersons().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    persons = hri_listener.getPersons();
    EXPECT_EQ(persons.size(), 1U) << "known persons can go down in case of eg an anonymous person";
    EXPECT_TRUE(persons.find("A") == persons.end());
    ASSERT_TRUE(persons.find("B") != persons.end());

    shared_ptr<const Person> person_b = persons["B"].lock();
    EXPECT_TRUE(person_b != nullptr);  // person B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getPersons().size(), 0U);

    EXPECT_TRUE(person_b != nullptr);  // person B still exists!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

TEST(libhri, GetTrackedPersons)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);

    ASSERT_EQ(pub.getNumSubscribers(), 1U);


    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto known_persons = hri_listener.getPersons();
    EXPECT_EQ(known_persons.size(), 0U);

    auto persons = hri_listener.getTrackedPersons();
    EXPECT_EQ(persons.size(), 1U);
    ASSERT_TRUE(persons.find("A") != persons.end());
    EXPECT_TRUE(persons["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getTrackedPersons().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    persons = hri_listener.getTrackedPersons();
    EXPECT_EQ(persons.size(), 2U);
    EXPECT_TRUE(persons.find("A") != persons.end());
    EXPECT_TRUE(persons.find("B") != persons.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getTrackedPersons().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    persons = hri_listener.getTrackedPersons();
    EXPECT_EQ(persons.size(), 1U);
    EXPECT_TRUE(persons.find("A") == persons.end());
    ASSERT_TRUE(persons.find("B") != persons.end());

    shared_ptr<const Person> person_b = persons["B"].lock();
    EXPECT_TRUE(person_b != nullptr);  // person B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getTrackedPersons().size(), 0U);

    EXPECT_TRUE(person_b != nullptr);  // person B still exists!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

TEST(libhri, PersonAttributes)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  auto person_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);
  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto person_face_pub = nh.advertise<std_msgs::String>("/humans/persons/p1/face_id", 1);

  auto person_ids = hri_msgs::IdsList();
  person_ids.ids = { "p1" };
  person_pub.publish(person_ids);

  auto face_ids = hri_msgs::IdsList();
  face_ids.ids = { "f1", "f2" };
  face_pub.publish(face_ids);

  WAIT;

  auto p1 = hri_listener.getTrackedPersons()["p1"].lock();

  ASSERT_FALSE(p1->anonymous()) << "whether a person is anonymous or not has to be explicitely set";

  auto face0 = p1->face();

  ASSERT_EQ(face0.lock(), nullptr);
  ASSERT_TRUE(face0.expired());

  auto face_id = std_msgs::String();
  face_id.data = "f1";

  person_face_pub.publish(face_id);

  WAIT;

  auto face1 = hri_listener.getTrackedPersons()["p1"].lock()->face();

  ASSERT_NE(face1.lock(), nullptr);
  ASSERT_FALSE(face1.expired());
  ASSERT_EQ(face1.lock()->id(), "f1");

  spinner.stop();
}

TEST(libhri, AnonymousPersonsAndAliases)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  auto person_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);
  auto p1_anon_pub = nh.advertise<std_msgs::Bool>("/humans/persons/p1/anonymous", 1);
  auto p2_anon_pub = nh.advertise<std_msgs::Bool>("/humans/persons/p2/anonymous", 1);

  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto p1_face_pub = nh.advertise<std_msgs::String>("/humans/persons/p1/face_id", 1);
  auto p2_face_pub = nh.advertise<std_msgs::String>("/humans/persons/p2/face_id", 1);

  auto p2_alias_pub = nh.advertise<std_msgs::String>("/humans/persons/p2/alias", 1);

  auto person_ids = hri_msgs::IdsList();
  person_ids.ids = { "p1", "p2" };
  person_pub.publish(person_ids);

  auto face_ids = hri_msgs::IdsList();
  face_ids.ids = { "f1", "f2" };
  face_pub.publish(face_ids);

  WAIT;

  // each person is associated to a face
  auto face_id = std_msgs::String();
  face_id.data = "f1";
  p1_face_pub.publish(face_id);
  face_id.data = "f2";
  p2_face_pub.publish(face_id);


  WAIT;

  std_msgs::Bool msg;
  msg.data = false;
  p1_anon_pub.publish(msg);
  msg.data = true;
  p2_anon_pub.publish(msg);

  WAIT;

  ASSERT_EQ(hri_listener.getTrackedPersons().size(), 2);

  auto p1 = hri_listener.getTrackedPersons()["p1"].lock();

  {
    auto p2 = hri_listener.getTrackedPersons()["p2"].lock();

    ASSERT_TRUE(p1->anonymous()); // the anonymous optional flag should have been set
    ASSERT_TRUE(p2->anonymous()); // the anonymous optional flag should have been set
    ASSERT_FALSE(*(p1->anonymous()));
    ASSERT_TRUE(*(p2->anonymous()));

    // being anonymous or not should have no impact on face associations
    ASSERT_EQ(p1->face().lock()->id(), "f1");
    ASSERT_EQ(p2->face().lock()->id(), "f2");
  }

  ///////////// ALIASES ///////////////////////////

  // set p2 as an alias of p1
  auto alias_id = std_msgs::String();
  alias_id.data = "p1";

  p2_alias_pub.publish(alias_id);

  WAIT;

  ASSERT_EQ(hri_listener.getTrackedPersons().size(), 2U);

  {
    auto p2 = hri_listener.getTrackedPersons()["p2"].lock();

    ASSERT_EQ(p1, p2) << "p2 should now point to the same person as p1";

    ASSERT_EQ(p2->face().lock()->id(), "f1") << "p2's face now points to f1";
  }
  // remove the alias
  alias_id.data = "";

  p2_alias_pub.publish(alias_id);

  WAIT;

  {
    auto p2 = hri_listener.getTrackedPersons()["p2"].lock();

    ASSERT_NE(p1, p2) << "p2 is not anymore the same person as p1";

    ASSERT_EQ(p2->face().lock()->id(), "f2")
        << "p2's face should still points to its former f2 face";
  }

  // republish the alias
  alias_id.data = "p1";

  p2_alias_pub.publish(alias_id);

  WAIT;

  auto p2 = hri_listener.getTrackedPersons()["p2"].lock();

  ASSERT_EQ(p1, p2) << "p2 is again the same person as p1";

  // delete p1 -> p2 should be deleted as well

  person_ids.ids = { "p2" };
  person_pub.publish(person_ids);

  WAIT;

  ASSERT_EQ(hri_listener.getTrackedPersons().size(), 0U)
      << "the aliased person should have been deleted with its alias";

  spinner.stop();
}


TEST(libhri, Callbacks)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  // create mock callbacks
  testing::MockFunction<void(FaceWeakConstPtr)> face_callback;
  hri_listener.onFace(face_callback.AsStdFunction());

  testing::MockFunction<void(ID)> face_lost_callback;
  hri_listener.onFaceLost(face_lost_callback.AsStdFunction());


  testing::MockFunction<void(BodyWeakConstPtr)> body_callback;
  hri_listener.onBody(body_callback.AsStdFunction());

  testing::MockFunction<void(ID)> body_lost_callback;
  hri_listener.onBodyLost(body_lost_callback.AsStdFunction());


  testing::MockFunction<void(VoiceWeakConstPtr)> voice_callback;
  hri_listener.onVoice(voice_callback.AsStdFunction());

  testing::MockFunction<void(ID)> voice_lost_callback;
  hri_listener.onVoiceLost(voice_lost_callback.AsStdFunction());


  testing::MockFunction<void(PersonWeakConstPtr)> person_callback;
  hri_listener.onPerson(person_callback.AsStdFunction());

  testing::MockFunction<void(PersonWeakConstPtr)> person_tracked_callback;
  hri_listener.onTrackedPerson(person_tracked_callback.AsStdFunction());

  testing::MockFunction<void(ID)> person_tracked_lost_callback;
  hri_listener.onTrackedPersonLost(person_tracked_lost_callback.AsStdFunction());



  auto ids = hri_msgs::IdsList();

  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto body_pub = nh.advertise<hri_msgs::IdsList>("/humans/bodies/tracked", 1);
  auto voice_pub = nh.advertise<hri_msgs::IdsList>("/humans/voices/tracked", 1);
  auto person_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1);
  auto person_tracked_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);


  // uses a workaround to fake interleaving of EXPECT_CALL with function calls, see https://stackoverflow.com/a/60905880
  uint32_t face_call_count = 0;
  EXPECT_CALL(face_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&face_call_count](){ face_call_count++; }));

  uint32_t face_lost_call_count = 0;
  EXPECT_CALL(face_lost_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&face_lost_call_count](){ face_lost_call_count++; }));

  uint32_t body_call_count = 0;
  EXPECT_CALL(body_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&body_call_count](){ body_call_count++; }));

  uint32_t body_lost_call_count = 0;
  EXPECT_CALL(body_lost_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&body_lost_call_count](){ body_lost_call_count++; }));

  uint32_t voice_call_count = 0;
  EXPECT_CALL(voice_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&voice_call_count](){ voice_call_count++; }));

  uint32_t voice_lost_call_count = 0;
  EXPECT_CALL(voice_lost_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&voice_lost_call_count](){ voice_lost_call_count++; }));

  uint32_t person_call_count = 0;
  EXPECT_CALL(person_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&person_call_count](){ person_call_count++; }));

  uint32_t person_tracked_call_count = 0;
  EXPECT_CALL(person_tracked_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&person_tracked_call_count](){ person_tracked_call_count++; }));

  uint32_t person_tracked_lost_call_count = 0;
  EXPECT_CALL(person_tracked_lost_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&person_tracked_lost_call_count](){ person_tracked_lost_call_count++; }));


  // start sequential testing
  face_call_count = 0;
  face_lost_call_count = 0;
  ids.ids = { "id1" };
  face_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(face_call_count, 1);
  EXPECT_EQ(face_lost_call_count, 0);


  face_call_count = 0;
  face_lost_call_count = 0;
  ids.ids = { "id1", "id2" };
  face_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(face_call_count, 1);
  EXPECT_EQ(face_lost_call_count, 0);

  face_call_count = 0;
  face_lost_call_count = 0;
  ids.ids = { "id3", "id4" };
  face_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(face_call_count, 2);
  EXPECT_EQ(face_lost_call_count, 2);

  body_call_count = 0;
  body_lost_call_count = 0;
  ids.ids = { "id1", "id2" };
  body_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(body_call_count, 2);
  EXPECT_EQ(body_lost_call_count, 0);

  face_call_count = 0;
  face_lost_call_count = 0;
  body_call_count = 0;
  body_lost_call_count = 0;
  ids.ids = { "id1", "id2", "id3" };
  face_pub.publish(ids);
  body_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(face_call_count, 2);
  EXPECT_EQ(face_lost_call_count, 1);
  EXPECT_EQ(body_call_count, 1);
  EXPECT_EQ(body_lost_call_count, 0);

  face_call_count = 0;
  face_lost_call_count = 0;
  body_call_count = 0;
  body_lost_call_count = 0;
  ids.ids = { "id5", "id6", "id7" };
  face_pub.publish(ids);
  body_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(face_call_count, 3);
  EXPECT_EQ(face_lost_call_count, 3);
  EXPECT_EQ(body_call_count, 3);
  EXPECT_EQ(body_lost_call_count, 3);

  voice_call_count = 0;
  person_call_count = 0;
  person_tracked_call_count = 0;
  ids.ids = { "id1", "id2" };
  voice_pub.publish(ids);
  person_pub.publish(ids);
  person_tracked_pub.publish(ids);
  WAIT_LONG;
  EXPECT_EQ(voice_call_count, 2);
  EXPECT_EQ(person_call_count, 2);
  EXPECT_EQ(person_tracked_call_count, 2);

  spinner.stop();
}

TEST(libhri, SoftBiometrics)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  auto person_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);
  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto person_face_pub = nh.advertise<std_msgs::String>("/humans/persons/p1/face_id", 1);
  auto softbiometrics_pub =
      nh.advertise<hri_msgs::SoftBiometrics>("/humans/faces/f1/softbiometrics", 1);

  auto person_ids = hri_msgs::IdsList();
  person_ids.ids = { "p1" };
  person_pub.publish(person_ids);

  auto face_ids = hri_msgs::IdsList();
  face_ids.ids = { "f1" };
  face_pub.publish(face_ids);

  WAIT;

  auto softbiometrics_msg = hri_msgs::SoftBiometrics();
  softbiometrics_msg.age = 45;
  softbiometrics_msg.age_confidence = 0.8;
  softbiometrics_msg.gender = hri_msgs::SoftBiometrics::FEMALE;
  softbiometrics_msg.gender_confidence = 0.7;
  softbiometrics_pub.publish(softbiometrics_msg);

  auto face_id = std_msgs::String();
  face_id.data = "f1";

  person_face_pub.publish(face_id);

  WAIT;

  auto face = hri_listener.getTrackedPersons()["p1"].lock()->face().lock();

  ASSERT_EQ(face->id(), "f1");

  ASSERT_TRUE(face->age());
  ASSERT_EQ(*(face->age()), 45);
  ASSERT_TRUE(face->gender());
  ASSERT_EQ(*(face->gender()), hri::FEMALE);

  softbiometrics_msg.gender = hri_msgs::SoftBiometrics::OTHER;
  softbiometrics_pub.publish(softbiometrics_msg);
  WAIT;

  ASSERT_EQ(*(face->gender()), hri::OTHER);

  softbiometrics_msg.gender = hri_msgs::SoftBiometrics::UNDEFINED;
  softbiometrics_pub.publish(softbiometrics_msg);
  WAIT;

  ASSERT_FALSE(face->gender());

  spinner.stop();
}

TEST(libhri, EngagementLevel)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  auto person_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);
  auto engagement_pub =
      nh.advertise<hri_msgs::EngagementLevel>("/humans/persons/p1/engagement_status", 1);

  auto person_ids = hri_msgs::IdsList();
  person_ids.ids = { "p1" };
  person_pub.publish(person_ids);

  WAIT;

  auto msg = hri_msgs::EngagementLevel();
  msg.level = hri_msgs::EngagementLevel::DISENGAGED;
  engagement_pub.publish(msg);

  WAIT;

  auto p = hri_listener.getTrackedPersons()["p1"].lock();
  ASSERT_TRUE(p->engagement_status());
  ASSERT_EQ(*(p->engagement_status()), hri::DISENGAGED);

  msg.level = hri_msgs::EngagementLevel::ENGAGED;
  engagement_pub.publish(msg);
  WAIT;

  ASSERT_EQ(*(p->engagement_status()), hri::ENGAGED);

  msg.level = hri_msgs::EngagementLevel::UNKNOWN;
  engagement_pub.publish(msg);
  WAIT;

  ASSERT_FALSE(p->engagement_status());

  spinner.stop();
}

TEST(libhri, PeopleLocation)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;
  hri_listener.setReferenceFrame("base_link");

  tf2_ros::StaticTransformBroadcaster static_broadcaster;

  geometry_msgs::TransformStamped world_transform;
  world_transform.header.stamp = ros::Time::now();
  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base_link";
  world_transform.transform.translation.x = -1.0;
  world_transform.transform.translation.y = 0.0;
  world_transform.transform.translation.z = 0.0;
  world_transform.transform.rotation.w = 1.0;
  static_broadcaster.sendTransform(world_transform);

  geometry_msgs::TransformStamped p1_transform;
  p1_transform.header.stamp = ros::Time::now();
  p1_transform.header.frame_id = "world";
  p1_transform.child_frame_id = "person_p1";
  p1_transform.transform.translation.x = 1.0;
  p1_transform.transform.translation.y = 0.0;
  p1_transform.transform.translation.z = 0.0;
  p1_transform.transform.rotation.w = 1.0;

  auto person_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);
  auto loc_confidence_pub =
      nh.advertise<std_msgs::Float32>("/humans/persons/p1/location_confidence", 1);

  auto person_ids = hri_msgs::IdsList();
  person_ids.ids = { "p1" };
  person_pub.publish(person_ids);

  WAIT;

  auto p = hri_listener.getTrackedPersons()["p1"].lock();

  auto msg = std_msgs::Float32();
  msg.data = 0.;
  loc_confidence_pub.publish(msg);
  WAIT;

  ASSERT_EQ(p->location_confidence(), 0.);
  ASSERT_FALSE(p->transform()) << "location confidence at 0, no transform should be available";

  msg.data = 0.5;
  loc_confidence_pub.publish(msg);
  WAIT;

  ASSERT_EQ(p->location_confidence(), 0.5);
  p->transform();
  ASSERT_FALSE(p->transform()) << "location confidence > 0 but no transform published yet -> no transform should be returned";


  static_broadcaster.sendTransform(p1_transform);
  WAIT;

  ASSERT_EQ(p->location_confidence(), 0.5);
  ASSERT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";
  auto t = *(p->transform());
  ASSERT_EQ(t.child_frame_id, "person_p1");
  ASSERT_EQ(t.header.frame_id, "base_link");
  ASSERT_EQ(t.transform.translation.x, 2.0);


  msg.data = 1.0;
  loc_confidence_pub.publish(msg);
  WAIT;

  ASSERT_EQ(p->location_confidence(), 1.);
  ASSERT_TRUE(p->transform()) << "location confidence > 0 => a transform should be available";

  spinner.stop();
}

TEST(libhri, SpeechCallbacks)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  // create mock callbacks
  testing::MockFunction<void(bool)> is_speaking_callback;
  testing::MockFunction<void(string)> speech_callback;
  testing::MockFunction<void(string)> incremental_speech_callback;

  hri_listener.onVoice([&](VoiceWeakPtr weak_voice) {
    auto voice = weak_voice.lock();
    voice->onSpeaking(is_speaking_callback.AsStdFunction());
    voice->onSpeech(speech_callback.AsStdFunction());
    voice->onIncrementalSpeech(incremental_speech_callback.AsStdFunction());
  });

  // testing::MockFunction<void(ID)> voice_lost_callback;
  // hri_listener.onVoiceLost(voice_lost_callback.AsStdFunction());

  auto ids = hri_msgs::IdsList();
  auto is_speaking = std_msgs::Bool();
  auto speech = hri_msgs::LiveSpeech();

  auto voice_pub = nh.advertise<hri_msgs::IdsList>("/humans/voices/tracked", 1);
  auto is_speaking_pub = nh.advertise<std_msgs::Bool>("/humans/voices/id1/is_speaking", 1);
  auto speech_pub = nh.advertise<hri_msgs::LiveSpeech>("/humans/voices/id1/speech", 1);


  // uses a workaround to fake interleaving of EXPECT_CALL with function calls, see https://stackoverflow.com/a/60905880
  uint32_t is_speaking_call_count = 0;
  EXPECT_CALL(is_speaking_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&is_speaking_call_count](){ is_speaking_call_count++; }));

  uint32_t speech_call_count = 0;
  EXPECT_CALL(speech_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&speech_call_count](){ speech_call_count++; }));

  uint32_t incremental_speech_call_count = 0;
  EXPECT_CALL(incremental_speech_callback, Call(testing::_)).WillRepeatedly(
    testing::InvokeWithoutArgs([&incremental_speech_call_count](){ incremental_speech_call_count++; }));


  // start sequential testing
  ids.ids = { "id1" };
  voice_pub.publish(ids);
  WAIT;

  is_speaking_call_count = 0;
  speech_call_count = 0;
  incremental_speech_call_count = 0;
  is_speaking.data = true;
  is_speaking_pub.publish(is_speaking);
  WAIT;
  EXPECT_EQ(is_speaking_call_count, 1);
  EXPECT_EQ(speech_call_count, 0);
  EXPECT_EQ(incremental_speech_call_count, 0);

  is_speaking_call_count = 0;
  is_speaking.data = false;
  is_speaking_pub.publish(is_speaking);
  WAIT;
  EXPECT_EQ(is_speaking_call_count, 1);

  is_speaking_call_count = 0;
  speech_call_count = 0;
  incremental_speech_call_count = 0;
  speech.final = "final sentence";
  speech.incremental = "";
  speech_pub.publish(speech);
  WAIT;
  EXPECT_EQ(is_speaking_call_count, 0);
  EXPECT_EQ(speech_call_count, 1);
  EXPECT_EQ(incremental_speech_call_count, 0);

  is_speaking_call_count = 0;
  speech_call_count = 0;
  incremental_speech_call_count = 0;
  speech.final = "";
  speech.incremental = "incremental sentence";
  speech_pub.publish(speech);
  WAIT;
  EXPECT_EQ(is_speaking_call_count, 0);
  EXPECT_EQ(speech_call_count, 0);
  EXPECT_EQ(incremental_speech_call_count, 1);

  is_speaking_call_count = 0;
  speech_call_count = 0;
  incremental_speech_call_count = 0;
  speech.final = "final sentence";
  speech.incremental = "incremental sentence";
  speech_pub.publish(speech);
  WAIT;
  EXPECT_EQ(is_speaking_call_count, 0);
  EXPECT_EQ(speech_call_count, 1);
  EXPECT_EQ(incremental_speech_call_count, 1);

  spinner.stop();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "test_hri");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI tests");
  return RUN_ALL_TESTS();
}

