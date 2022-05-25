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
#include "hri_msgs/IdsList.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/RegionOfInterest.h"

using namespace std;
using namespace ros;
using namespace hri;

// waiting time for the libhri callback to process their inputs
#define WAIT std::this_thread::sleep_for(std::chrono::milliseconds(30))
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

  auto pub_r1 = nh.advertise<sensor_msgs::RegionOfInterest>("/humans/faces/A/roi", 1, true);  // /roi topic is latched
  auto pub_r2 = nh.advertise<sensor_msgs::RegionOfInterest>("/humans/faces/B/roi", 1, true);  // /roi topic is latched

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

  auto roi = sensor_msgs::RegionOfInterest();

  {
    auto face = faces["B"].lock();
    EXPECT_FALSE(face == nullptr);

    EXPECT_EQ(face->ns(), "/humans/faces/B");

    EXPECT_EQ(face->roi().width, 0);


    roi.width = 10;
    pub_r2.publish(roi);
    WAIT;
    EXPECT_EQ(face->roi().width, 10);

    roi.width = 20;
    pub_r2.publish(roi);
    WAIT;
    EXPECT_EQ(face->roi().width, 20);
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
    EXPECT_EQ(face_a->roi().width, 20);
    EXPECT_EQ(face_b->ns(), "/humans/faces/B");
    EXPECT_EQ(face_b->roi().width, 20);
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

  ASSERT_FALSE(p1->anonymous()) << "by default, persons are not supposed to be anonymous";

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

    ASSERT_FALSE(p1->anonymous());
    ASSERT_TRUE(p2->anonymous());

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


  EXPECT_CALL(face_callback, Call(testing::_)).Times(1);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(0);
  ids.ids = { "id1" };
  face_pub.publish(ids);

  WAIT;

  EXPECT_CALL(face_callback, Call(testing::_)).Times(1);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(0);
  ids.ids = { "id1", "id2" };
  face_pub.publish(ids);

  WAIT;

  EXPECT_CALL(face_callback, Call(testing::_)).Times(2);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(2);
  ids.ids = { "id3", "id4" };
  face_pub.publish(ids);

  WAIT;

  EXPECT_CALL(body_callback, Call(testing::_)).Times(2);
  EXPECT_CALL(body_lost_callback, Call(testing::_)).Times(0);
  ids.ids = { "id1", "id2" };
  body_pub.publish(ids);

  WAIT;

  EXPECT_CALL(face_callback, Call(testing::_)).Times(2);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(1);
  EXPECT_CALL(body_callback, Call(testing::_)).Times(1);
  EXPECT_CALL(body_lost_callback, Call(testing::_)).Times(0);
  ids.ids = { "id1", "id2", "id3" };
  face_pub.publish(ids);
  body_pub.publish(ids);

  WAIT;

  EXPECT_CALL(face_callback, Call(testing::_)).Times(3);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(3);
  EXPECT_CALL(body_callback, Call(testing::_)).Times(3);
  EXPECT_CALL(body_lost_callback, Call(testing::_)).Times(3);
  ids.ids = { "id5", "id6", "id7" };
  face_pub.publish(ids);
  body_pub.publish(ids);

  WAIT;

  EXPECT_CALL(voice_callback, Call(testing::_)).Times(2);
  EXPECT_CALL(person_callback, Call(testing::_)).Times(2);
  EXPECT_CALL(person_tracked_callback, Call(testing::_)).Times(2);
  ids.ids = { "id1", "id2" };
  voice_pub.publish(ids);
  person_pub.publish(ids);
  person_tracked_pub.publish(ids);

  WAIT;

  spinner.stop();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "test_hri");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI tests");
  return RUN_ALL_TESTS();
}

