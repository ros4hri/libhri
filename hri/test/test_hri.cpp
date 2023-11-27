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

#include <thread>
#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "hri/hri.hpp"
#include "hri/face.hpp"
#include "hri/person.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/live_speech.hpp"
#include "hri_msgs/msg/normalized_region_of_interest2_d.hpp"
#include "hri_msgs/msg/skeleton2_d.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class HRITest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, NULL);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    tester_node_ = rclcpp::Node::make_shared("tester_node_");
    hri_executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
    hri_node_ = rclcpp::Node::make_shared("hri_node");
    hri_executor_->add_node(hri_node_);
    hri_listener_ = hri::HRIListener::create(hri_node_);
  }

  void TearDown() override
  {
    hri_listener_.reset();
    hri_node_.reset();
    hri_executor_.reset();
    tester_node_.reset();
  }

  void spin(std::chrono::nanoseconds hri_timeout = 100ms)
  {
    hri_executor_->spin_some(hri_timeout);
  }

  const rclcpp::QoS kQoSLatched_{rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()};
  rclcpp::Executor::SharedPtr hri_executor_;
  rclcpp::Node::SharedPtr hri_node_;
  rclcpp::Node::SharedPtr tester_node_;
  std::shared_ptr<hri::HRIListener> hri_listener_;
};

TEST_F(HRITest, GetFaces)
{
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto ids_msg = hri_msgs::msg::IdsList();

  ASSERT_EQ(faces_pub->get_subscription_count(), 1U);
  EXPECT_EQ(hri_listener_->getFaces().size(), 0U);

  ids_msg.ids = {"A"};
  faces_pub->publish(ids_msg);
  spin();
  auto faces = hri_listener_->getFaces();
  EXPECT_EQ(faces.size(), 1U);
  ASSERT_TRUE(faces.count("A"));
  EXPECT_EQ(faces["A"]->id(), "A");

  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getFaces().size(), 1U);

  ids_msg.ids = {"A", "B"};
  faces_pub->publish(ids_msg);
  spin();
  faces = hri_listener_->getFaces();
  EXPECT_EQ(faces.size(), 2U);
  EXPECT_TRUE(faces.count("A"));
  EXPECT_TRUE(faces.count("B"));

  ids_msg.ids = {"B"};
  faces_pub->publish(ids_msg);
  spin();
  faces = hri_listener_->getFaces();
  EXPECT_EQ(faces.size(), 1U);
  EXPECT_FALSE(faces.count("A"));
  ASSERT_TRUE(faces.count("B"));

  ids_msg.ids = {};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getFaces().size(), 0U);
  // check face B is not used anymore by hri_listener_!
  EXPECT_EQ(faces["B"].use_count(), 1U);

  hri_listener_.reset();
  EXPECT_EQ(faces_pub->get_subscription_count(), 0U);
  EXPECT_FALSE(faces["B"]->valid());
}

TEST_F(HRITest, GetFacesRoi)
{
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto roi_a_pub = tester_node_->create_publisher<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    "/humans/faces/A/roi", 1);
  auto roi_b_pub = tester_node_->create_publisher<hri_msgs::msg::NormalizedRegionOfInterest2D>(
    "/humans/faces/B/roi", 1);
  auto ids_msg = hri_msgs::msg::IdsList();
  auto roi_msg = hri_msgs::msg::NormalizedRegionOfInterest2D();

  ids_msg.ids = {"A"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(roi_a_pub->get_subscription_count(), 1U);

  ids_msg.ids = {"B"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(roi_a_pub->get_subscription_count(), 0U)
    << "Face A is deleted. No one should be subscribed to /humans/faces/A/roi anymore";
  EXPECT_EQ(roi_b_pub->get_subscription_count(), 1U)
    << "Face B should have subscribed to /humans/faces/B/roi";
  auto faces = hri_listener_->getFaces();
  ASSERT_TRUE(faces.count("B"));
  auto face = faces["B"];
  EXPECT_FALSE(face == nullptr);
  EXPECT_EQ(face->ns(), "/humans/faces/B");
  EXPECT_FALSE(face->roi());

  roi_msg.xmin = 0.1;
  roi_msg.ymin = 0;
  roi_msg.xmax = 1;
  roi_msg.ymax = 1;
  roi_b_pub->publish(roi_msg);
  spin();
  ASSERT_TRUE(face->roi());
  EXPECT_FLOAT_EQ(face->roi().value().x, 0.1f);

  roi_msg.xmin = 0.2;
  roi_b_pub->publish(roi_msg);
  spin();
  EXPECT_FLOAT_EQ(face->roi().value().x, 0.2f);

  ids_msg.ids = {"B", "A"};
  faces_pub->publish(ids_msg);
  spin();

  roi_a_pub->publish(roi_msg);
  spin();
  faces = hri_listener_->getFaces();
  auto face_a = faces["A"];
  auto face_b = faces["B"];
  ASSERT_FALSE(face_a == nullptr);
  ASSERT_FALSE(face_b == nullptr);
  EXPECT_EQ(face_a->ns(), "/humans/faces/A");
  ASSERT_TRUE(face_a->roi());
  EXPECT_FLOAT_EQ(face_a->roi().value().x, 0.2f);
  EXPECT_EQ(face_b->ns(), "/humans/faces/B");
  ASSERT_TRUE(face_b->roi());
  EXPECT_FLOAT_EQ(face_b->roi().value().x, 0.2f);
}

TEST_F(HRITest, GetBodies)
{
  auto bodies_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 1);
  auto ids_msg = hri_msgs::msg::IdsList();

  ASSERT_EQ(bodies_pub->get_subscription_count(), 1U);
  EXPECT_EQ(hri_listener_->getBodies().size(), 0U);

  ids_msg.ids = {"A"};
  bodies_pub->publish(ids_msg);
  spin();
  auto bodies = hri_listener_->getBodies();
  EXPECT_EQ(bodies.size(), 1U);
  ASSERT_TRUE(bodies.count("A"));
  EXPECT_EQ(bodies["A"]->id(), "A");

  bodies_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getBodies().size(), 1U);

  ids_msg.ids = {"A", "B"};
  bodies_pub->publish(ids_msg);
  spin();
  bodies = hri_listener_->getBodies();
  EXPECT_EQ(bodies.size(), 2U);
  EXPECT_TRUE(bodies.count("A"));
  EXPECT_TRUE(bodies.count("B"));

  ids_msg.ids = {"B"};
  bodies_pub->publish(ids_msg);
  spin();
  bodies = hri_listener_->getBodies();
  EXPECT_EQ(bodies.size(), 1U);
  EXPECT_FALSE(bodies.count("A"));
  ASSERT_TRUE(bodies.count("B"));

  ids_msg.ids = {};
  bodies_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getBodies().size(), 0U);
  // check body B is not used anymore by hri_listener_!
  EXPECT_EQ(bodies["B"].use_count(), 1U);

  hri_listener_.reset();
  EXPECT_EQ(bodies_pub->get_subscription_count(), 0U);
  EXPECT_FALSE(bodies["B"]->valid());
}

TEST_F(HRITest, GetVoices)
{
  auto voices_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", 1);
  auto ids_msg = hri_msgs::msg::IdsList();

  ASSERT_EQ(voices_pub->get_subscription_count(), 1U);
  EXPECT_EQ(hri_listener_->getVoices().size(), 0U);

  ids_msg.ids = {"A"};
  voices_pub->publish(ids_msg);
  spin();
  auto voices = hri_listener_->getVoices();
  EXPECT_EQ(voices.size(), 1U);
  ASSERT_TRUE(voices.count("A"));
  EXPECT_EQ(voices["A"]->id(), "A");

  voices_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getVoices().size(), 1U);

  ids_msg.ids = {"A", "B"};
  voices_pub->publish(ids_msg);
  spin();
  voices = hri_listener_->getVoices();
  EXPECT_EQ(voices.size(), 2U);
  EXPECT_TRUE(voices.count("A"));
  EXPECT_TRUE(voices.count("B"));

  ids_msg.ids = {"B"};
  voices_pub->publish(ids_msg);
  spin();
  voices = hri_listener_->getVoices();
  EXPECT_EQ(voices.size(), 1U);
  EXPECT_FALSE(voices.count("A"));
  ASSERT_TRUE(voices.count("B"));

  ids_msg.ids = {};
  voices_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getVoices().size(), 0U);
  // check face B is not used anymore by hri_listener_!
  EXPECT_EQ(voices["B"].use_count(), 1U);

  hri_listener_.reset();
  EXPECT_EQ(voices_pub->get_subscription_count(), 0U);
  EXPECT_FALSE(voices["B"]->valid());
}

TEST_F(HRITest, GetVoiceCallbacks)
{
  auto voices_pub =
    tester_node_->create_publisher<hri_msgs::msg::IdsList>("/humans/voices/tracked", 1);
  auto voice_a_is_speaking_pub =
    tester_node_->create_publisher<std_msgs::msg::Bool>("/humans/voices/A/is_speaking", 1);
  auto voice_a_speech_pub =
    tester_node_->create_publisher<hri_msgs::msg::LiveSpeech>("/humans/voices/A/speech", 1);
  auto ids_msg = hri_msgs::msg::IdsList();
  auto is_speaking_msg = std_msgs::msg::Bool();
  auto speech_msg = hri_msgs::msg::LiveSpeech();

  auto cb_triggered = false;
  hri_listener_->onVoice(
    [&](hri::VoicePtr voice) {
      cb_triggered = true;
      voice->onSpeaking([&]([[maybe_unused]] bool speaking) {cb_triggered = true;});
      voice->onIncrementalSpeech(
        [&]([[maybe_unused]] const std::string & speech) {cb_triggered = true;});
      voice->onSpeech([&]([[maybe_unused]] const std::string & speech) {cb_triggered = true;});
    });

  cb_triggered = false;
  ids_msg.ids = {"A"};
  voices_pub->publish(ids_msg);
  spin();
  EXPECT_TRUE(cb_triggered);

  cb_triggered = false;
  is_speaking_msg.data = true;
  voice_a_is_speaking_pub->publish(is_speaking_msg);
  spin();
  EXPECT_TRUE(cb_triggered);
  EXPECT_TRUE(hri_listener_->getVoices()["A"]->isSpeaking().value());

  cb_triggered = false;
  is_speaking_msg.data = false;
  voice_a_is_speaking_pub->publish(is_speaking_msg);
  spin();
  EXPECT_TRUE(cb_triggered);
  EXPECT_FALSE(hri_listener_->getVoices()["A"]->isSpeaking().value());

  cb_triggered = false;
  speech_msg.final = "test speech";
  voice_a_speech_pub->publish(speech_msg);
  spin();
  EXPECT_TRUE(cb_triggered);
  EXPECT_EQ(hri_listener_->getVoices()["A"]->speech().value(), "test speech");

  cb_triggered = false;
  speech_msg.incremental = "test speech incremental";
  voice_a_speech_pub->publish(speech_msg);
  spin();
  EXPECT_TRUE(cb_triggered);
  EXPECT_EQ(
    hri_listener_->getVoices()["A"]->incrementalSpeech().value(), "test speech incremental");
}

TEST_F(HRITest, GetKnownPersons)
{
  auto persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/known", 1);
  auto ids_msg = hri_msgs::msg::IdsList();

  ASSERT_EQ(persons_pub->get_subscription_count(), 1U);
  EXPECT_EQ(hri_listener_->getPersons().size(), 0U);

  ids_msg.ids = {"A"};
  persons_pub->publish(ids_msg);
  spin();
  auto persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 1U);
  ASSERT_TRUE(persons.count("A"));
  EXPECT_EQ(persons["A"]->id(), "A");

  persons_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 1U);

  ids_msg.ids = {"A", "B"};
  persons_pub->publish(ids_msg);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 2U);
  EXPECT_TRUE(persons.count("A"));
  EXPECT_TRUE(persons.count("B"));

  ids_msg.ids = {"B"};
  persons_pub->publish(ids_msg);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 1U);
  EXPECT_FALSE(persons.count("A"));
  ASSERT_TRUE(persons.count("B"));

  ids_msg.ids = {};
  persons_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 0U);
  // check face B is not used anymore by hri_listener_!
  EXPECT_EQ(persons["B"].use_count(), 1U);

  hri_listener_.reset();
  EXPECT_EQ(persons_pub->get_subscription_count(), 0U);
  EXPECT_FALSE(persons["B"]->valid());
}

TEST_F(HRITest, GetTrackedPersons)
{
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto ids_msg = hri_msgs::msg::IdsList();

  ASSERT_EQ(tracked_persons_pub->get_subscription_count(), 1U);
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 0U);

  ids_msg.ids = {"A"};
  tracked_persons_pub->publish(ids_msg);
  spin();
  auto persons = hri_listener_->getTrackedPersons();
  EXPECT_EQ(persons.size(), 1U);
  ASSERT_TRUE(persons.count("A"));
  EXPECT_EQ(persons["A"]->id(), "A");

  tracked_persons_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 1U);

  ids_msg.ids = {"A", "B"};
  tracked_persons_pub->publish(ids_msg);
  spin();
  persons = hri_listener_->getTrackedPersons();
  EXPECT_EQ(persons.size(), 2U);
  EXPECT_TRUE(persons.count("A"));
  EXPECT_TRUE(persons.count("B"));

  ids_msg.ids = {"B"};
  tracked_persons_pub->publish(ids_msg);
  spin();
  persons = hri_listener_->getTrackedPersons();
  EXPECT_EQ(persons.size(), 1U);
  EXPECT_FALSE(persons.count("A"));
  ASSERT_TRUE(persons.count("B"));

  ids_msg.ids = {};
  tracked_persons_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 0U);
  // check face B is not used anymore by hri_listener_!
  EXPECT_EQ(persons["B"].use_count(), 1U);

  hri_listener_.reset();
  EXPECT_EQ(tracked_persons_pub->get_subscription_count(), 0U);
  EXPECT_FALSE(persons["B"]->valid());
}

TEST_F(HRITest, PersonAttributes)
{
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto p1_face_pub = tester_node_->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", kQoSLatched_);
  auto person_ids_msg = hri_msgs::msg::IdsList();
  auto face_ids_msg = hri_msgs::msg::IdsList();
  auto face_id_msg = std_msgs::msg::String();

  person_ids_msg.ids = {"p1"};
  tracked_persons_pub->publish(person_ids_msg);
  face_ids_msg.ids = {"f1", "f2"};
  faces_pub->publish(face_ids_msg);
  spin();
  auto p1 = hri_listener_->getTrackedPersons()["p1"];
  EXPECT_FALSE(p1->anonymous()) << "By default, persons are not supposed to be anonymous";
  EXPECT_EQ(p1->face(), nullptr);

  face_id_msg.data = "f1";
  p1_face_pub->publish(face_id_msg);
  spin();
  auto f1 = p1->face();
  ASSERT_NE(f1, nullptr);
  EXPECT_EQ(f1->id(), "f1");
}

TEST_F(HRITest, AnonymousPersonsAndAliases)
{
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "humans/persons/tracked", 1);
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto p1_anon_pub = tester_node_->create_publisher<std_msgs::msg::Bool>(
    "/humans/persons/p1/anonymous", kQoSLatched_);
  auto p2_anon_pub = tester_node_->create_publisher<std_msgs::msg::Bool>(
    "/humans/persons/p2/anonymous", kQoSLatched_);
  auto p1_face_pub = tester_node_->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", kQoSLatched_);
  auto p2_face_pub = tester_node_->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p2/face_id", kQoSLatched_);
  auto p2_alias_pub = tester_node_->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p2/alias", kQoSLatched_);
  auto person_ids_msg = hri_msgs::msg::IdsList();
  auto face_ids_msg = hri_msgs::msg::IdsList();
  auto face_id_msg = std_msgs::msg::String();
  auto anon_msg = std_msgs::msg::Bool();
  auto alias_msg = std_msgs::msg::String();

  person_ids_msg.ids = {"p1", "p2"};
  tracked_persons_pub->publish(person_ids_msg);
  face_ids_msg.ids = {"f1", "f2"};
  faces_pub->publish(face_ids_msg);
  spin();
  ASSERT_EQ(hri_listener_->getTrackedPersons().size(), 2U);
  ASSERT_EQ(hri_listener_->getFaces().size(), 2U);

  // each person is associated to a face
  face_id_msg.data = "f1";
  p1_face_pub->publish(face_id_msg);
  face_id_msg.data = "f2";
  p2_face_pub->publish(face_id_msg);
  anon_msg.data = false;
  p1_anon_pub->publish(anon_msg);
  anon_msg.data = true;
  p2_anon_pub->publish(anon_msg);
  spin();
  auto p1 = hri_listener_->getTrackedPersons()["p1"];
  auto p2 = hri_listener_->getTrackedPersons()["p2"];
  ASSERT_TRUE(p1->anonymous());
  ASSERT_TRUE(p2->anonymous());
  ASSERT_FALSE(p1->anonymous().value());
  ASSERT_TRUE(p2->anonymous().value());
  // being anonymous or not should have no impact on face associations
  ASSERT_EQ(p1->face()->id(), "f1");
  ASSERT_EQ(p2->face()->id(), "f2");

  // ALIASES

  // set p2 as an alias of p1
  alias_msg.data = "p1";
  p2_alias_pub->publish(alias_msg);
  spin();
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 2U);
  p2 = hri_listener_->getTrackedPersons()["p2"];
  EXPECT_EQ(p1, p2) << "p2 should now point to the same person as p1";
  EXPECT_EQ(p2->face()->id(), "f1") << "p2's face now points to f1";

  // remove the alias
  alias_msg.data = "";
  p2_alias_pub->publish(alias_msg);
  spin();
  p2 = hri_listener_->getTrackedPersons()["p2"];
  EXPECT_NE(p1, p2) << "p2 is not anymore the same person as p1";
  EXPECT_EQ(p2->face()->id(), "f2") << "p2's face should still points to its former f2 face";

  // republish the alias
  alias_msg.data = "p1";
  p2_alias_pub->publish(alias_msg);
  spin();
  p2 = hri_listener_->getTrackedPersons()["p2"];
  EXPECT_EQ(p1, p2) << "p2 is again the same person as p1";

  // delete p1 -> p2 should be deleted as well
  person_ids_msg.ids = {"p2"};
  tracked_persons_pub->publish(person_ids_msg);
  spin();
  ASSERT_EQ(hri_listener_->getTrackedPersons().size(), 0U)
    << "The aliased person should have been deleted with its alias";
}

TEST_F(HRITest, SoftBiometrics)
{
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto p1_face_pub = tester_node_->create_publisher<std_msgs::msg::String>(
    "/humans/persons/p1/face_id", kQoSLatched_);
  auto softbiometrics_pub = tester_node_->create_publisher<hri_msgs::msg::SoftBiometrics>(
    "/humans/faces/f1/softbiometrics", 1);
  auto person_ids_msg = hri_msgs::msg::IdsList();
  auto face_ids_msg = hri_msgs::msg::IdsList();
  auto softbiometrics_msg = hri_msgs::msg::SoftBiometrics();
  auto face_id_msg = std_msgs::msg::String();

  person_ids_msg.ids = {"p1"};
  tracked_persons_pub->publish(person_ids_msg);
  face_ids_msg.ids = {"f1"};
  faces_pub->publish(face_ids_msg);
  spin();
  ASSERT_EQ(hri_listener_->getTrackedPersons().size(), 1U);
  ASSERT_EQ(hri_listener_->getFaces().size(), 1U);

  softbiometrics_msg.age = 45;
  softbiometrics_msg.age_confidence = 0.8;
  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::FEMALE;
  softbiometrics_msg.gender_confidence = 0.7;
  softbiometrics_pub->publish(softbiometrics_msg);
  face_id_msg.data = "f1";
  p1_face_pub->publish(face_id_msg);
  spin();
  auto face = hri_listener_->getTrackedPersons()["p1"]->face();
  EXPECT_EQ(face->id(), "f1");
  ASSERT_TRUE(face->age());
  EXPECT_FLOAT_EQ(face->age().value(), 45.f);
  ASSERT_TRUE(face->gender());
  EXPECT_EQ(face->gender().value(), hri::Gender::kFemale);

  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::OTHER;
  softbiometrics_pub->publish(softbiometrics_msg);
  spin();
  EXPECT_EQ(face->gender().value(), hri::Gender::kOther);

  softbiometrics_msg.gender = hri_msgs::msg::SoftBiometrics::UNDEFINED;
  softbiometrics_pub->publish(softbiometrics_msg);
  spin();
  EXPECT_FALSE(face->gender());
}

TEST_F(HRITest, EngagementLevel)
{
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto engagement_pub = tester_node_->create_publisher<hri_msgs::msg::EngagementLevel>(
    "/humans/persons/p1/engagement_status", 1);
  auto person_ids_msg = hri_msgs::msg::IdsList();
  auto msg = hri_msgs::msg::EngagementLevel();

  person_ids_msg.ids = {"p1"};
  tracked_persons_pub->publish(person_ids_msg);
  spin();
  auto p1 = hri_listener_->getTrackedPersons()["p1"];
  msg.level = hri_msgs::msg::EngagementLevel::DISENGAGED;
  engagement_pub->publish(msg);
  spin();
  ASSERT_TRUE(p1->engagementStatus());
  EXPECT_EQ(p1->engagementStatus().value(), hri::EngagementLevel::kDisengaged);

  msg.level = hri_msgs::msg::EngagementLevel::ENGAGED;
  engagement_pub->publish(msg);
  spin();
  EXPECT_EQ(p1->engagementStatus().value(), hri::EngagementLevel::kEngaged);

  msg.level = hri_msgs::msg::EngagementLevel::UNKNOWN;
  engagement_pub->publish(msg);
  spin();
  EXPECT_FALSE(p1->engagementStatus());
}

TEST_F(HRITest, Image)
{
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto face_cropped_a_pub = tester_node_->create_publisher<sensor_msgs::msg::Image>(
    "/humans/faces/A/cropped", 1);
  auto ids_msg = hri_msgs::msg::IdsList();
  auto header = std_msgs::msg::Header();
  cv::Mat image(64, 64, CV_8UC3);
  cv::RNG image_rng;
  image_rng.fill(image, cv::RNG::UNIFORM, 0, 255);
  cv_bridge::CvImage cv_bridge(header, "bgr8", image);
  auto image_msg = cv_bridge.toImageMsg();

  ids_msg.ids = {"A"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(face_cropped_a_pub->get_subscription_count(), 1U);
  auto face_a = hri_listener_->getFaces()["A"];
  EXPECT_FALSE(face_a->cropped());

  face_cropped_a_pub->publish(*image_msg);
  spin();
  ASSERT_TRUE(face_a->cropped());
  EXPECT_DOUBLE_EQ(cv::norm(face_a->cropped().value(), image), 0.);
}

TEST_F(HRITest, FacialActionUnits)
{
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto fau_a_pub = tester_node_->create_publisher<hri_msgs::msg::FacialActionUnits>(
    "/humans/faces/A/facs", 1);
  auto ids_msg = hri_msgs::msg::IdsList();
  auto fau_msg = hri_msgs::msg::FacialActionUnits();
  auto fau = hri::IntensityConfidence();

  ids_msg.ids = {"A"};
  faces_pub->publish(ids_msg);
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(fau_a_pub->get_subscription_count(), 1U);
  auto face_a = hri_listener_->getFaces()["A"];
  EXPECT_FALSE(face_a->facialActionUnits());

  fau_msg.intensity[fau_msg.WINK] = 0.5;
  fau_msg.confidence[fau_msg.WINK] = 0.8;
  fau_a_pub->publish(fau_msg);
  spin();
  ASSERT_TRUE(face_a->facialActionUnits());
  fau = (*face_a->facialActionUnits())[hri::FacialActionUnit::kWink];
  EXPECT_FLOAT_EQ(fau.intensity, fau_msg.intensity[fau_msg.WINK]);
  EXPECT_FLOAT_EQ(fau.confidence, fau_msg.confidence[fau_msg.WINK]);

  fau_msg.intensity[fau_msg.WINK] = 0.0;
  fau_msg.confidence[fau_msg.WINK] = 1.0;
  fau_a_pub->publish(fau_msg);
  spin();
  fau = (*face_a->facialActionUnits())[hri::FacialActionUnit::kWink];
  EXPECT_FLOAT_EQ(fau.intensity, fau_msg.intensity[fau_msg.WINK]);
  EXPECT_FLOAT_EQ(fau.confidence, fau_msg.confidence[fau_msg.WINK]);
}

TEST_F(HRITest, FacialLandmarks)
{
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto facial_landmarks_a_pub = tester_node_->create_publisher<hri_msgs::msg::FacialLandmarks>(
    "/humans/faces/A/landmarks", 1);
  auto ids_msg = hri_msgs::msg::IdsList();
  auto facial_landmarks_msg = hri_msgs::msg::FacialLandmarks();
  auto point = hri::PointOfInterest();

  ids_msg.ids = {"A"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(facial_landmarks_a_pub->get_subscription_count(), 1U);
  auto face_a = hri_listener_->getFaces()["A"];
  EXPECT_FALSE(face_a->facialLandmarks());

  facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x = 0.3;
  facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].y = 0.5;
  facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c = 0.8;
  facial_landmarks_a_pub->publish(facial_landmarks_msg);
  spin();
  ASSERT_TRUE(face_a->facialLandmarks());
  point = (*face_a->facialLandmarks())[hri::FacialLandmark::kNose];
  EXPECT_FLOAT_EQ(point.x, facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x);
  EXPECT_FLOAT_EQ(point.y, facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].y);
  EXPECT_FLOAT_EQ(point.c, facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c);

  facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x = 1.0;
  facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c = 0.0;
  facial_landmarks_a_pub->publish(facial_landmarks_msg);
  spin();
  point = (*face_a->facialLandmarks())[hri::FacialLandmark::kNose];
  EXPECT_FLOAT_EQ(point.x, facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].x);
  EXPECT_FLOAT_EQ(point.y, facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].y);
  EXPECT_FLOAT_EQ(point.c, facial_landmarks_msg.landmarks[facial_landmarks_msg.NOSE].c);
}

TEST_F(HRITest, SkeletalKeypoints)
{
  auto bodies_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 1);
  auto body_keypoints_a_pub = tester_node_->create_publisher<hri_msgs::msg::Skeleton2D>(
    "/humans/bodies/A/skeleton2d", 1);
  auto ids_msg = hri_msgs::msg::IdsList();
  auto skeleton_msg = hri_msgs::msg::Skeleton2D();
  auto point = hri::PointOfInterest();

  ids_msg.ids = {"A"};
  bodies_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(body_keypoints_a_pub->get_subscription_count(), 1U);
  auto body_a = hri_listener_->getBodies()["A"];
  EXPECT_FALSE(body_a->skeleton());

  skeleton_msg.skeleton[skeleton_msg.NOSE].x = 0.3;
  skeleton_msg.skeleton[skeleton_msg.NOSE].y = 0.5;
  skeleton_msg.skeleton[skeleton_msg.NOSE].c = 0.8;
  body_keypoints_a_pub->publish(skeleton_msg);
  spin();
  ASSERT_TRUE(body_a->skeleton());
  point = (*body_a->skeleton())[hri::SkeletalKeypoint::kNose];
  EXPECT_FLOAT_EQ(point.x, skeleton_msg.skeleton[skeleton_msg.NOSE].x);
  EXPECT_FLOAT_EQ(point.y, skeleton_msg.skeleton[skeleton_msg.NOSE].y);
  EXPECT_FLOAT_EQ(point.c, skeleton_msg.skeleton[skeleton_msg.NOSE].c);

  skeleton_msg.skeleton[skeleton_msg.NOSE].x = 1.0;
  skeleton_msg.skeleton[skeleton_msg.NOSE].c = 0.0;
  body_keypoints_a_pub->publish(skeleton_msg);
  spin();
  point = (*body_a->skeleton())[hri::SkeletalKeypoint::kNose];
  EXPECT_FLOAT_EQ(point.x, skeleton_msg.skeleton[skeleton_msg.NOSE].x);
  EXPECT_FLOAT_EQ(point.y, skeleton_msg.skeleton[skeleton_msg.NOSE].y);
  EXPECT_FLOAT_EQ(point.c, skeleton_msg.skeleton[skeleton_msg.NOSE].c);
}

TEST_F(HRITest, Callback)
{
  auto faces_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/faces/tracked", 1);
  auto body_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 1);
  auto voice_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/voices/tracked", 1);
  auto person_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/known", 1);
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto ids_msg = hri_msgs::msg::IdsList();

  int face_cb_invoked = 0;
  int face_lost_cb_invoked = 0;
  int body_cb_invoked = 0;
  int body_lost_cb_invoked = 0;
  int voice_cb_invoked = 0;
  int voice_lost_cb_invoked = 0;
  int person_cb_invoked = 0;
  int person_lost_cb_invoked = 0;
  int tracked_person_cb_invoked = 0;
  int tracked_person_lost_cb_invoked = 0;
  int old_face_cb_invoked;
  int old_face_lost_cb_invoked;
  int old_body_cb_invoked;
  int old_body_lost_cb_invoked;
  int old_voice_cb_invoked;
  int old_voice_lost_cb_invoked;
  int old_person_cb_invoked;
  int old_person_lost_cb_invoked;
  int old_tracked_person_cb_invoked;
  int old_tracked_person_lost_cb_invoked;

  hri_listener_->onFace([&]([[maybe_unused]] hri::FacePtr face) {++face_cb_invoked;});
  hri_listener_->onFaceLost([&]([[maybe_unused]] hri::ID face_lost) {++face_lost_cb_invoked;});
  hri_listener_->onBody([&]([[maybe_unused]] hri::BodyPtr body) {++body_cb_invoked;});
  hri_listener_->onBodyLost([&]([[maybe_unused]] hri::ID body_lost) {++body_lost_cb_invoked;});
  hri_listener_->onVoice([&]([[maybe_unused]] hri::VoicePtr voice) {++voice_cb_invoked;});
  hri_listener_->onVoiceLost([&]([[maybe_unused]] hri::ID voice_lost) {++voice_lost_cb_invoked;});
  hri_listener_->onPerson([&]([[maybe_unused]] hri::PersonPtr person) {++person_cb_invoked;});
  hri_listener_->onPersonLost([&]([[maybe_unused]] hri::ID person) {++person_lost_cb_invoked;});
  hri_listener_->onTrackedPerson(
    [&]([[maybe_unused]] hri::PersonPtr tracked_person) {++tracked_person_cb_invoked;});
  hri_listener_->onTrackedPersonLost(
    [&]([[maybe_unused]] hri::ID tracked_person_lost) {++tracked_person_lost_cb_invoked;});

  old_face_cb_invoked = face_cb_invoked;
  old_face_lost_cb_invoked = face_lost_cb_invoked;
  ids_msg.ids = {"id1"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(face_cb_invoked, old_face_cb_invoked + 1);
  EXPECT_EQ(face_lost_cb_invoked, old_face_lost_cb_invoked);

  old_face_cb_invoked = face_cb_invoked;
  ids_msg.ids = {"id1", "id2"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(face_cb_invoked, old_face_cb_invoked + 1);

  old_face_cb_invoked = face_cb_invoked;
  old_face_lost_cb_invoked = face_lost_cb_invoked;
  ids_msg.ids = {"id3", "id4"};
  faces_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(face_cb_invoked, old_face_cb_invoked + 2);
  EXPECT_EQ(face_lost_cb_invoked, old_face_lost_cb_invoked + 2);

  old_body_cb_invoked = body_cb_invoked;
  old_body_lost_cb_invoked = body_lost_cb_invoked;
  ids_msg.ids = {"id1", "id2"};
  body_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(body_cb_invoked, old_body_cb_invoked + 2);
  EXPECT_EQ(body_lost_cb_invoked, old_body_lost_cb_invoked);

  old_face_cb_invoked = face_cb_invoked;
  old_face_lost_cb_invoked = face_lost_cb_invoked;
  old_body_cb_invoked = body_cb_invoked;
  old_body_lost_cb_invoked = body_lost_cb_invoked;
  ids_msg.ids = {"id1", "id2", "id3"};
  faces_pub->publish(ids_msg);
  body_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(face_cb_invoked, old_face_cb_invoked + 2);
  EXPECT_EQ(face_lost_cb_invoked, old_face_lost_cb_invoked + 1);
  EXPECT_EQ(body_cb_invoked, old_body_cb_invoked + 1);
  EXPECT_EQ(body_lost_cb_invoked, old_body_lost_cb_invoked);

  old_face_cb_invoked = face_cb_invoked;
  old_face_lost_cb_invoked = face_lost_cb_invoked;
  old_body_cb_invoked = body_cb_invoked;
  old_body_lost_cb_invoked = body_lost_cb_invoked;
  ids_msg.ids = {"id5", "id6", "id7"};
  faces_pub->publish(ids_msg);
  body_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(face_cb_invoked, old_face_cb_invoked + 3);
  EXPECT_EQ(face_lost_cb_invoked, old_face_lost_cb_invoked + 3);
  EXPECT_EQ(body_cb_invoked, old_body_cb_invoked + 3);
  EXPECT_EQ(body_lost_cb_invoked, old_body_lost_cb_invoked + 3);

  old_voice_cb_invoked = voice_cb_invoked;
  old_voice_lost_cb_invoked = voice_lost_cb_invoked;
  old_person_cb_invoked = person_cb_invoked;
  old_person_lost_cb_invoked = person_lost_cb_invoked;
  old_tracked_person_cb_invoked = tracked_person_cb_invoked;
  old_tracked_person_lost_cb_invoked = tracked_person_lost_cb_invoked;
  ids_msg.ids = {"id1", "id2", "id3"};
  voice_pub->publish(ids_msg);
  ids_msg.ids = {"id1", "id2"};
  person_pub->publish(ids_msg);
  ids_msg.ids = {"id1"};
  tracked_persons_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(voice_cb_invoked, old_voice_cb_invoked + 3);
  EXPECT_EQ(voice_lost_cb_invoked, old_voice_lost_cb_invoked);
  EXPECT_EQ(person_cb_invoked, old_person_cb_invoked + 2);
  EXPECT_EQ(person_lost_cb_invoked, old_person_lost_cb_invoked);
  EXPECT_EQ(tracked_person_cb_invoked, old_tracked_person_cb_invoked + 1);
  EXPECT_EQ(tracked_person_lost_cb_invoked, old_tracked_person_lost_cb_invoked);

  old_voice_cb_invoked = voice_cb_invoked;
  old_voice_lost_cb_invoked = voice_lost_cb_invoked;
  old_person_cb_invoked = person_cb_invoked;
  old_person_lost_cb_invoked = person_lost_cb_invoked;
  old_tracked_person_cb_invoked = tracked_person_cb_invoked;
  old_tracked_person_lost_cb_invoked = tracked_person_lost_cb_invoked;
  ids_msg.ids = {};
  voice_pub->publish(ids_msg);
  person_pub->publish(ids_msg);
  tracked_persons_pub->publish(ids_msg);
  spin();
  EXPECT_EQ(voice_cb_invoked, old_voice_cb_invoked);
  EXPECT_EQ(voice_lost_cb_invoked, old_voice_lost_cb_invoked + 3);
  EXPECT_EQ(person_cb_invoked, old_person_cb_invoked);
  EXPECT_EQ(person_lost_cb_invoked, old_person_lost_cb_invoked + 2);
  EXPECT_EQ(tracked_person_cb_invoked, old_tracked_person_cb_invoked);
  EXPECT_EQ(tracked_person_lost_cb_invoked, old_tracked_person_lost_cb_invoked + 1);
}

TEST_F(HRITest, PeopleLocation)
{
  auto tracked_persons_pub = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", 1);
  auto location_confidence_pub = tester_node_->create_publisher<std_msgs::msg::Float32>(
    "/humans/persons/p1/location_confidence", 1);
  auto tester_executor = rclcpp::executors::SingleThreadedExecutor();
  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(tester_node_);
  auto person_ids_msg = hri_msgs::msg::IdsList();
  auto transform_msg = geometry_msgs::msg::TransformStamped();
  auto loc_confindence_msg = std_msgs::msg::Float32();

  hri_listener_->setReferenceFrame("base_link");
  transform_msg.header.stamp = tester_node_->now();
  transform_msg.header.frame_id = "world";
  transform_msg.child_frame_id = "base_link";
  transform_msg.transform.translation.x = -1.0;
  transform_msg.transform.translation.y = 0.0;
  transform_msg.transform.translation.z = 0.0;
  transform_msg.transform.rotation.w = 1.0;
  static_broadcaster->sendTransform(transform_msg);
  tester_executor.spin_node_once(tester_node_);
  spin();

  person_ids_msg.ids = {"p1"};
  tracked_persons_pub->publish(person_ids_msg);
  spin();
  auto p1 = hri_listener_->getTrackedPersons()["p1"];

  loc_confindence_msg.data = 0.;
  location_confidence_pub->publish(loc_confindence_msg);
  spin();
  ASSERT_TRUE(p1->locationConfidence());
  EXPECT_FLOAT_EQ(p1->locationConfidence().value(), 0.f);
  EXPECT_FALSE(p1->transform()) << "location confidence at 0, no transform should be available";

  loc_confindence_msg.data = 0.5;
  location_confidence_pub->publish(loc_confindence_msg);
  spin();
  EXPECT_FLOAT_EQ(p1->locationConfidence().value(), 0.5f);
  EXPECT_FALSE(p1->transform())
    << "location confidence > 0 but no transform published yet -> no transform should be returned";

  transform_msg.child_frame_id = "person_p1";
  transform_msg.transform.translation.x += 2.0;
  static_broadcaster->sendTransform(transform_msg);
  tester_executor.spin_node_once(tester_node_);
  spin();
  EXPECT_FLOAT_EQ(p1->locationConfidence().value(), 0.5f);
  ASSERT_TRUE(p1->transform()) << "location confidence > 0 => a transform should be available";
  auto t = p1->transform().value();
  EXPECT_EQ(t.child_frame_id, "person_p1");
  EXPECT_EQ(t.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(t.transform.translation.x, 2.0f);

  hri_listener_->setReferenceFrame("person_p1");
  ASSERT_TRUE(p1->transform());
  t = p1->transform().value();
  EXPECT_EQ(t.child_frame_id, "person_p1");
  EXPECT_EQ(t.header.frame_id, "person_p1");
  EXPECT_FLOAT_EQ(t.transform.translation.x, 0.f);

  loc_confindence_msg.data = 1.0;
  location_confidence_pub->publish(loc_confindence_msg);
  spin();
  EXPECT_FLOAT_EQ(p1->locationConfidence().value(), 1.f);
  EXPECT_TRUE(p1->transform()) << "location confidence > 0 => a transform should be available";
}

// TODO(LJU): missing quite a few tests, should have at least a basic one for each topic subscribed

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
