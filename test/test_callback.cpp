// // Copyright 2021 PAL Robotics S.L.
// //
// // Redistribution and use in source and binary forms, with or without
// // modification, are permitted provided that the following conditions are met:
// //
// //    * Redistributions of source code must retain the above copyright
// //      notice, this list of conditions and the following disclaimer.
// //
// //    * Redistributions in binary form must reproduce the above copyright
// //      notice, this list of conditions and the following disclaimer in the
// //      documentation and/or other materials provided with the distribution.
// //
// //    * Neither the name of the PAL Robotics S.L. nor the names of its
// //      contributors may be used to endorse or promote products derived from
// //      this software without specific prior written permission.
// //
// // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// // POSSIBILITY OF SUCH DAMAGE.


#include "gmock/gmock.h"
#include <hri/hri.hpp>
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <chrono>
#include <memory>
#include "hri/face.hpp"
#include "hri/person.hpp"
#include "hri_msgs/msg/engagement_level.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/soft_biometrics.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>


using namespace std::chrono_literals;



TEST(libhri, Callbacks1)
{
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::Rate rate(30);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  auto hri_listener = std::make_shared<hri::HRIListener>();

  int num_callbacks_invoked = 0;
  // create mock callbacks
  testing::MockFunction<void(hri::FaceWeakConstPtr)> face_callback;
  hri_listener->onFace(face_callback.AsStdFunction());

  testing::MockFunction<void(hri::ID)> face_lost_callback;
  hri_listener->onFaceLost(face_lost_callback.AsStdFunction());

  testing::MockFunction<void(hri::BodyWeakConstPtr)> body_callback;
  hri_listener->onBody(body_callback.AsStdFunction());

  testing::MockFunction<void(hri::ID)> body_lost_callback;
  hri_listener->onBodyLost(body_lost_callback.AsStdFunction());


  testing::MockFunction<void(hri::VoiceWeakConstPtr)> voice_callback;
  hri_listener->onVoice(voice_callback.AsStdFunction());

  testing::MockFunction<void(hri::ID)> voice_lost_callback;
  hri_listener->onVoiceLost(voice_lost_callback.AsStdFunction());


  testing::MockFunction<void(hri::PersonWeakConstPtr)> person_callback;
  hri_listener->onPerson(person_callback.AsStdFunction());

  testing::MockFunction<void(hri::PersonWeakConstPtr)> person_tracked_callback;
  hri_listener->onTrackedPerson(person_tracked_callback.AsStdFunction());

  testing::MockFunction<void(hri::ID)> person_tracked_lost_callback;
  hri_listener->onTrackedPersonLost(person_tracked_lost_callback.AsStdFunction());



  auto ids = hri_msgs::msg::IdsList();

  auto face_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/faces/tracked", 1);
  auto body_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/bodies/tracked", 1);
  auto voice_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/voices/tracked", 1);
  auto person_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/known", 1);
  auto person_tracked_pub = node->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/tracked", 1);


  EXPECT_CALL(face_callback, Call(testing::_)).Times(0);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(0);

  ids.ids = { "id1" };

  face_pub->publish(ids);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));

  EXPECT_CALL(face_callback, Call(testing::_)).Times(1);
  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(0);

  ids.ids = { "id1", "id2" };
  face_pub->publish(ids);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));


  EXPECT_CALL(face_callback, Call(testing::_)).Times(2);

  ids.ids = { "id3", "id4" };
  face_pub->publish(ids);
  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(30));


  EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(2);
    
// //   WAIT;

// //   EXPECT_CALL(body_callback, Call(testing::_)).Times(2);
// //   EXPECT_CALL(body_lost_callback, Call(testing::_)).Times(0);
// //   ids.ids = { "id1", "id2" };
// //   body_pub.publish(ids);

// //   WAIT;

// //   EXPECT_CALL(face_callback, Call(testing::_)).Times(2);
// //   EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(1);
// //   EXPECT_CALL(body_callback, Call(testing::_)).Times(1);
// //   EXPECT_CALL(body_lost_callback, Call(testing::_)).Times(0);
// //   ids.ids = { "id1", "id2", "id3" };
// //   face_pub.publish(ids);
// //   body_pub.publish(ids);

// //   WAIT;

// //   EXPECT_CALL(face_callback, Call(testing::_)).Times(3);
// //   EXPECT_CALL(face_lost_callback, Call(testing::_)).Times(3);
// //   EXPECT_CALL(body_callback, Call(testing::_)).Times(3);
// //   EXPECT_CALL(body_lost_callback, Call(testing::_)).Times(3);
// //   ids.ids = { "id5", "id6", "id7" };
// //   face_pub.publish(ids);
// //   body_pub.publish(ids);

// //   WAIT;

// //   EXPECT_CALL(voice_callback, Call(testing::_)).Times(2);
// //   EXPECT_CALL(person_callback, Call(testing::_)).Times(2);
// //   EXPECT_CALL(person_tracked_callback, Call(testing::_)).Times(2);
// //   ids.ids = { "id1", "id2" };
// //   voice_pub.publish(ids);
// //   person_pub.publish(ids);
// //   person_tracked_pub.publish(ids);

// //   WAIT;

// //   spinner.stop();

  hri_listener.reset();
  executor.remove_node(node);
  executor.cancel();
}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  // ros::Time::init();  // needed for ros::Time::now()
  // ros::init(argc, argv, "test_hri");
  // ros::NodeHandle nh;
  // ROS_INFO("Starting HRI tests");
  return RUN_ALL_TESTS();
}

