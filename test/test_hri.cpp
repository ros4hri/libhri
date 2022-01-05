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
#include <hri/hri.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include "hri_msgs/IdsList.h"

using namespace ros;
using namespace hri;

// waiting time for the libhri callback to process their inputs
#define WAIT std::this_thread::sleep_for(std::chrono::milliseconds(5))

TEST(libhri, GetFaces)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);

    EXPECT_EQ(pub.getNumSubscribers(), 1U);


    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 1U);

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 2U);

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 1U);

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 0U);
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

// TEST(hri, ListHumans2)
//{
//  EXPECT_TRUE(false);
//}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "hri_unittest");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI tests");
  return RUN_ALL_TESTS();
}

