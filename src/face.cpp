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

#include "hri/face.h"

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace hri;

Face::~Face()
{
  ROS_DEBUG_STREAM("Deleting face " << id_);
  roi_subscriber_.shutdown();
}

void Face::init()
{
  ns_ = "/humans/faces/" + id_;
  ROS_DEBUG_STREAM("New face detected: " << ns_);

  roi_subscriber_ = node_.subscribe<sensor_msgs::RegionOfInterest>(
      ns_ + "/roi", 1, bind(&Face::onRoI, this, _1));

  cropped_subscriber_ = node_.subscribe<sensor_msgs::Image>(
      ns_ + "/cropped", 1, bind(&Face::onCropped, this, _1));

  landmarks_subscriber_ = node_.subscribe<hri_msgs::FacialLandmarks>(
      ns_ + "/landmarks", 1, bind(&Face::onLandmarks, this, _1));  
}

void Face::onRoI(sensor_msgs::RegionOfInterestConstPtr roi)
{
  roi_ = cv::Rect(roi->x_offset, roi->y_offset, roi->width, roi->height);
}

cv::Rect Face::roi() const
{
  return roi_;
}

void Face::onCropped(sensor_msgs::ImageConstPtr msg)
{
  cropped_ = cv_bridge::toCvShare(msg)->image;
}

cv::Mat Face::cropped() const
{
  return cropped_;
}

void Face::onLandmarks(hri_msgs::FacialLandmarksConstPtr msg)
{
  int i = 0;

  for(auto landmark : msg->landmarks){
    facial_landmarks_[i].x = landmark.x;
    facial_landmarks_[i].y = landmark.y;
    facial_landmarks_[i].c = landmark.c;
    ++i;
  }
}

