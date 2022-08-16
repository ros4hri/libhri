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
#include "hri_msgs/SoftBiometrics.h"

using namespace std;
using namespace hri;

Face::Face(ID id, const ros::NodeHandle& nh)
  : FeatureTracker(id, nh), softbiometrics_(nullptr)
{
}

Face::~Face()
{
  ROS_DEBUG_STREAM("Deleting face " << id_);
  roi_subscriber_.shutdown();
  cropped_subscriber_.shutdown();
  aligned_subscriber_.shutdown();
  landmarks_subscriber_.shutdown();
  softbiometrics_subscriber_.shutdown();
}

void Face::init()
{
  ns_ = "/humans/faces/" + id_;
  ROS_DEBUG_STREAM("New face detected: " << ns_);
}

void Face::onRoI(hri_msgs::NormalizedRegionOfInterest2DConstPtr roi)
{
  ROS_DEBUG_STREAM("updating face " << id_ << "'s ROI to " << *roi);
  roi_ = cv::Rect(roi->xmin, roi->ymin, roi->xmax - roi->xmin, roi->ymax - roi->ymin);
}

cv::Rect Face::roi() const
{
  return roi_;
}

void Face::onCropped(const sensor_msgs::Image& msg)
{
  cropped_ = cv_bridge::toCvCopy(msg)->image;  // if using toCvShare, the image ends up shared with aligned_!
}

cv::Mat Face::cropped() const
{
  return cropped_;
}

void Face::onAligned(const sensor_msgs::Image& msg)
{
  aligned_ = cv_bridge::toCvCopy(msg)->image;  // if using toCvShare, the image ends up shared with cropped_!
}

cv::Mat Face::aligned() const
{
  return aligned_;
}

void Face::onLandmarks(hri_msgs::FacialLandmarksConstPtr msg)
{
  int i = 0;

  for (auto landmark : msg->landmarks)
  {
    facial_landmarks_[i].x = landmark.x;
    facial_landmarks_[i].y = landmark.y;
    facial_landmarks_[i].c = landmark.c;
    ++i;
  }
}

void Face::onSoftBiometrics(hri_msgs::SoftBiometricsConstPtr biometrics)
{
  softbiometrics_ = biometrics;
}


boost::optional<float> Face::age() const
{
  if (!softbiometrics_)
    return boost::optional<float>();

  return softbiometrics_->age;
}

boost::optional<Gender> Face::gender() const
{
  if (!softbiometrics_)
    return boost::optional<Gender>();
  if (softbiometrics_->gender == 0)  // UNDEFINED
    return boost::optional<Gender>();

  return static_cast<Gender>(softbiometrics_->gender);
}
