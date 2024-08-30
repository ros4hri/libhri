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
#include "hri_msgs/Expression.h"

using namespace std;
using namespace hri;

Face::Face(ID id, ros::NodeHandle& nh, tf2_ros::Buffer* tf_buffer_ptr,
           const std::string& reference_frame)
  : FeatureTracker{ id, nh }
  , softbiometrics_(nullptr)
  , _tf_buffer_ptr(tf_buffer_ptr)
  , _reference_frame(reference_frame)
{
}

Face::~Face()
{
  ROS_DEBUG_STREAM("Deleting face " << id_);
  roi_subscriber_.shutdown();
}

void Face::init()
{
  ns_ = "/humans/faces/" + id_;
  ROS_DEBUG_STREAM("New face detected: " << ns_);

  roi_subscriber_ = node_.subscribe<hri_msgs::NormalizedRegionOfInterest2D>(
      ns_ + "/roi", 1, bind(&Face::onRoI, this, _1));

  cropped_subscriber_ = node_.subscribe<sensor_msgs::Image>(
      ns_ + "/cropped", 1, bind(&Face::onCropped, this, _1));

  aligned_subscriber_ = node_.subscribe<sensor_msgs::Image>(
      ns_ + "/aligned", 1, bind(&Face::onAligned, this, _1));

  landmarks_subscriber_ = node_.subscribe<hri_msgs::FacialLandmarks>(
      ns_ + "/landmarks", 1, bind(&Face::onLandmarks, this, _1));

  softbiometrics_subscriber_ = node_.subscribe<hri_msgs::SoftBiometrics>(
      ns_ + "/softbiometrics", 1, bind(&Face::onSoftBiometrics, this, _1));

  expression_subscriber_ = node_.subscribe<hri_msgs::Expression>(
      ns_ + "/expression", 1, bind(&Face::onExpression, this, _1));
}

void Face::onRoI(hri_msgs::NormalizedRegionOfInterest2DConstPtr roi)
{
  roi_ = *roi;
}

NormROI Face::roi() const
{
  return roi_;
}

void Face::onCropped(sensor_msgs::ImageConstPtr msg)
{
  cropped_ = cv_bridge::toCvCopy(msg)->image;  // if using toCvShare, the image ends up shared with aligned_!
}

cv::Mat Face::cropped() const
{
  return cropped_;
}

void Face::onAligned(sensor_msgs::ImageConstPtr msg)
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

void Face::onExpression(hri_msgs::ExpressionConstPtr msg)
{
// Map the string to the enum manually
if (msg->expression == hri_msgs::Expression::NEUTRAL) expression_ = kNeutral;
else if (msg->expression == hri_msgs::Expression::ANGRY) expression_ = kAngry;
else if (msg->expression == hri_msgs::Expression::SAD) expression_ = kSad;
else if (msg->expression == hri_msgs::Expression::HAPPY) expression_ = kHappy;
else if (msg->expression == hri_msgs::Expression::SURPRISED) expression_ = kSurprised;
else if (msg->expression == hri_msgs::Expression::DISGUSTED) expression_ = kDisgusted;
else if (msg->expression == hri_msgs::Expression::SCARED) expression_ = kScared;
else if (msg->expression == hri_msgs::Expression::PLEADING) expression_ = kPleading;
else if (msg->expression == hri_msgs::Expression::VULNERABLE) expression_ = kVulnerable;
else if (msg->expression == hri_msgs::Expression::DESPAIRED) expression_ = kDespaired;
else if (msg->expression == hri_msgs::Expression::GUILTY) expression_ = kGuilty;
else if (msg->expression == hri_msgs::Expression::DISAPPOINTED) expression_ = kDisappointed;
else if (msg->expression == hri_msgs::Expression::EMBARRASSED) expression_ = kEmbarrassed;
else if (msg->expression == hri_msgs::Expression::HORRIFIED) expression_ = kHorrified;
else if (msg->expression == hri_msgs::Expression::SKEPTICAL) expression_ = kSkeptical;
else if (msg->expression == hri_msgs::Expression::ANNOYED) expression_ = kAnnoyed;
else if (msg->expression == hri_msgs::Expression::FURIOUS) expression_ = kFurious;
else if (msg->expression == hri_msgs::Expression::SUSPICIOUS) expression_ = kSuspicious;
else if (msg->expression == hri_msgs::Expression::REJECTED) expression_ = kRejected;
else if (msg->expression == hri_msgs::Expression::BORED) expression_ = kBored;
else if (msg->expression == hri_msgs::Expression::TIRED) expression_ = kTired;
else if (msg->expression == hri_msgs::Expression::ASLEEP) expression_ = kAsleep;
else if (msg->expression == hri_msgs::Expression::CONFUSED) expression_ = kConfused;
else if (msg->expression == hri_msgs::Expression::AMAZED) expression_ = kAmazed;
else if (msg->expression == hri_msgs::Expression::EXCITED) expression_ = kExcited;
else
{
    ROS_WARN_STREAM("Received invalid expression: " << msg->expression);
    return;
}

  // Store valence and arousal
  expression_va_ = ExpressionVA{msg->valence, msg->arousal};
  expression_confidence_ = msg->confidence;
}

boost::optional<geometry_msgs::TransformStamped> Face::transform() const
{
  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, frame(),
                                                     ros::Time(0), FACE_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    ROS_WARN_STREAM("failed to transform the face frame " << frame() << " to " << _reference_frame
                                                          << ". Are the frames published?");
    return boost::optional<geometry_msgs::TransformStamped>();
  }
}

boost::optional<geometry_msgs::TransformStamped> Face::gazeTransform() const
{
  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, gazeFrame(),
                                                     ros::Time(0), FACE_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    ROS_WARN_STREAM("failed to transform the gaze frame " << frame() << " to " << _reference_frame
                                                          << ". Are the frames published?");
    return boost::optional<geometry_msgs::TransformStamped>();
  }
}

