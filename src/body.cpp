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

#include "hri/body.h"

#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace hri;

Body::Body(ID id, ros::NodeHandle& nh, tf2_ros::Buffer* tf_buffer_ptr,
           const std::string& reference_frame)
  : FeatureTracker{ id, nh }, _tf_buffer_ptr(tf_buffer_ptr), _reference_frame(reference_frame)
{
}


Body::~Body()
{
  ROS_DEBUG_STREAM("Deleting body " << id_);
  roi_subscriber_.shutdown();
}

void Body::init()
{
  ns_ = "/humans/bodies/" + id_;
  ROS_DEBUG_STREAM("New body detected: " << ns_);

  roi_subscriber_ = node_.subscribe<hri_msgs::NormalizedRegionOfInterest2D>(
      ns_ + "/roi", 1, bind(&Body::onRoI, this, _1));

  cropped_subscriber_ = node_.subscribe<sensor_msgs::Image>(
      ns_ + "/cropped", 1, bind(&Body::onCropped, this, _1));

  skeleton_subscriber_ = node_.subscribe<hri_msgs::Skeleton2D>(
      ns_ + "/skeleton2d", 1, bind(&Body::onSkeleton, this, _1));
}

void Body::onRoI(hri_msgs::NormalizedRegionOfInterest2DConstPtr roi)
{
  roi_ = *roi;
}

NormROI Body::roi() const
{
  return roi_;
}

void Body::onCropped(sensor_msgs::ImageConstPtr msg)
{
  cropped_ = cv_bridge::toCvShare(msg)->image;
}

cv::Mat Body::cropped() const
{
  return cropped_;
}

void Body::onSkeleton(hri_msgs::Skeleton2DConstPtr msg)
{
  skeleton_ = msg->skeleton;
}

std::vector<SkeletonPoint> Body::skeleton() const
{
  return skeleton_;
}

boost::optional<geometry_msgs::TransformStamped> Body::transform() const
{
  try
  {
    auto transform = _tf_buffer_ptr->lookupTransform(_reference_frame, frame(),
                                                     ros::Time(0), BODY_TF_TIMEOUT);

    return transform;
  }
  catch (tf2::LookupException)
  {
    ROS_WARN_STREAM("failed to transform the body frame " << frame() << " to " << _reference_frame
                                                          << ". Are the frames published?");
    return boost::optional<geometry_msgs::TransformStamped>();
  }
}
