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


#ifndef HRI_FACE_H
#define HRI_FACE_H

#include <hri_msgs/PointOfInterest2D.h>
#include <hri_msgs/RegionOfInterestStamped.h>
#include <memory>

#include "base.h"
#include "ros/subscriber.h"

namespace hri
{
struct IntensityConfidence
{
  float intensity;
  float confidence;
};

class Face : public FeatureTracker
{
public:
  using FeatureTracker::FeatureTracker;  // inherits FeatureTracker's ctor

  virtual ~Face();

  hri_msgs::RegionOfInterestStamped getRoI() const
  {
    return roi_;
  }

  /** \brief the list of the 66 facial landmarks (2D points, expressed in normalised coordinates).
   *
   * The location of the landmarks is defined here:
   * https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#face-output-format
   *
   * Constants defined in hri_msgs/FacialLandmarks.h can be used to access
   * specific points on the face.
   */
  std::array<hri_msgs::PointOfInterest2D, 66> getFacialLandmarks() const
  {
    return facial_landmarks_;
  }

  /** \brief the list of the facial action units detected on the face.
   *
   * Action units indices follow the Action Unit naming convention by Ekman.
   * List here: https://en.wikipedia.org/wiki/Facial_Action_Coding_System
   *
   * Note that the list is sparse (some indices do not exist in Ekman classification).
   * In addition, depending on the AU detector, some action units might not be
   * detected. In these cases, the confidence value will be 0.
   *
   * Constants defined in hri_msgs/FacialActionUnits.h can be used to access
   * specific action units by name.
   */
  std::array<IntensityConfidence, 99> getFacialActionUnits() const
  {
    return facial_action_units_;
  }

  void init() override;

private:
  size_t nb_roi;

  std::string ns;

  ros::Subscriber roi_subscriber_;
  void onRoI(hri_msgs::RegionOfInterestStampedConstPtr roi);
  hri_msgs::RegionOfInterestStamped roi_;

  std::array<hri_msgs::PointOfInterest2D, 66> facial_landmarks_;
  std::array<IntensityConfidence, 99> facial_action_units_;
};

typedef std::shared_ptr<Face> FacePtr;

}  // namespace hri
#endif
