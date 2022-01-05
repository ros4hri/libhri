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


#ifndef HRI_BODY_H
#define HRI_BODY_H

#include <hri_msgs/RegionOfInterestStamped.h>
#include <memory>
#include <boost/optional.hpp>

#include "base.h"
#include "ros/subscriber.h"

namespace hri
{
class Body : public FeatureTracker
{
public:
  using FeatureTracker::FeatureTracker;  // inherits FeatureTracker's ctor

  virtual ~Body();

  /** \brief If available, returns the 2D region of interest (RoI) of the body.
   *
   * Use example:
   *
   * ```cpp
   * HRIListener hri_listener;
   *
   * auto bodies = hri_listener.getBodies();
   *
   * for (auto const& body : bodies)
   * {
   *   auto roi = body.second.lock()->getRoI();
   *   if (roi) {
   *     cout << "Size of body_" << body.first << ": ";
   *     cout << roi->roi.width << "x" << roi->roi.height << endl;
   *   }
   *   else {
   *     cerr << "Body " << body.first << " has no published RoI.";
   *   }
   * }
   * ```
   *
   * The coordinates are provided in the original camera's image coordinate
   * space.
   *
   * The header's timestamp is the same as a the timestamp of the original
   * image from which the body has been detected.
   */
  boost::optional<hri_msgs::RegionOfInterestStamped> getRoI() const;

  void init() override;

private:
  size_t nb_roi;

  std::string ns;

  ros::Subscriber roi_subscriber_;
  void onRoI(hri_msgs::RegionOfInterestStampedConstPtr roi);
  hri_msgs::RegionOfInterestStampedConstPtr roi_;
};

typedef std::shared_ptr<Body> BodyPtr;

}  // namespace hri
#endif
