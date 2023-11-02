// Copyright 2022 PAL Robotics
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the PAL Robotics S.L. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <hri/hri.hpp>
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;


class ShowFaces : public rclcpp::Node
{
public:
  ShowFaces()
  : Node("show_faces")
  {
    hri_listener_ = std::make_shared<hri::HRIListener>();
    timer_ = create_wall_timer(
      500ms, std::bind(&ShowFaces::timer_callback, this));
  }


  void timer_callback()
  {
    auto faces = hri_listener_->getFaces();
    for (auto & f : faces) {
      auto face_id = f.first;
      auto face = f.second;
      if (face) {
        if (!face->cropped().empty()) {
          cv::imshow("Cropped face " + face_id, face->cropped());
        }
        if (!face->aligned().empty()) {
          cv::imshow("Aligned face " + face_id, face->aligned());
        }
        cv::waitKey(10);
      }
    }
  }

private:
  std::shared_ptr<hri::HRIListener> hri_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShowFaces>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
