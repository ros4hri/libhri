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

#include <memory>

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
    hri_listener_ = std::make_shared<hri::HRIListener>(this->shared_from_this());
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
