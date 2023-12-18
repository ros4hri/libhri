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

#include <functional>
#include <memory>

#include "hri/hri.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ShowFaces : public rclcpp::Node
{
public:
  ShowFaces()
  : rclcpp::Node("hri_show_faces")
  {}

  void init()
  {
    // "shared_from_this()" cannot be used in the constructor!
    hri_listener_ = hri::HRIListener::create(shared_from_this());
    timer_ = create_wall_timer(500ms, std::bind(&ShowFaces::timer_callback, this));
  }

  void timer_callback()
  {
    auto faces = hri_listener_->getFaces();
    for (auto const & [face_id, face] : faces) {
      if (auto cropped = face->cropped()) {
        cv::imshow("Cropped face " + face_id, *cropped);
      }
      if (auto aligned = face->aligned()) {
        cv::imshow("Aligned face " + face_id, *aligned);
      }
      cv::waitKey(10);
    }
  }

private:
  std::shared_ptr<hri::HRIListener> hri_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShowFaces>();
  node->init();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
