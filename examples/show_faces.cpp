#include <hri/hri.hpp>
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>




using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);


  // ros::NodeHandle nh;

  rclcpp::Rate loop_rate(10ms);
  auto hri_listener_ = std::make_shared<hri::HRIListener>();
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(hri_listener_);
  // hri::HRIListener hri_listener;



  // hri_listener.onFace(&onFace);

  while (rclcpp::ok())
  {
    auto faces = hri_listener_->getFaces();
    for (auto& f : faces)
    {
      auto face_id = f.first;
      RCLCPP_INFO(hri_listener_->get_logger(),"face_id: %s",face_id);
      auto face = f.second.lock();
      //  auto face = f.second;    
      if (face)
      {
        if (!face->cropped().empty())
        {
          cv::imshow("Cropped face " + face_id, face->cropped());
        }
        if (!face->aligned().empty())
        {
          cv::imshow("Aligned face " + face_id, face->aligned());
        }

        cv::waitKey(10);
      }
    }
    // executor.spin();
    rclcpp::spin_some(hri_listener_);
    loop_rate.sleep();
  }

  return 0;
}
