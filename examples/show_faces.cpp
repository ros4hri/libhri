#include <hri/hri.h>
#include <ros/ros.h>

#include <opencv2/highgui.hpp>

using namespace ros;
using namespace hri;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_example_show_faces");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  HRIListener hri_listener;

  // hri_listener.onFace(&onFace);

  while (ros::ok())
  {
    auto faces = hri_listener.getFaces();
    for (auto& f : faces)
    {
      auto face_id = f.first;
      auto face = f.second.lock();
      if (face)
      {
        if (face->aligned().empty())
        {
          continue;
        }

        cv::imshow("Aligned face " + face_id, face->aligned());
        cv::waitKey(10);
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
