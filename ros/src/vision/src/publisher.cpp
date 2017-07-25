#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream> // for converting the command line parameter to integer
#include <string>
#include "../include/video_api.h"

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  //if(argv[1] == NULL) return 1;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  std::string name, fd, pub;
    nh.getParam("name", name);
    nh.getParam("fd", fd);
    nh.getParam("pub", pub);
    image_transport::Publisher publisher = it.advertise("/camera/one/image", 5);
  ROS_DEBUG_STREAM("Hello " << "World");

  VideoSource source;
    if (!source.SetSource(fd)) {
        ROS_ERROR("%s failed to open device on %s", name.c_str(), fd.c_str());
        return -1;
    }

    ros::Rate loop_rate(source.GetFPS());
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  while (nh.ok()) {
    frame = source.capture();
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      publisher.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

