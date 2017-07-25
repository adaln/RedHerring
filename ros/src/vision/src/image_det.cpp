#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <iostream>
#include <stdio.h>

// Msg headers
#include <visiontest/BoundingBox.h>
#include <visiontest/msg_vec.h>

using namespace cv;

int count = 0;
std::clock_t start;
CascadeClassifier cascade;
ros::Publisher detect_pub;

void detection( Mat frame ){
    // Detect objects in each frame using classifier (or insert OpenCV code here)
    std::vector<Rect> detection;    
    Mat frame_gray;
    
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY ); //converts frame to grayscale before detection can occur
    equalizeHist( frame_gray, frame_gray ); //smooths frame for img processing
    cascade.detectMultiScale( frame_gray, detection, 1.1 , 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) ); //detection of the classifier object

    visiontest::msg_vec box_vec; // Msg which stores the positions of all the detected objects in the frame
    for( size_t i = 0; i < detection.size(); i++ ){ //loop draws elipse around detected objects to display on screen
        Point center( detection[i].x + detection[i].width/2, detection[i].y + detection[i].height/2 );
        ellipse( frame, center, Size( detection[i].width/2, detection[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );         
        visiontest::BoundingBox box_msg; // Msg which stores position of only one detected object in frame
        box_msg.center.x = detection[i].x - detection[i].width / 2;
        box_msg.center.y = detection[i].y - detection[i].height / 2;
        box_msg.width = detection[i].width;
        box_msg.height = detection[i].height;
        box_vec.msg_vec.push_back(box_msg);
    }
    detect_pub.publish(box_vec); // Publishes positions of all detected objects
    
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (count == 0) {
        start = std::clock();
    }
    
    count++;
	cv_bridge::CvImagePtr cv_ptr; 
	
	
	// Convert from ROS image msg to OpenCV format
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception e) {
        ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    Mat im_bgr = cv_ptr->image;
    
    
    detection( im_bgr ); // Do detection
    
    imshow("Webcam", cv_ptr->image);
    waitKey(30);
    if (count == 10) {
        double duration = (std::clock() - start) / ( (double) CLOCKS_PER_SEC);
        ROS_INFO("FPS: %f", 10 / duration);
        count = 0;
    }
    return;
    
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "image_reciever");
    ros::NodeHandle nh;
    String cascade_name = argv[1]; //Input location of classifier as needed
    if( !cascade.load( cascade_name ) ){ // load classifier
        ROS_ERROR("--(!)Error loading classifier\n");
    }
    namedWindow("Webcam");
    startWindowThread();
    
    image_transport::ImageTransport it(nh);
    detect_pub = nh.advertise<visiontest::msg_vec>("/detection/clasifier", 1000); // Start publisher for detected object position msgs
    image_transport::Subscriber sub = it.subscribe("camera/one/image", 5, imageCallback); // may need to change first parameter to subscribe to correct topic
    
    ros::spin();
    destroyWindow("Webcam");

    return 0;
}
