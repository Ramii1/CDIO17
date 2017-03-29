#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <cstdlib>

using namespace ros;
using namespace std;
using namespace cv;

static const char WINDOW[]="RGB Image";
static const char WINDOW2[]="Gray Image";
static const std::string OPENCV_WINDOW = "Image window";
static int drone_state;

std_msgs::Empty emp_msg;

void process(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
   
       // Draw an example circle on the video stream

       // Update GUI Window
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(3);
       
       // Output modified video stream
      //  image_pub_.publish(cv_ptr->toImageMsg());
     }

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  init(argc, argv, "hello");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  NodeHandle n;

  ROS_INFO_STREAM("HELLO ROS!");

	ros::init(argc, argv,"ARDrone_test");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);

/* Commands */
  ros::Publisher pub_takeoff;
  ros::Publisher pub_land;
  ros::Publisher pub_reset;
  ros::Publisher pub_hover;
  ros::Publisher pub_rot90;

/* initialize commands */
	pub_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
  pub_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
  pub_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
  pub_hover = node.advertise<std_msgs::Empty>("/ardrone/hover", 1);
//  pub_rot90 = node.advertise<std_msgs::Empty>("/ardrone/cw", 90);

/* Imagetransport to OpenCV */
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);

  cv::namedWindow(WINDOW);
  cv::namedWindow(WINDOW2);


  cv::destroyWindow(WINDOW);
  cv::destroyWindow(WINDOW2);

  ros::spin();

  return 0;
}
