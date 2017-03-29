#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace std;
using namespace cv;


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

int main()
{
      image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);

  cv::namedWindow(WINDOW);
  cv::namedWindow(WINDOW2);

    return 0;
}