#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <zbar.h>
#include <zbar/Symbol.h>
#include "boost/unordered_map.hpp"

#include <Object.h>
#include <Drone.h>

using namespace ros;
using namespace std;
using namespace cv;

char *dinmor[] =
    {
        (char*)"/home/bente/Desktop/CDIOF17/devel/lib/cdio/huladetection3"};

Drone drone(1, dinmor);
//initial min and max HSV filter values.
//these will be changed using trackbars
// int H_MIN = 72;
// int H_MAX = 256;
// int S_MIN = 108;
// int S_MAX = 256;
// int V_MIN = 31;
// int V_MAX = 256;

int H_MIN = 0;
int H_MAX = 10;
int S_MIN = 70; 
int S_MAX = 255;
int V_MIN = 50;
int V_MAX = 255;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 10000;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

//The following for canny edge detec
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 2;
const char *window_name = "Edge Map";

// barcode
boost::unordered_map<std::string, ros::Time> barcode_memory_;
double throttle_ = 1;

void on_trackbar(int, void *)
{ //This function gets called whenever a
    // trackbar position is changed
}

string intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

string floatToString(float number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

float myRoundFunc(float toRound)
{
    return std::ceil(toRound - 0.5);
}

void createTrackbars()
{
    //create window for trackbars
    namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    // sprintf(TrackbarName, "H_MIN", H_MIN);
    // sprintf(TrackbarName, "H_MAX", H_MAX);
    // sprintf(TrackbarName, "S_MIN", S_MIN);
    // sprintf(TrackbarName, "S_MAX", S_MAX);
    // sprintf(TrackbarName, "V_MIN", V_MIN);
    // sprintf(TrackbarName, "V_MAX", V_MAX);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
    createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
    createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
    createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
    createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
    createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}

void drawObject(vector<Object> theObjects, Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy)
{
    for (size_t i = 0; i < contours.size(); i++)
    {
        size_t count = contours[i].size();

        if (count < 6)
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        RotatedRect box = fitEllipse(pointsf);

        if (box.size.height * box.size.width < MIN_OBJECT_AREA)
            continue;

        if (box.size.width / box.size.height < 0.7)
            continue;

        ellipse(frame, box.center, box.size * 0.5f, box.angle, 0, 360, Scalar(0, 255, 255), 1, LINE_AA);
        Point2f vtx[4];
        box.points(vtx);

        cv::putText(frame, floatToString(box.center.x) + "," + floatToString(box.center.y),
                    cv::Point(box.center.x, box.center.y), 1, 1, Scalar(0, 0, 0));
    }
}

void morphOps(Mat &thresh)
{
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(1, 1));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(10, 10));

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);

    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}

void findQrCodes(Mat &cameraFeed) {
    Mat qrcode;

    cvtColor(cameraFeed, qrcode, CV_BGR2GRAY);

    zbar::ImageScanner scanner_;
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    zbar::Image zbar_image(qrcode.cols, qrcode.rows, "Y800", qrcode.data,
                           qrcode.cols * qrcode.rows);
    int n = scanner_.scan(zbar_image);
    imshow("Camera feed", qrcode);

    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol)
    {
        ROS_INFO("%s", symbol->get_data().c_str());
        ROS_INFO("%s", symbol->get_type_name().c_str());
        ROS_INFO("%d,%d", symbol->get_location_x(0), symbol->get_location_y(0));
    }
}

void findRings(Object theObject, Mat threshold, Mat HSV, Mat &cameraFeed)
{
    vector<Object> objects;
    Mat temp;
    threshold.copyTo(temp);

    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;

    // If we actually find contours
    if (hierarchy.size() > 0)
    {
        int numObjects = hierarchy.size();

        // If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if (numObjects < MAX_NUM_OBJECTS)
        {
            // Loop through the contours
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {

                // Capture the moment
                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                // Area check
                // TODO: This should be optimized to only find areas that are needed
                if (area > MIN_OBJECT_AREA)
                {
                    Object object;

                    object.setXPos(moment.m10 / area);
                    object.setYPos(moment.m01 / area);
                    object.setType(theObject.getType());
                    object.setColor(theObject.getColor());
                    object.setArea(area);

                    objects.push_back(object);

                    objectFound = true;

                } //else objectFound = false;
            }
            //let user know you found an object
            if (objectFound == true)
            {
                //draw object location on screen
                drawObject(objects, cameraFeed, temp, contours, hierarchy);
            }
        }
        else
            putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
}

void process(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //Matrix to store each frame of the webcam feed
    Mat cameraFeed = cv_ptr->image;
    Mat threshold;
    Mat HSV;
    Mat src_gray;
    Mat src = cameraFeed;


    drone.Takeoff();

    double time_start = (double)ros::Time::now().toSec();
    while((double)ros::Time::now().toSec() < time_start + 5.0) {
        drone.Left();
    }

    while((double)ros::Time::now().toSec() < time_start + 10.0) {
        drone.Right();
    }

    drone.Land();

    cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);

    //create slider bars for HSV filtering
    createTrackbars();
    Object black("black");
    //need to find the appropriate color range values
    // calibrationMode must be false

    //if in calibration mode, we track objects based on the HSV slider values.
    // cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    morphOps(threshold);
    imshow(windowName2, threshold);

    //the folowing for canny edge detec
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());
    /// Convert the image to grayscale
    cvtColor(src, src_gray, CV_BGR2GRAY);
    /// Create a window
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    /// Create a Trackbar for user to enter threshold
    createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold);
    
    
    
    
    /// Track the image
    findRings(black, threshold, HSV, cameraFeed);
    // Find QR Codes
    findQrCodes(cameraFeed);







    imshow(windowName, cameraFeed);

    cv::waitKey(3);
}

int main(int argc, char *argv[])
{
    init(argc, argv, "huladetection2");

    NodeHandle n;

    ROS_INFO_STREAM("HELLO ROS!");

    ROS_INFO("Flying ARdrone");
    ros::Rate loop_rate(50);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw", 1, process);

    cv::namedWindow(windowName);
    cv::namedWindow(windowName1);

    cv::destroyWindow(windowName);
    cv::destroyWindow(windowName2);

    ros::spin();
}