#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

int main()
{
    cv::VideoCapture cap;
    cv::Mat image;

    if (!cap.open("tcp://192.168.1.1:5555"))
    {
        printf("AR.Drone ERROR CONNECT\n");
        return -1;
    }

    while (1)
    {
        cap >> image;

        if (!image.empty())
        {
            cv::imshow("AR.Drone", image);
            cout << "OK" << endl;
        }
        else
        {
            cout << "ERROR" << endl;
            cv::waitKey(1);
        }
    }

    return 0;
}