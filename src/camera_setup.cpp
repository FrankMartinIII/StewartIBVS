#include <ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::VideoCapture cap(0);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame;

    while(true)
    {
        cap >> frame;
        cv::imshow("Camera Feed", frame);
        cv::waitKey(1);
    }
    return 0;
}