#include<iostream>
#include<filesystem>
#include<opencv2/opencv.hpp>
#include<vector>
#include<Eigen/Dense>
#include <ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<fstream>

cv::Mat readCameraMatrix(std::string filename)
{
    //Setting up 3x3 matrix of doubles to hold the camera parameters
    cv::Mat cameraMatrix = cv::Mat(3,3,CV_64F);
    std::ifstream file(filename);

    //Handle case where file cannot be opened
    if(!file.is_open())
    {
        std::cerr << "File cannot be opened " << filename << std::endl;
        return cv::Mat();
    }

    //Read in 2D array
    for(int i = 0 ; i < 3 ; i++)
    {
        for(int j = 0 ; j < 3; j++)
        {
            file >> cameraMatrix.at<double>(i,j);
        }
    }

    file.close();
    return cameraMatrix;
}

int main(int argc, char** argv)
{
    std::string path = std::filesystem::current_path().string();
    path = path + "/src/vs_stewart/calibration_params/camera_matrix.txt";
    cv::Mat K = readCameraMatrix(path);

    for(int i = 0 ; i < 3 ; i++)
    {
        for(int j = 0 ; j < 3 ; j++)
        {
            std::cout << K.at<double>(i,j)<< std::endl;
        }
    }
}