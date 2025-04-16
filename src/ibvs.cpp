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

cv::Point2d getImagePlaneCoordsFromPixels(const cv::Mat& cameraMatrix, double u, double v)
{
    //Function to take the pixel values of a feature m = (u, v) and transform them to coordinate of the point in the image plane
    //This is perspective projection/normalization
    //Detailed in Chaumette 2006 pg 83
    //x = X/Z = (u-c_u)/f*alpha
    //y = Y/Z = (v-c_v)/f

    //Get intrinsic camera parameters
    double f_x = cameraMatrix.at<double>(0,0);
    double f_y = cameraMatrix.at<double>(1,1);
    double c_u = cameraMatrix.at<double>(0,2);
    double c_v = cameraMatrix.at<double>(1,2);

    std::cout << "f_x " << f_x << " f_y " << f_y << " c_u " << c_u << " c_v " << c_v << std::endl;

    //Actually we do not need alpha if we simply use f_x and f_y in their respective equations (I think) //double alpha = f_x / f_y;

    double x = (u - c_u) / f_x;
    double y = (v - c_v) / f_y;

    return cv::Point2d(x,y);
}

Eigen::MatrixXd makeLePlusMatrix(std::vector<cv::Point2d>& curFeaturePointsInPlane, std::vector<cv::Point2d>& desiredFeaturePointsInPlane)
{
    //Need to pass in list of feature points to calc pseudoinverse matrix
    //Also will need the points in the desired image if we want to use (1/2)(L_e + L_eStar) for the pseudoinverse calculation
    //L matrix has 2 rows per point(and we will have 4 colored blobs) and 6 columns each
    Eigen::MatrixXd L_total(8,6);
    //L_total_star is made from the desired image points
    Eigen::MatrixXd L_total_star(8,6);
    Eigen::VectorXd e(8);
    double Z = 1;

    for(int i = 0 ; i < 4 ; i++)
    {
        //For each feature point (x,y)

        //Extract current x and y and desired x and y for that particular feature
        double curx = curFeaturePointsInPlane[i].x;
        double cury = curFeaturePointsInPlane[i].y;
        double desiredx = desiredFeaturePointsInPlane[i].x;
        double desiredy = desiredFeaturePointsInPlane[i].y;
        
        /* According to Chaumette 2006 pg 83
        L_x = (-1/Z)     0        x/Z       xy     -(1+x^2)     y
            0       (-1/Z)     y/Z     (1+y^2)    -xy       -x
        */
        Eigen::Matrix<double, 2, 6> L_x;
        L_x << (-1/Z),   0,      (curx/Z),      curx*cury,      -(1+curx*curx),    cury,
                0,     (-1/Z),   (cury/Z),      (1+cury*cury),    -curx*cury,       -curx;

        L_total.block<2,6>(2*i, 0) = L_x;
        
        //Doing the same calculation for L_star. 
        //This is inefficient though, since this technically would never change. It could be passed in as a variable.
        Eigen::Matrix<double, 2, 6> L_star;
        L_star << (-1/Z),   0,      (desiredx/Z),      desiredx*desiredy,      -(1+desiredx*curx),    desiredy,
                    0,     (-1/Z),   (desiredy/Z),      (1+desiredy*cury),    -desiredx*desiredy,       -desiredx;

        L_total_star.block<2,6>(2*i, 0) = L_star;

        //Now create error matrix e
        //Calculate e vector, which is an 8 dimensional vector in the form [x1 y1 x2 y2 x3 y3...]
        e(2*i) = curx - desiredx;
        e(2*i + 1) = cury - desiredy;
    }

    //Then we need to convert L_x to L_e+, which is the inverse
    //But really, we find pseudoinverse L_e+hat
    //If using the averaging method L_e+hat = (1/2)*(L_e + L_e*)+
    Eigen::MatrixXd L_avg = 0.5 * (L_total + L_total_star);
    Eigen::MatrixXd L_pseudoInverse = L_avg.completeOrthogonalDecomposition().pseudoInverse();

    return L_pseudoInverse;

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
    getImagePlaneCoordsFromPixels(K, 1, 2);
    


    std::vector<cv::Point2d> current_pixels = {
        {320, 240}, {330, 240}, {320, 250}, {330, 250}
    };
    std::vector<cv::Point2d> desired_pixels = {
        {322, 242}, {332, 242}, {322, 252}, {332, 252}
    };

    std::vector<cv::Point2d> current_normalized;
    std::vector<cv::Point2d> desired_normalized;

    for (int i = 0; i < 4; ++i)
    {
        current_normalized.push_back(getImagePlaneCoordsFromPixels(K, current_pixels[i].x, current_pixels[i].y));
        desired_normalized.push_back(getImagePlaneCoordsFromPixels(K, desired_pixels[i].x, desired_pixels[i].y));
    }

    Eigen::MatrixXd L_pinv = makeLePlusMatrix(current_normalized, desired_normalized);
    std::cout << "Pseudoinverse:\n" << L_pinv << std::endl;

    return 0;
}