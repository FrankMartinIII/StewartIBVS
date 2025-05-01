#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

const double lambda = 0.5; //PBVS gain value, not sure what to put this as
double dt = 1; //This needs to be some rate to integrate over to get position I think
const Eigen::Vector3d needleOffset(0.0, 0.0, 0.10) //How far is the needle offset from the center of the cameras

void computePosePBVS(const Eigen::Isometry3d& currPose, const Eigen::Vector3d& desiredPose, const Eigen::Vector3d desiredOrientation)
{
    Eigen::Isometry3d desiredPose;
}