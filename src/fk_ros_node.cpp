#include "inverse_kinematics.cpp"
#include<ros/ros.h>
#include<ros/console.h>
#include<stdlib.h>
#include<signal.h>
#include<time.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

ros::NodeHandle* nh = nullptr;
ros::Subscriber feedbackSub;
ros::Subscriber poseSub;
ros::Publisher pub;
ros::Timer delayTimer;
double delaySeconds;

IK ik;

geometry_msgs::Twist rosSubMsg; //Listen to the current pose
geometry_msgs::Twist rosPubMsg; //Publish the pose from the forward kinematic calcs

std::vector<double> latestLegFB(6);
std::vector<double> latestPose(6);
bool pendingFKReq = false;

void legCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    //Save the latest leg lengths
    for(int i = 0 ; i < latestLegFB.size() ; i++)
    {
        latestLegFB[i] = msg->data[i];
    }

}

void poseCallback(const geometry_msgs::Twist::ConstPtr& pose)
{
    pendingFKReq = true;

    // Cancel any existing timer
    delayTimer.stop();

    // Start a new one-shot timer
    delayTimer = nh->createTimer(ros::Duration(delaySeconds), &timerCallback, true);
}

void timerCallback(const ros::TimerEvent&)
{
    if(pendingFKReq)
    {
        //Do FK
        ik.forwardKinematics(latestLegFB, latestPose)
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fk_ros_node");
    ROS_INFO("Starting fk_ros_node");
    nh = new ros::NodeHandle();
    feedbackSub = nh->subscribe("vs_stewart/legFeedback", 100, legCallback);
    poseSub = nh->subscribe("vs_stewart/platformCmd", 100, poseCallback);

    ros::spin();

    delete(nh);
    return 0;
}