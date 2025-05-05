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
std::vector<double> latestPoseCmd(6);
bool pendingFKReq = false;

double convertCommandToLength(double command)
{
    double length = ((command / 1023) * 30) + 121.5;
    return length;
}

void legCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    //Save the latest leg lengths
    for(int i = 0 ; i < latestLegFB.size() ; i++)
    {
        latestLegFB[i] = convertCommandToLength(msg->data[i]);
    }
    //Do the forward kinematics
    std::vector<double> realPose = ik.forwardKinematics(latestLegFB, latestPoseCmd);
    //Publish the realPose
    rosPubMsg.linear.x = realPose[0];
    rosPubMsg.linear.y = realPose[1];
    rosPubMsg.linear.z = realPose[2];
    rosPubMsg.angular.x = realPose[3];
    rosPubMsg.angular.y = realPose[4];
    rosPubMsg.angular.z = realPose[5];
    pub.publish(rosPubMsg);
}

//Get the latest pose command, which will be used as our estimate
void poseCallback(const geometry_msgs::Twist::ConstPtr& pose)
{
    latestPoseCmd[0] = pose->linear.x;
    latestPoseCmd[1] = pose->linear.y;
    latestPoseCmd[2] = pose->linear.z;
    latestPoseCmd[3] = pose->angular.x;
    latestPoseCmd[4] = pose->angular.y;
    latestPoseCmd[5] = pose->angular.z;
}
/*  //This approach isnt going to work because it is going to constantly cancel the FKRequest
//I am thinking that I should just constantly read the feedback and do the FK
//Seems inefficient and potentially problematic, but I do not know how else to do this
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
        ik.forwardKinematics(latestLegFB, latestPoseCmd)
    }
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fk_ros_node");
    ROS_INFO("Starting fk_ros_node");
    nh = new ros::NodeHandle();
    //Initialize latestPoseCmd to 0 to prevent null data
    latestPoseCmd[0] = 0;
    latestPoseCmd[1] = 0;
    latestPoseCmd[2] = 0;
    latestPoseCmd[3] = 0;
    latestPoseCmd[4] = 0;
    latestPoseCmd[5] = 0;
    //I temporarily changed legFeedback to legOperation for testing- FRANK, change this back
    feedbackSub = nh->subscribe("vs_stewart/legFeedback", 100, legCallback);
    poseSub = nh->subscribe("vs_stewart/platformCmd", 100, poseCallback);
    pub = nh->advertise<geometry_msgs::Twist>("vs_stewart/fkPose", 100, true);

    ros::spin();

    delete(nh);
    return 0;
}