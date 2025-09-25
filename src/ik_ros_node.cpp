//#include "inverse_kinematics.cpp"
#include "kinematics.cpp"
#include<ros/ros.h>
#include<ros/console.h>
#include<stdlib.h>
#include<signal.h>
#include<time.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

ros::Subscriber sub;
ros::Publisher pub;
IK ik;
std::vector<double> legLens;
std_msgs::Float64MultiArray rosPubMsg;

void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    double xtrans = msg->linear.x;
    double ytrans = msg->linear.y;
    double ztrans = msg->linear.z;
    double roll = msg->angular.x;
    double pitch = msg->angular.y;
    double yaw = msg->angular.z;
    ik.inverseKinematics(xtrans, ytrans, ztrans, roll, pitch, yaw);
    legLens = ik.getLegLengths();

    for(int i = 0 ; i < legLens.size() ; i++)
    {
        rosPubMsg.data[i] = legLens[i];
    }
    pub.publish(rosPubMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_ros_node");
    ROS_INFO("Starting ik_ros_node");
    std::vector<double> v = ik.getLegLengths();
    rosPubMsg.data.resize(6);
    rosPubMsg.data = v;
    //std::cout<<rosPubMsg.data.at(5)<<std::endl;
    ros::NodeHandle nh;
    sub = nh.subscribe("vs_stewart/platformCmd", 100, callback);
    pub = nh.advertise<std_msgs::Float64MultiArray>("vs_stewart/legOperation", 100, true);
    
    pub.publish(rosPubMsg);

    ros::spin();


    return 0;
}