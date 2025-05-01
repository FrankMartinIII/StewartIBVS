#include<ros/ros.h>
#include<ros/console.h>
#include<stdlib.h>
#include<signal.h>
#include<time.h>
#include "std_msgs/Float64MultiArray.h"
#include "firgelli.h"

ros::Publisher pub;
std_msgs::Float64MultiArray rosPubMsg;

Firgelli firgelli;
int numFound = 0;

void getFeedback()
{
    for(int i = 0 ; i < numFound ; i++)
    {
        //Write 0x10 or 16 to read the value
        rosPubMsg.data[i] = firgelli.WriteCodeToRank(i, 16, 0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lac_feedback_node");
    firgelli.SetDebug(1);
    numFound = firgelli.OpenAll();

    std::vector<double> testVec(6);
    ROS_INFO("Starting lac_feedback_node");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float64MultiArray>("vs_stewart/legFeedback", 100);
    ros::Rate loop_rate(30);
    rosPubMsg.data.resize(6);

    int count = 0;
    while(ros::ok())
    {
        /*
        for(int i = 0 ; i< testVec.size() ; i++)
        {
            rosPubMsg.data[i] = count;
        }
        */
        getFeedback();

        pub.publish(rosPubMsg);

        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    ros::spin();


    return 0;
}