#include<ros/ros.h>
#include<ros/console.h>
#include<stdlib.h>
#include<signal.h>
#include<time.h>
#include "std_msgs/Float64MultiArray.h"
#include "firgelli.h"
#include "hardwareConfig.h"

Firgelli firgelli;
int numFound;
ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher pub2; //For debugging leg commands
std_msgs::Float64MultiArray rosPubMsg;
std_msgs::Float64MultiArray legCommands;


void initStewartPlatform()
{
    firgelli.SetDebug(1);
    numFound = firgelli.OpenAll();
    //numFound = firgelli.OpenAll(); Maybe I should fix this later. OpenAll doesnt have a return rn

    /* This logic should be moved to be handled by sending an initial message setting the platform pose
    int baseExtensionCmd = convertLengthToCommand(0); //15 mm is the home position

    for(int rank = 0 ; rank < numfound ; rank++)
    {
    	//firgelli.WriteCodeToRank(rank, 1, 17); //Calibrate accuracy
        //firgelli.WriteCodeToRank(rank, 32, baseExtensionCmd);
        movePlatform(0, 0, 0, 0, 0, 0);
    }
    */
   if(numFound !=6)
   {
        exit(-1);
   }
   for(int rank = 0 ; rank < numFound ; rank++)
    {
    	//firgelli.WriteCodeToRank(rank, 1, 17);
        firgelli.WriteCodeToRank(rank, 1, 5); //Accuracy
        firgelli.WriteCodeToRank(rank, 5, 2500); //Stall time. Will stop moving after x ms of being stuck.
        //firgelli.WriteCodeToRank(rank, 33, 700); //Set speed of linear actuators
    }

}

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ROS_INFO("Callback activated");
    bool flagBadMove = false; //True if the legs cannot move into the asked position.
    std::vector<int> commands;
    commands.resize(msg->data.size());
    for(int rank = 0 ; rank < msg->data.size() ; rank++)
    {
        commands.at(rank) = convertLengthToCommand(msg->data.at(rank));
        std::string m = std::string("Leg command ") + std::to_string(rank) + std::string(" ") + std::to_string(commands[rank]) + std::string("\n");
        ROS_INFO(m.c_str());
        legCommands.data[rank] = commands[rank];
    }

    //Check for bad moves that exceed bounds
    if(commands.at(0) < 0) //If ik_ros_node sends a bad flag in the data
    {
        flagBadMove = true;
    }
    for(int rank = 0; rank < commands.size() ; rank++)
    {
        std::string err;
        if(commands[rank] < minExtensionCommand)
        {
            flagBadMove = true;
            err = std::string("ERROR: Command to leg: ") + std::to_string(rank) + std::string(" is below ") + std::to_string(minExtensionCommand) + std::string("\n");
            ROS_INFO("%s",err.c_str());
        }

        if(commands[rank] > maxExtensionCommand)
        {
            flagBadMove = true;
            err = std::string("ERROR: Command to leg: ") + std::to_string(rank) + std::string(" is above ") + std::to_string(maxExtensionCommand) + std::string("\n");
            ROS_INFO("%s",err.c_str());
        }
        
    }

    //Move the legs
    if(!flagBadMove)
    {
        for(int rank = 0 ; rank < numFound ; rank++)
        {
            //Move each leg
            ROS_INFO("Moving the legs");
            firgelli.WriteCodeToRank(rank, 32, commands[rank]);
        }
    }

    else
    {
        ROS_INFO("Aborting this position command");
    }
}

void getFeedback()
{
    for(int i = 0 ; i < numFound ; i++)
    {
        //Write 0x10 or 16 to read the value
        rosPubMsg.data[i] = firgelli.WriteCodeToRank(i, 16, 0);
    }
}

void minimizeStewartPlatform()
{
    firgelli.SetDebug(1);

    for(int rank = 0 ; rank < numFound ; rank++)
    {
        firgelli.WriteCodeToRank(rank, 32, 0);
    }

}

int main(int argc, char** argv)
{
    ROS_INFO("Starting stew_driver_node");
    initStewartPlatform();
    firgelli.SetDebug(1);
    rosPubMsg.data.resize(6);
    legCommands.data.resize(6);
    ros::init(argc, argv, "stew_driver_node");
    ros::NodeHandle nh;
    sub = nh.subscribe("vs_stewart/legOperation", 100, callback);
    pub = nh.advertise<std_msgs::Float64MultiArray>("vs_stewart/legFeedback", 100);
    pub2 = nh.advertise<std_msgs::Float64MultiArray>("vs_stewart/legCommands", 100); //For debugging
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        getFeedback();

        pub.publish(rosPubMsg);
        pub2.publish(legCommands);

        ros::spinOnce();
        loop_rate.sleep();
    }

    
    minimizeStewartPlatform();
}