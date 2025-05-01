#include<ros/ros.h>
#include<ros/console.h>
#include<stdlib.h>
#include<signal.h>
#include<time.h>
#include "std_msgs/Float64MultiArray.h"
#include "firgelli.h"

Firgelli firgelli;
int numFound;
ros::Subscriber sub;
std_msgs::Float64MultiArray rosPubMsg;

//Old vals were good for motor one but bounds are inconsistent across motors
const int minExtensionCommand = 45;
const int maxExtensionCommand = 950;

int convertLengthToCommand(double length)
{
    int command = (31.91 * (length-121.5)) - 16.01;
    return command;
}

void initStewartPlatform()
{
    firgelli.SetDebug(1);
    firgelli.OpenAll();
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
   for(int rank = 0 ; rank < numFound ; rank++)
    {
    	firgelli.WriteCodeToRank(rank, 1, 17);
    }

}

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    bool flagBadMove = false; //True if the legs cannot move into the asked position.
    std::vector<int> commands;
    commands.resize(msg->data.size());
    for(int rank = 0 ; rank < msg->data.size() ; rank++)
    {
        commands.at(rank) = convertLengthToCommand(msg->data.at(rank));
        std::string m = std::string("Leg command ") + std::to_string(rank) + std::string(" ") + std::to_string(commands[rank]) + std::string("\n");
        ROS_INFO(m.c_str());
    }

    //Check for bad moves that exceed bounds
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
            firgelli.WriteCodeToRank(rank, 32, commands[rank]);
        }
    }

    else
    {
        ROS_INFO("Aborting this position command");
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
    ros::init(argc, argv, "stew_driver_node");
    ros::NodeHandle nh;
    sub = nh.subscribe("vs_stewart/legOperation", 100, callback);

    ros::spin();
}