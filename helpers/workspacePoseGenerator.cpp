#include<iostream>
#include<fstream>
#include<filesystem>
#include<random>
#include "../src/kinematics.cpp"
#include "hardwareConfig.h"
#include <ros/ros.h>
#include <ros/package.h>

int main()
{
    IK platform;
    float xyRangeDifferential = 15.0; //15 mm radius
    float zRangePos = 14.0;
    float zRangeNeg = -14.0;
    float rollRange = 10.0;
    float pitchRange = 10.0;

    //Open file
    std::string pkgPath = ros::package::getPath("vs_stewart");
    std::string filePath = pkgPath + "/data/calibration_poses_list.txt";
    std::ofstream outFile(filePath);

    //Set up rng
    std::random_device rnd;
    std::mt19937 gen(rnd());

    std::uniform_real_distribution<> xyTranslationDist(-xyRangeDifferential, xyRangeDifferential);
    std::uniform_real_distribution<> zTranslationDist(zRangeNeg, zRangePos);
    std::uniform_real_distribution<> rollRotationDist(-rollRange, rollRange);
    std::uniform_real_distribution<> pitchRotationDist(-pitchRange, pitchRange);

    int targetPoseCount = 100;
    int successfulPoses = 0;
    int totalAttempts = 0;

    while(successfulPoses < targetPoseCount)
    {
        totalAttempts++;

        //Generate pose
        double x = xyTranslationDist(gen);
        double y = xyTranslationDist(gen);
        double z = zTranslationDist(gen);

        double roll = rollRotationDist(gen);
        double pitch = pitchRotationDist(gen);
        double yaw = 0.0;
        

        //Check if pose is valid
        int status = platform.inverseKinematicsToolOffset(x,y,z,roll,pitch,yaw);
        if(status == 0)
        {
            std::vector<double> legLens = platform.getLegLengths();

            for(int i = 0 ; i < legLens.size() ; i++)
            {
                //conversion func and min and max commands are stored in stew_driver_node.cpp
                int command = convertLengthToCommand(legLens[i]);
                if((command < minExtensionCommand) || (command > maxExtensionCommand))
                {
                    //Bad commands are skipped
                    status = -1;
                    break;
                }
            }
            if(status == 0) //If the convertLengthToCommand check also passed
            {
                outFile << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << "\n";
                successfulPoses++;
            }
        }

    }
    outFile.close();
    std::cout << "Successfully generated " << successfulPoses << " poses out of " << totalAttempts << " random attempts." << std::endl;
    std::cout << "Filepath: " << filePath << std::endl;
    return 0;

}