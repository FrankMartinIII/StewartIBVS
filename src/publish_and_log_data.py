#This is a second node to replace publish_and_log_pose.py
#instead, I will log the leg length commands and actual feedback from actuators,
#along with the rvec and tvec of the charuco board in the camera frame

import sys, select, tty, termios
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from vs_stewart.msg import PoseData
from std_msgs.msg import Float64MultiArray
import csv
import numpy as np
import rospkg
import os

class PoseLogger:
    def __init__(self, cmd_file, target_samples=10, wait_time=100.0, log_file="calibration_log_with_legs.csv"):
        rospy.init_node('publish_and_log_data')
        #Rvecs and tvecs of the charuco as detected by estimatePoseCharucoBoard, in camera space I think
        self.charuco_rvecs_buffer = np.zeros(3)
        self.charuco_tvecs_buffer = np.zeros(3)
        #Hold the charuco pose from the pose tracking node (xyzrpy) and the current charuco pose relative to the starting pose
        #These are both in the world frame
        self.charuco_starting_pose_in_world_coords = np.zeros(6)
        self.charuco_relative_pose_in_world_coords = np.zeros(6)
        self.charuco_most_recent_time = 0
        self.most_recent_leg_commands = np.zeros(6)
        self.most_recent_leg_feedback = np.zeros(6)
        self.wait_time = wait_time
        self.target_samples = target_samples

        #New subscriber here gets the rvecs and tvecs and charuco xyzrpy starting and relative to starting. I will load these into a pose, ignoring the w part of quat.
        #I set queue size to 1 so that older messages are thrown away
        rospy.Subscriber("vs_stewart/charuco_vecs", PoseData, self.charuco_callback, queue_size=1)
        #Another subscriber for commanded leg lengths
        rospy.Subscriber("vs_stewart/legOperation", Float64MultiArray, self.leg_commands_callback, queue_size=1)
        #And another subscriber for leg feedback. Problem is that leg feedback isnt going to be in mm
        #So I need to convert from command to leng in callback
        rospy.Subscriber("vs_stewart/legFeedback", Float64MultiArray, self.leg_feedback_callback, queue_size=1)
        #Publisher to send the input commands
        self.pub = rospy.Publisher('vs_stewart/platformCmd', Twist, queue_size=10)

        self.cmd_poses = self.load_cmd_file(cmd_file)
        self.log_file = log_file
        with open(self.log_file, "w") as f:
            f.write("cmd_x,cmd_y,cmd_z,cmd_roll,cmd_pitch,cmd_yaw,"
                    "tvec_x,tvec_y,tvec_z,rvec_x,rvec_y,rvec_z,legCom_0,legCom_1,legCom_2,legCom_3,legCom_4,legCom_5,legFeed_0,legFeed_1,legFeed_2,legFeed_3,legFeed_4,legFeed_5,rel_x,rel_y,rel_z,rel_roll,rel_pitch,rel_yaw,start_x,start_y,start_z,start_roll,start_pitch,start_yaw,time_published,time_received\n")

    #Load file and read it
    def load_cmd_file(self, filename):
        poses = []
        #Open file for reading
        with open(filename, "r") as f: 
            for line in f:
                if line.strip() == "": #Skipping blanks I think
                    continue
                parts = list(map(float, line.strip().split()))
                if len(parts) != 6:
                    rospy.logwarn(f"Line does not have 6 vals. Ignoring invalid line: {line.strip()}")
                else:
                    poses.append(parts)
        return poses
    
    #I have not done anything with this function as of right now.
    #Perhaps I should try averaging the data afterwards, and instead collect it all for now.
    def average_rvecs_tvecs(self):
        #Not implemented for now
        return
    
    def convert_cmd_to_len(self, command):
        #int command = ((length - 122.64) / 30) * 1023;
        length = ((30 * command) / 1023) + 122.64
        return length
    
    def charuco_callback(self, pose_data : PoseData):
        tvec = np.zeros(3)
        rvec = np.zeros(3)
        tvec[0] = pose_data.charucoTvec.x
        tvec[1] = pose_data.charucoTvec.y
        tvec[2] = pose_data.charucoTvec.z
        rvec[0] = pose_data.charucoRvec.x 
        rvec[1] = pose_data.charucoRvec.y  
        rvec[2] = pose_data.charucoRvec.z
        self.charuco_rvecs_buffer = rvec
        self.charuco_tvecs_buffer = tvec
        self.charuco_starting_pose_in_world_coords[0] = pose_data.charucoStartingInWorldCoordinates.position.x
        self.charuco_starting_pose_in_world_coords[1] = pose_data.charucoStartingInWorldCoordinates.position.y
        self.charuco_starting_pose_in_world_coords[2] = pose_data.charucoStartingInWorldCoordinates.position.z
        self.charuco_starting_pose_in_world_coords[3] = pose_data.charucoStartingInWorldCoordinates.orientation.x
        self.charuco_starting_pose_in_world_coords[4] = pose_data.charucoStartingInWorldCoordinates.orientation.y
        self.charuco_starting_pose_in_world_coords[5] = pose_data.charucoStartingInWorldCoordinates.orientation.z

        self.charuco_relative_pose_in_world_coords[0] = pose_data.charucoInWorldCoordinates.position.x
        self.charuco_relative_pose_in_world_coords[1] = pose_data.charucoInWorldCoordinates.position.y
        self.charuco_relative_pose_in_world_coords[2] = pose_data.charucoInWorldCoordinates.position.z
        self.charuco_relative_pose_in_world_coords[3] = pose_data.charucoInWorldCoordinates.orientation.x
        self.charuco_relative_pose_in_world_coords[4] = pose_data.charucoInWorldCoordinates.orientation.y
        self.charuco_relative_pose_in_world_coords[5] = pose_data.charucoInWorldCoordinates.orientation.z
        self.charuco_most_recent_time = pose_data.header.stamp
    
    def leg_commands_callback(self, float_64_multi_array):
        incoming_data = np.array(float_64_multi_array.data).reshape(self.most_recent_leg_feedback.shape)
        self.most_recent_leg_commands[:] = incoming_data

    def leg_feedback_callback(self, float_64_multi_array):
        #Loop thru the 6 legs and convert feedback to lengths
        incoming_data = np.array(float_64_multi_array.data).reshape(self.most_recent_leg_feedback.shape)
        for leg_id in range(0,6):
            incoming_data[leg_id] = self.convert_cmd_to_len(incoming_data[leg_id])
        self.most_recent_leg_feedback[:] = incoming_data


    
    def run(self):
        rate = rospy.Rate(10)
        for cmd in self.cmd_poses:
            #Publishing the pose
            rospy.loginfo(f"Publishing commanded pose: {cmd}")

            twist = Twist()
            twist.linear.x = cmd[0]
            twist.linear.y = cmd[1]
            twist.linear.z = cmd[2]
            twist.angular.x = cmd[3]
            twist.angular.y = cmd[4]
            twist.angular.z = cmd[5]

            self.pub.publish(twist)
            pub_time = rospy.Time.now()

            #Getting the response
            #Two empty buffers to hold the rvec and tvec neatly
            #These will hold the most recent rvec and tvec for this command
            #The point of this delay code is to give the robot time to move into position

            start_time = time.time()
            while time.time() - start_time < self.wait_time and not rospy.is_shutdown():
                rate.sleep()
            #Once this is over, we should be safe to get leg lengths and start logging

            #Log and append to csv
            
            with open(self.log_file, "a") as f:
                row = cmd + self.charuco_tvecs_buffer.tolist() + self.charuco_rvecs_buffer.tolist() + self.most_recent_leg_commands.tolist() + self.most_recent_leg_feedback.tolist() + self.charuco_relative_pose_in_world_coords.tolist() + self.charuco_starting_pose_in_world_coords.tolist() + [pub_time, self.charuco_most_recent_time]
                f.write(",".join(map(str,row)) + "\n")
                


if __name__ == "__main__":
    # Create a RosPack object
    rospack = rospkg.RosPack()
    # Get the path to your package
    package_path = rospack.get_path('vs_stewart')
    commands_file_path = os.path.join(package_path, 'data', 'commands.txt')
    return_file_path = os.path.join(package_path, 'data', 'calib_log2.csv')
    cmd_file = rospy.get_param("~cmd_file", commands_file_path)
    wait_time = rospy.get_param("~wait_time", 6.0) #seconds
    logger = PoseLogger(cmd_file, wait_time=wait_time, log_file=return_file_path)
    logger.run()
            
            


        