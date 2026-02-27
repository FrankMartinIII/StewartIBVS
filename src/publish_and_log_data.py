#This is a second node to replace publish_and_log_pose.py
#instead, I will log the leg length commands and actual feedback from actuators,
#along with the rvec and tvec of the charuco board in the camera frame

import sys, select, tty, termios
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray
import csv
import numpy as np
import rospkg
import os

class PoseLogger:
    def __init__(self, cmd_file, target_samples=10, wait_time=100.0, log_file="calibration_log_with_legs.csv"):
        rospy.init_node('publish_and_log_data')
        self.charuco_rvecs_buffer = np.zeros(3)
        self.charuco_tvecs_buffer = np.zeros(3)
        self.most_recent_leg_lengths = np.zeros(6)
        self.most_recent_leg_feedback = np.zeros(6)
        self.wait_time = wait_time
        self.target_samples = target_samples

        #New subscriber here gets the rvecs and tvecs. I will load these into a pose, ignoring the w part of quat.
        #I really should make a custom msg but that would take more time.
        #I set queue size to 1 so that older messages are thrown away
        rospy.Subscriber("/charuco_vecs", PoseStamped, self.charuco_callback, queue_size=1)
        #Another subscriber for commanded leg lengths

        #And another subscriber for leg feedback. Problem is that leg feedback isnt going to be in mm
        #So I need to convert from command to leng in callback
        rospy.Subscriber("/legFeedback", Float64MultiArray, self.charuco_callback, queue_size=1)
        #Publisher to send the input commands
        self.pub = rospy.Publisher('vs_stewart/platformCmd', Twist, queue_size=10)

        self.cmd_poses = self.load_cmd_file(cmd_file)
        self.log_file = log_file
        with open(self.log_file, "w") as f:
            f.write("cmd_x,cmd_y,cmd_z,cmd_roll,cmd_pitch,cmd_yaw,"
                    "meas_x,meas_y,meas_z,meas_roll,meas_pitch,meas_yaw,time_published,time_received\n")

    #Load file and read it
    def load_cmd_file(self, filename):
        poses = []
        #Open file for reading
        with open(filename, "r") as f: 
            for line in f:
                if line.strip == "": #Skipping blanks I think
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
    
    def charuco_callback(self, pose_stamped):
        tvec = np.zeros(3)
        rvec = np.zeroes(3)
        tvec[0] = pose_stamped.pose.position.x
        tvec[1] = pose_stamped.pose.position.y 
        tvec[2] = pose_stamped.pose.position.z 
        rvec[0] = pose_stamped.pose.orientation.x  
        rvec[1] = pose_stamped.pose.orientation.y  
        rvec[2] = pose_stamped.pose.orientation.z 
        #w is thrown away
        self.charuco_rvecs_buffer = rvec
        self.charuco_tvecs_buffer = tvec

    
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
            while time.time() - start_time < self.wait_time and len(self.pose_buffer) < self.target_samples and not rospy.is_shutdown():
                rate.sleep()
            #Once this is over, we should be safe to get leg lengths and start logging

            
            


        