import sys, select, tty, termios
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import csv
import numpy as np

class PoseLogger:
    def __init__(self, cmd_file, target_samples=10, wait_time=5.0, log_file="calib_log.csv"):
        self.pose_buffer = []
        self.wait_time = wait_time
        self.target_samples = target_samples
        self.initial_measured = None

        rospy.Subscriber("/aruco_pose", PoseStamped, self.aruco_callback)
        self.pub = rospy.Publisher('vs_stewart/platformCmd', Twist, queue_size=10)

        self.cmd_poses = self.load_cmd_file(cmd_file)
        self.log_file = log_file
        with open(self.log_file, "w") as f:
            f.write("cmd_x,cmd_y,cmd_z,cmd_roll,cmd_pitch,cmd_yaw,"
                    "meas_x,meas_y,meas_z,meas_roll,meas_pitch,meas_yaw\n")
            
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
    
    #I will get the average pose over a few frames. Should be more stable
    def average_pose(self):
        if not self.pose_buffer:
            return None
        # average position
        # Only use the first target_samples poses
        buffer_to_average = self.pose_buffer[:self.target_samples]
        xs = [p.pose.position.x for p in self.pose_buffer]
        ys = [p.pose.position.y for p in self.pose_buffer]
        zs = [p.pose.position.z for p in self.pose_buffer]
        mean_pos = [np.mean(xs), np.mean(ys), np.mean(zs)]

        # average orientation (roll/pitch/yaw in degrees)
        rolls = [p.pose.orientation.x for p in self.pose_buffer]
        pitches = [p.pose.orientation.y for p in self.pose_buffer]
        yaws = [p.pose.orientation.z for p in self.pose_buffer]
        mean_rpy = [np.mean(rolls), np.mean(pitches), np.mean(yaws)]

        return mean_pos + mean_rpy
        
    def run(self):
        rate  = rospy.Rate(10)
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

            #Getting the response
            self.pose_buffer = []
            start_time = time.time()
            '''while time.time() - start_time < self.wait_time and not rospy.is_shutdown():
                rate.sleep()'''
            while time.time() - start_time < self.wait_time and len(self.pose_buffer) < self.target_samples and not rospy.is_shutdown():
                rate.sleep()
            avg_measured = self.average_pose()

            if avg_measured is None:
                rospy.logwarn("No ArUco measurements collected for this command")
                continue
            #Log the pose and append to csv
            with open(self.log_file, "a") as f:
                row = cmd + avg_measured
                f.write(",".join(map(str,row)) + "\n")
            rospy.loginfo(f"Logged aruco pose: {avg_measured}")

if __name__ == "__main__":
    rospy.init_node("publish_and_log_pose_node")
    cmd_file = rospy.get_param("~cmd_file", "commands.txt")
    wait_time = rospy.get_param("~wait_time", 5.0)
    logger = PoseLogger(cmd_file, wait_time)
    logger.run()