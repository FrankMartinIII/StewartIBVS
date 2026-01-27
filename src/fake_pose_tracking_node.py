import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import PoseStamped
from collections import deque
import sys, select, tty, termios
import time
import rospy
import random
from geometry_msgs.msg import Twist


def main():
    rospy.init_node('test_pos_pub', anonymous=True)
    pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
    rate = rospy.Rate(30)  # adjust publish rate

    while not rospy.is_shutdown():
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "platform_measured"

        pose_msg.pose.position.x = random.randint(1, 10)
        pose_msg.pose.position.y = random.randint(1, 10)
        pose_msg.pose.position.z = random.randint(1, 10)

        pose_msg.pose.orientation.x = random.randint(1, 10)
        pose_msg.pose.orientation.y = random.randint(1, 10)
        pose_msg.pose.orientation.z = random.randint(1, 10)
        pose_msg.pose.orientation.w = 1.0  # not a true quaternion here

        pub.publish(pose_msg)
        rospy.loginfo("Published: %s", pose_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass