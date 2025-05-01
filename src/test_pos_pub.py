
import sys, select, tty, termios
import time
import rospy
from geometry_msgs.msg import Twist

def get_user_input():
    try:
        line = input("Enter x y z roll pitch yaw (space-separated): ")
        parts = line.strip().split()
        if len(parts) != 6:
            raise ValueError("You must enter exactly 6 values.")
        values = [float(p) for p in parts]
        return values
    except ValueError as e:
        print(f"Invalid input: {e}")
        return get_user_input()

def main():
    rospy.init_node('test_pos_pub', anonymous=True)
    pub = rospy.Publisher('vs_stewart/platformCmd', Twist, queue_size=100)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        x, y, z, roll, pitch, yaw = get_user_input()

        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = roll
        twist.angular.y = pitch
        twist.angular.z = yaw

        pub.publish(twist)
        rospy.loginfo("Published Twist: %s", twist)

        cont = input("Publish another Twist? (y/n): ").lower()
        if cont != 'y':
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass