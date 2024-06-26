#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

# Initialize last_logged_time with None
last_logged_time = None

def odom_callback(data):
    global last_logged_time

    # If last_logged_time is not set, initialize it with the current time
    if last_logged_time is None:
        last_logged_time = rospy.Time.now()

    current_time = rospy.Time.now()
    if (current_time - last_logged_time).to_sec() >= 1.0:  # Change 1.0 to the desired interval in seconds
        position = data.pose.pose.position
        rospy.loginfo("Position -> x: {}, y: {}, z: {}".format(position.x, position.y, position.z))
        last_logged_time = current_time

def main():
    rospy.init_node('get_turtlebot_position', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()