#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import json

position_data = {"x": 0, "y": 0, "z": 0}

def odom_callback(data):
    global position_data
    position = data.pose.pose.position
    position_data = {"x": position.x, "y": position.y, "z": position.z}
    with open('/tmp/latest_position.json', 'w') as outfile:
        json.dump(position_data, outfile)

def position_subscriber():
    rospy.init_node('position_subscriber', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        position_subscriber()
    except rospy.ROSInterruptException:
        pass
