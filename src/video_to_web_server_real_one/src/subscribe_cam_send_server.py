#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imwrite('/tmp/latest_frame.jpg', cv_image)
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def image_subscriber():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/camera/image', Image, image_callback)
    #rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass

