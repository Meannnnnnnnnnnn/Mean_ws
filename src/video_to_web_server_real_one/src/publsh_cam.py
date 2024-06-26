#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_image():
    rospy.init_node('camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue

        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(image_msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
