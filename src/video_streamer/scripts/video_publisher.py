import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def video_publisher():
    rospy.init_node('video_publisher', anonymous=True)
    video_pub = rospy.Publisher('video_stream', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(1)  # Change 0 to your video source if needed

    rate = rospy.Rate(10)  # Publish at 10 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            video_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
