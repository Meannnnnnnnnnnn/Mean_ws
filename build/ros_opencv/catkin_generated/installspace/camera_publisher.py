#!/use/bin/env python3
#here, we are saying to ROS that this is a python source file

#Here we import rospy that enables us to use ROS with Python
import rospy

#We will send messages in form of images consequentfly, we need to import Image
from sensor_msgs.msg import Image

#cvbrideg is a package that consists of a library for converting OpenCV images(of type cv::Mat)
#into a ROS images message and for conerting Ros image message back to OpenCV images.
#That is, it serves asa a bridge between OpenCV and ROS
from cv_bridge import CvBridge
#here we import OpenCV
import cv2

#create the name of our publisher node - change as you wish
publisherNodeName= 'camera_sensor_publisher'
#create the name of our topic over with we will transmit the image messages
#make sure that the same name is used in the source file of the subscriber
topicName='video_topic'

#initialize the node
rospy.init_node(publisherNodeName, anonymous=True)
#create a publisher object, specify the name of the topic, a type of the message being 
#sent (Image), and define the buffer size (queue-size)
publisher=rospy.Publisher(topicName, Image, queue_size=60)
#rate of transmitting the messages
rate = rospy.Rate(30)

#create the video capture object
videoCaptureObject = cv2.VideoCapture(0)

#create the CvBridge object that will be used to convert OpenCV Images to ros messages
bridgeObject=CvBridge()


#here is where the magic happens
#this is an infinite loop that captures the images and transmits them through the topic

while not rospy.is_shutdown():
    #returns two values, the first value is the boolean for success/failure
    #the second one is the actual frame
    returnValue, capturedFrame = videoCaptureObject.read()
    #everything is OK, transmit
    if returnValue == True:
        #print the message
        rospy.loginfo('Video frame captured and published')
        #convert OpenCV to ROS image message
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame)
        #publish the converted image throught the topic
        publisher.publish(imageToTransmit)
    #here wait for certain time to make sure that the specified transmission rate is achieved
    rate.sleep()