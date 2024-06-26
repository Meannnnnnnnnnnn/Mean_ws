#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import threading
import time

class VideoStreamNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.frame = None
        self.sub = rospy.Subscriber("/camera_topic", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Could not convert from '{msg.encoding}' to 'bgr8'. {e}")

    def get_frame(self):
        return self.frame

def stream_video(node):
    # Replace with your streaming URL
    stream_url = "rtmp://localhost/live/stream"


    # FFmpeg command
    command = [
        'ffmpeg',
        '-re', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '640x480', '-r', '30', '-i', '-',
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'ultrafast', '-f', 'flv', stream_url
    ]

    # Open a pipe to FFmpeg
    process = subprocess.Popen(command, stdin=subprocess.PIPE)

    try:
        while not rospy.is_shutdown():
            frame = node.get_frame()
            if frame is not None:
                process.stdin.write(frame.tobytes())
            time.sleep(1 / 30.0)  # Approximately 30 FPS
    except Exception as e:
        rospy.logerr(f"Error streaming video: {e}")
    finally:
        process.stdin.close()
        process.wait()

def main():
    rospy.init_node('video_stream_node')
    video_stream_node = VideoStreamNode()

    # Start the streaming thread
    streaming_thread = threading.Thread(target=stream_video, args=(video_stream_node,))
    streaming_thread.start()

    rospy.spin()
    streaming_thread.join()

if __name__ == '__main__':
    main()
