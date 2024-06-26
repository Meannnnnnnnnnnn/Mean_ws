#!/usr/bin/env python3


from flask import Flask, Response, request, render_template_string
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import threading

app = Flask(__name__)

class WebVideoServer:
    def __init__(self):
        rospy.init_node('web_video_server', anonymous=True)
        self.bridge = CvBridge()
        self.image_subscribers = {}
        self.lock = threading.Lock()
        self.default_stream_type = rospy.get_param("~default_stream_type", "mjpeg")
        self.verbose = rospy.get_param("~verbose", True)
        self.publish_rate = rospy.get_param("~publish_rate", -1.0)
        self.port = rospy.get_param("~port", 8080)
        self.address = rospy.get_param("~address", "0.0.0.0")
        self.server_threads = rospy.get_param("~server_threads", 1)
        self.ros_threads = rospy.get_param("~ros_threads", 2)
        
    def run(self):
        self.start_ros_spinner()
        app.run(host=self.address, port=self.port, threaded=True)

    def start_ros_spinner(self):
        if self.publish_rate > 0:
            rate = rospy.Rate(self.publish_rate)
            while not rospy.is_shutdown():
                self.restream_frames(1.0 / self.publish_rate)
                rate.sleep()
        else:
            rospy.spin()

    def restream_frames(self, max_age):
        with self.lock:
            for topic, subscriber in self.image_subscribers.items():
                subscriber.restream_frame(max_age)

    def cleanup_inactive_streams(self):
        with self.lock:
            inactive_topics = [topic for topic, subscriber in self.image_subscribers.items() if subscriber.is_inactive()]
            for topic in inactive_topics:
                if self.verbose:
                    rospy.loginfo(f"Removed Stream: {topic}")
                del self.image_subscribers[topic]

    def add_image_subscriber(self, topic, streamer):
        with self.lock:
            self.image_subscribers[topic] = streamer

web_video_server = WebVideoServer()

class ImageStreamer:
    def __init__(self, topic):
        self.topic = topic
        self.frame = None
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(topic, Image, self.callback)

    def callback(self, data):
        with self.lock:
            self.frame = web_video_server.bridge.imgmsg_to_cv2(data, "bgr8")

    def get_frame(self):
        with self.lock:
            return self.frame

    def is_inactive(self):
        # Implement inactivity check logic
        return False

    def restream_frame(self, max_age):
        # Implement restream logic
        pass

@app.route('/')
def list_streams():
    topics = rospy.get_published_topics()
    image_topics = [topic for topic, msg_type in topics if msg_type == 'sensor_msgs/Image']
    response = "<html><head><title>ROS Image Topic List</title></head><body><h1>Available ROS Image Topics:</h1><ul>"
    for topic in image_topics:
        response += f'<li><a href="/stream_viewer?topic={topic}">{topic}</a> (<a href="/snapshot?topic={topic}">Snapshot</a>)</li>'
    response += "</ul></body></html>"
    return response

@app.route('/stream')
def stream():
    topic = request.args.get('topic')
    if topic:
        streamer = ImageStreamer(topic)
        web_video_server.add_image_subscriber(topic, streamer)
        return Response(generate_frame(streamer), mimetype='multipart/x-mixed-replace; boundary=frame')
    return "Topic not found", 404

@app.route('/snapshot')
def snapshot():
    topic = request.args.get('topic')
    if topic:
        streamer = ImageStreamer(topic)
        web_video_server.add_image_subscriber(topic, streamer)
        frame = streamer.get_frame()
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            return Response(jpeg.tobytes(), mimetype='image/jpeg')
    return "Topic not found", 404

@app.route('/stream_viewer')
def stream_viewer():
    topic = request.args.get('topic')
    if topic:
        streamer = ImageStreamer(topic)
        web_video_server.add_image_subscriber(topic, streamer)
        return render_template_string('''
        <html><head><title>{{ topic }}</title></head>
        <body><h1>{{ topic }}</h1>
        <img src="/stream?topic={{ topic }}" />
        </body></html>
        ''', topic=topic)
    return "Topic not found", 404

def generate_frame(streamer):
    while True:
        frame = streamer.get_frame()
        if frame is not None:
            ret, jpeg = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')
        else:
            rospy.sleep(0.1)

if __name__ == '__main__':
    web_video_server.run()
