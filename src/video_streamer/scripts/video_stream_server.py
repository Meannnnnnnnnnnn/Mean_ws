import time
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading

# Global variable to store the latest frame
latest_frame = None

# HTTP request handler class
class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            if latest_frame is not None:
                ret, jpeg = cv2.imencode('.jpg', latest_frame)
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(jpeg.tobytes()))
                self.end_headers()
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b'\r\n')
            else:
                # If no frame is available, sleep briefly to avoid busy-waiting
                time.sleep(0.1)

# Function to start the HTTP server
def start_http_server():
    server_address = ('', 8080)  # Serve on all network interfaces on port 8080
    httpd = HTTPServer(server_address, StreamHandler)
    print("HTTP server running on port 8080")
    httpd.serve_forever()

# Function to capture video frames
def capture_video():
    global latest_frame
    cap = cv2.VideoCapture(1)  # Change 0 to your video source if needed
    while True:
        ret, frame = cap.read()
        if ret:
            latest_frame = frame

if __name__ == '__main__':
    # Start the video capture thread
    video_thread = threading.Thread(target=capture_video)
    video_thread.start()
    
    # Start the HTTP server
    start_http_server()
