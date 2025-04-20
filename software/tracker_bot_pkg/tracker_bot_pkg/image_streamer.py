#!/usr/bin/env python3

import cv2
from flask import Flask, Response

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from threading import Thread, Lock

# Flask app for streaming video
app = Flask(__name__)
latest_frame = None
frame_lock = Lock()

class ImageStreamer(Node):
    """Captures images from topic and serves them via Flask"""

    def __init__(self):
        """Constructor"""
        super().__init__('image_streamer')

        # Initialize subsciption to the image topic
        self.subscriber = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10
        )

        # Bridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """
        Each time a new image is received, convert it to OpenCV format and 
        store it
        """
        global latest_frame

        # Convert ROS image message to OpenCV format and store it
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with frame_lock:
            latest_frame = frame.copy()

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            with frame_lock:
                if latest_frame is None:
                    continue
                _, buffer = cv2.imencode('.jpg', latest_frame)
                jpg = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '<h1>Live Video</h1><img src="/video_feed">'

def flask_thread():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImageStreamer()
        thread = Thread(target=flask_thread, daemon=True)
        thread.start()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
