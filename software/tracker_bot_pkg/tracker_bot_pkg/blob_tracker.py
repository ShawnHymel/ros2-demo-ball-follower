#!/usr/bin/env python3

import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from tracker_bot_interfaces.msg import BoundingBox

class BlobTracker(Node):
    """Captures images, detects blobs, and publishes the results."""

    def __init__(self):
        """Constructor"""
        super().__init__('blob_tracker')

        # Declare parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('flip', -1)
        self.declare_parameter('lower_color', [100, 150, 50])
        self.declare_parameter('upper_color', [140, 255, 255])
        self.declare_parameter('skip_frames', 5)
        self.declare_parameter('publish_period', 0.05)

        # Create publishers for images and bounding boxes
        self._image_pub = self.create_publisher(Image, 'image', 10)
        self._bbox_pub = self.create_publisher(BoundingBox, 'bounding_box', 10)

        # Initialize OpenCV video capture
        self._cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('width').value)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('height').value)
        self._cap.set(cv2.CAP_PROP_FPS, self.get_parameter('fps').value)

        # Set color bounds as numpy arrays
        self._lower_color = np.array(self.get_parameter('lower_color').value)
        self._upper_color = np.array(self.get_parameter('upper_color').value)

        # Bridge for converting OpenCV images to ROS messages
        self._bridge = CvBridge()

        # Frame counter
        self._frame_counter = 0
        self._skip_frames = self.get_parameter('skip_frames').value

        # Publish counter to avoid publishing no bounding boxes
        self._publish_counter = 0

        # Create a timer to periodically capture images
        self._timer = self.create_timer(
            self.get_parameter('publish_period').value,
            self._timer_callback
        )

    def __del__(self):
        """Destructor"""
        if self.__hasattr__('_cap'):
            self._cap.release()

    def _timer_callback(self):
        """Capture image, find blobs, and publish results"""

        # Read a frame from the camera
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # Flip the frame if needed
        frame = cv2.flip(frame, self.get_parameter('flip').value)

        # Blur and convert to HSV
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Create mask for blue
        mask = cv2.inRange(
            hsv, 
            self._lower_color,
            self._upper_color,
        ) 
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(
            mask.copy(), 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )

        # If any contours are found
        if contours:

            # Get the largest one
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

            # Reset publish counter
            self._publish_counter = 0

        # If no contours are found, set bounding box to negative values
        else:
            x, y, w, h = -1, -1, -1, -1
            self._publish_counter += 1
            self._publish_counter = min(self._publish_counter, 2)

        # Publish bounding box
        if self._publish_counter < 2:
            bbox_msg = BoundingBox()
            bbox_msg.x = x
            bbox_msg.y = y
            bbox_msg.width = w
            bbox_msg.height = h
            self._bbox_pub.publish(bbox_msg)

        # Every nth frame, publish the image
        self._frame_counter += 1
        if self._frame_counter % self._skip_frames == 0:

            # Draw bounding box on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"({x}, {y}, {w}, {h})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Publish the image
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self._image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BlobTracker()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()