#!/usr/bin/env python3

import serial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tracker_bot_interfaces.msg import BoundingBox

class PID:
    """A simple PID controller class."""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, limits=(None, None)):
        """Constructor"""
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._prev_error = 0
        self._integral = 0
        self._setpoint = 0
        self._limits = limits

    def set_setpoint(self, setpoint):
        """Set the desired setpoint."""
        self._setpoint = setpoint

    def reset(self):
        """Reset the PID controller."""
        self._prev_error = 0
        self._integral = 0
    
    def update(self, measurement, dt):
        """Update the PID controller with the current measurement."""

        # Calculate the error
        error = self._setpoint - measurement

        # Calculate the integral component
        self._integral += error * dt

        # Clamp the integral to prevent windup
        low, high = self._limits
        if low is not None and self._integral < low:
            self._integral = low
        if high is not None and self._integral > high:
            self._integral = high

        # Calculate the derivative component
        if dt > 0:
            derivative = (error - self._prev_error) / dt
        else:
            derivative = 0

        # Calculate the output
        output = (self._kp * error) + (self._ki * self._integral) + (self._kd * derivative)
        low, high = self._limits
        if low is not None:
            output = max(low, output)
        if high is not None:
            output = min(high, output)

        return output


class Driver(Node):
    """Processes bounding box data and sends commands to the robot."""

    def __init__(self):
        """Constructor"""
        super().__init__('driver')

        # Declare parameters
        self.declare_parameter('debug', True)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('dead_zone', 10)
        self.declare_parameter('ideal_bbox_width', 200)
        self.declare_parameter('bbox_width_zone', 0.1)
        self.declare_parameter('speed', 50)

        # Assign member variables
        self._debug = self.get_parameter('debug').value
        self._frame_width = self.get_parameter('frame_width').value
        self._frame_height = self.get_parameter('frame_height').value
        self._dead_zone = self.get_parameter('dead_zone').value
        self._ideal_bbox_width = self.get_parameter('ideal_bbox_width').value
        self._bbox_width_zone = self.get_parameter('bbox_width_zone').value
        self._speed = self.get_parameter('speed').value

        # Initialize serial port
        self._serial_port = serial.Serial(
            port=self.get_parameter('serial_port').value,
            baudrate=self.get_parameter('baud_rate').value,
            timeout=self.get_parameter('timeout').value
        )

        # Initialize PID controllers for alignment and distance to target
        self._align_pid = PID(
            kp=0.1, 
            ki=0.005, 
            kd=0.0, 
            limits=(-100, 100)
        )
        self._distance_pid = PID(
            kp=0.3, 
            ki=0.02, 
            kd=0.0, 
            limits=(-100, 100)
        )
        self._last_time = self.get_clock().now()

        # Set the setpoint for the PID controllers
        self._align_pid.set_setpoint(self._frame_width / 2)

        # Subscribe to the bounding box topic
        self.create_subscription(
            BoundingBox,
            'bounding_box',
            self._bbox_callback,
            10
        )

    def _bbox_callback(self, msg):
        """
        Callback function for processing bounding box data.
        """
        # Extract bounding box coordinates
        x = msg.x
        y = msg.y
        width = msg.width
        height = msg.height

        # Compute motor speeds and direction based on bounding box position
        if width > 0:

            # Get timestamp and compute dtime
            now = self.get_clock().now()
            dt = (now - self._last_time).nanoseconds / 1e9
            self._last_time = now

            # Find difference between bbox center and image center
            bbox_center_x = x + (width // 2)
            frame_center_x = self._frame_width // 2

            # Calculate the PID output for alignment
            align_output = self._align_pid.update(bbox_center_x, dt)
            dist_error = self._ideal_bbox_width - width
            distance_output = self._distance_pid.update(dist_error, dt)

            # Account for dead zone
            if abs(bbox_center_x - frame_center_x) < self._dead_zone:
                align_output = 0

            # Calculate left and right motor speeds
            left_speed = distance_output + align_output
            right_speed = distance_output - align_output
            left_dir = 0 if left_speed >= 0 else 1
            right_dir = 1 if right_speed >= 0 else 0
            left_speed = int(min(abs(left_speed), 100))
            right_speed = int(min(abs(right_speed), 100))

        # If the bounding box is not detected, stop the robot
        else:
            left_speed = 0
            right_speed = 0
            left_dir = 1
            right_dir = 0
            self._align_pid.reset()
            self._distance_pid.reset()
            self._last_time = self.get_clock().now()

        # Debugging info
        if self._debug:
            self.get_logger().info(
                f"bbox: [{x}, {y}, {width}, {height}], " \
                f"msg: [{left_speed}, {right_speed}, {left_dir}, {right_dir}]"
            )

        # Send commands to the robot based on bounding box data
        self._serial_port.write(f"{left_speed}, {right_speed}, {left_dir}, {right_dir}\n".encode())
        self._serial_port.flush()

def main(args=None):
    rclpy.init(args=args)
    node = Driver()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
