#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
import serial
import json
import threading
import time

class PoseSerialReceiver(Node):
    def __init__(self):
        super().__init__('pose_serial_receiver')
        
        # Create a QoS profile matching your C++ subscriber:
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        # Create the publisher on the same topic as your subscriber:
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/mavros/local_position/pose',
            qos_profile
        )
        
        # Open the serial port (adjust as needed)
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.get_logger().info('PoseSerialReceiver started, publishing to /mavros/local_position/pose')
        
        # Start a thread to continuously read from serial
        self.thread = threading.Thread(target=self.read_serial)
        self.thread.daemon = True
        self.thread.start()

    def read_serial(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().debug(f'Received raw line: {line}')
                    data = json.loads(line)
                    # Create a PoseStamped message
                    pose_msg = PoseStamped()

                    # Use the current time for stamp (or parse from JSON if needed)
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = data['header'].get('frame_id', '')

                    # Set position
                    pose_msg.pose.position.x = data['pose']['position']['x']
                    pose_msg.pose.position.y = data['pose']['position']['y']
                    pose_msg.pose.position.z = data['pose']['position']['z']

                    # Set orientation
                    pose_msg.pose.orientation.x = data['pose']['orientation']['x']
                    pose_msg.pose.orientation.y = data['pose']['orientation']['y']
                    pose_msg.pose.orientation.z = data['pose']['orientation']['z']
                    pose_msg.pose.orientation.w = data['pose']['orientation']['w']

                    # Publish the message
                    self.publisher_.publish(pose_msg)
                    self.get_logger().info('Published PoseStamped to /mavros/local_position/pose')
            except Exception as e:
                self.get_logger().error(f'Error processing serial data: {e}')
                # Sleep briefly to avoid spamming errors
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PoseSerialReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
