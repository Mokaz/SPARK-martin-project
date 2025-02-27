#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
import serial
import json

class PoseSerialSender(Node):
    def __init__(self):
        super().__init__('pose_serial_sender')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        
        # Open the serial port; adjust the port name and baud rate as needed.
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.get_logger().info('PoseSerialSender started')

    def pose_callback(self, msg: PoseStamped):
        # Convert PoseStamped message to a dictionary
        data = {
            'header': {
                'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
            },
            'pose': {
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z,
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w,
                }
            }
        }
        # Serialize to JSON
        serialized = json.dumps(data)
        # Write the JSON string to the serial port followed by a newline as a delimiter
        self.ser.write((serialized + '\n').encode('utf-8'))
        self.get_logger().info(f'Sent Pose: {serialized}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseSerialSender()
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
    