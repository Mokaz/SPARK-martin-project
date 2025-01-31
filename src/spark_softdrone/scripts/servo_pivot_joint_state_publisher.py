#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StaticJointStatePublisher(Node):
    def __init__(self):
        super().__init__('static_joint_state_publisher')
        # Create a publisher to the /joint_states topic
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Publish at 10 Hz
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Choose a fixed angle (in radians)
        self.angle = 0.0

        self.get_logger().info("Initialized static joint state publisher with angle {}".format(self.angle))

    def timer_callback(self):
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        
        # This must match the joint name in URDF
        js_msg.name = ['cad_link_to_servo_pivot']
        js_msg.position = [self.angle]
        
        self.publisher_.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
