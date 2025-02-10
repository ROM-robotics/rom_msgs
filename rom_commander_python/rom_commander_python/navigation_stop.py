#! /usr/bin/env python3
# Copyright 2025 ROM DYNAMICS RESEARCH MYANMAR

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class NavigationStopPublisher(Node):
    def __init__(self):
        super().__init__('navigation_stop_publisher')
        self.publisher = self.create_publisher(Bool, 'navigation_stop', 10)  # Topic: 'navigation_stop'
        self.counter = 0

    def publish_stop_signal(self):
        """Publish True or False to the navigation_stop topic and then exit."""
        msg = Bool()

        # Publish True first (Stop signal)
        if self.counter == 0:
            msg.data = True
            self.get_logger().info('Publishing: STOP navigation (True)')
        
        # # Publish False second (Continue signal)
        # elif self.counter == 1:
        #     msg.data = False
        #     self.get_logger().info('Publishing: CONTINUE navigation (False)')
        
        self.publisher.publish(msg)
        self.counter += 1
        
        # Exit after sending 2 messages (True and False)
        if self.counter > 1:
            self.get_logger().info('Exiting publisher after sending stop signals.')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = NavigationStopPublisher()
    
    # Publish stop signals once or twice
    node.publish_stop_signal()
    node.publish_stop_signal()
    
    # Keep the node alive until the signals are sent and then exit
    rclpy.spin(node)

if __name__ == '__main__':
    main()
