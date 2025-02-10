#! /usr/bin/env python3
# Copyright 2025 ROM DYNAMICS RESEARCH MYANMAR

import os
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool

"""
Basic navigation demo to go to poses from a YAML file, loop through all waypoints repeatedly,
and stop when receiving a stop signal via a topic.
"""

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.stop_navigation = False
        
        # Subscriber for the navigation stop signal
        self.stop_subscriber = self.create_subscription(
            Bool,
            'navigation_stop',  # ဒီ topic name နဲ့ 
            self.navigation_stop_callback,  # Callback function
            10  # QoS History depth
        )

    def navigation_stop_callback(self, msg):
        """Callback function to stop navigation when receiving the stop signal."""
        if msg.data:  # If the message is True, stop navigation
            self.stop_navigation = True
            self.get_logger().info('Navigation stop signal received!')

def load_waypoints_from_yaml(yaml_file):
    """Load waypoints from a YAML file."""
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    return data['waypoints']

def create_pose_from_waypoint(waypoint, navigator):
    """Create a PoseStamped message from a waypoint dictionary."""
    pose = PoseStamped()
    pose.header.frame_id = waypoint['frame_id']
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = waypoint['pose']['position']['x']
    pose.pose.position.y = waypoint['pose']['position']['y']
    pose.pose.position.z = waypoint['pose']['position']['z']
    pose.pose.orientation.x = waypoint['pose']['orientation']['x']
    pose.pose.orientation.y = waypoint['pose']['orientation']['y']
    pose.pose.orientation.z = waypoint['pose']['orientation']['z']
    pose.pose.orientation.w = waypoint['pose']['orientation']['w']
    return pose

def main():
    rclpy.init()

    # Create the node to listen for stop signal
    navigation_node = NavigationNode()

    navigator = BasicNavigator()
    
    package_share_directory = get_package_share_directory('rom_waypoints_provider')
    
    yaml_file_path = os.path.join(package_share_directory, 'config/waypoints.yaml')
    
    waypoints = load_waypoints_from_yaml(yaml_file_path)
    
    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()
    
    # Infinite loop to repeatedly go through all waypoints
    while rclpy.ok() and not navigation_node.stop_navigation:
        for waypoint in waypoints:
            goal_pose = create_pose_from_waypoint(waypoint, navigator)

            # Go to the current waypoint
            navigator.goToPose(goal_pose)

            # Monitor the status of the navigation
            while not navigator.isTaskComplete() and not navigation_node.stop_navigation:
                feedback = navigator.getFeedback()
                if feedback:
                    print(
                        'Estimated time of arrival: '
                        + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                            / 1e9
                        )
                        + ' seconds.'
                    )

                    # If navigation takes too long, cancel the task
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelTask()

            # After reaching the goal, check the result
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Arrived at {waypoint["name"]}!')
            elif result == TaskResult.CANCELED:
                print(f'Navigation to {waypoint["name"]} was canceled!')
            elif result == TaskResult.FAILED:
                print(f'Navigation to {waypoint["name"]} failed!')
            else:
                print(f'Goal {waypoint["name"]} has an invalid return status!')

        # To keep going after completing one round of waypoints, check if stop signal was received
        if navigation_node.stop_navigation:
            print("Navigation stopped.")
            break

    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()
