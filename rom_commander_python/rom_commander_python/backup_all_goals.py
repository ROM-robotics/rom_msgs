#! /usr/bin/env python3
# Copyright 2025 ROM DYNAMICS RESEARCH MYANMAR

import os
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

"""
Basic navigation demo to go to poses from a YAML file.
"""

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

    navigator = BasicNavigator()
    
    package_share_directory = get_package_share_directory('rom_waypoints_provider')
    
    yaml_file_path = os.path.join(package_share_directory, 'config/waypoints.yaml')
    
    waypoints = load_waypoints_from_yaml(yaml_file_path)
    
    # သူက amcl ရဲ့ init pose ကို စောင့်တာမို့ comment ထားမယ်။
    #initial_pose = create_pose_from_waypoint(waypoints[0], navigator)
    #navigator.setInitialPose(initial_pose)
    
    navigator.waitUntilNav2Active()
    
    goal_poses = []
    
    for waypoint in waypoints:
        goal_pose = create_pose_from_waypoint(waypoint, navigator)
        goal_poses.append(goal_pose)

    # Start navigation to the goal poses
    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # feedback ကို ၅ ကြိမ်မှ တခါ print ပြချင်တာ
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # 10 minute ကျော်လို့မှမရောက်ရင်တော့ cancel လုပ်မယ်။
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # 35 seconds ကျော်လို့မှာ goal ကို မရောက်ရင် နောက် goal ကို သွားတော့။
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose4.pose.position.x = 0.0
                goal_pose4.pose.position.y = 0.0
                goal_pose4.pose.orientation.w = 1.0
                goal_pose4.pose.orientation.z = 0.0
                navigator.goThroughPoses([goal_pose4])

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)

if __name__ == '__main__':
    main()
