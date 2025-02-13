from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument
        DeclareLaunchArgument('loop', default_value='--loop', description='A loop parameter'),

        # Pass the launch argument as a parameter
        Node(
            package='rom_waypoints_provider',
            executable='send_waypoints_goals_loop',
            name='send_waypoints_goals_loop',
            output='screen',
            arguments=[LaunchConfiguration('loop')]  
        )
    ])
