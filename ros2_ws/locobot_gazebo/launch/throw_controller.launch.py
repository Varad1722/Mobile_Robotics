"""
throw_controller.launch.py
==========================
Launches the throw_controller_node in test mode.

Test mode behaviour:
  1. Waits 3 seconds for robot + TF to settle
  2. Reads gripper position from TF (locobot/ee_gripper_link)
  3. Moves ball to gripper position using gz service
  4. Executes full throw sequence automatically

Usage:
  # Test mode (ball spawns in gripper, auto throws)
  ros2 launch locobot_gazebo throw_controller.launch.py

  # Test mode, no auto throw (manually trigger later)
  ros2 launch locobot_gazebo throw_controller.launch.py auto_throw:=false

  # Real mode (waits for /throw_trigger from pick-up node)
  ros2 launch locobot_gazebo throw_controller.launch.py test_mode:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'test_mode', default_value='true',
            description='Spawn ball in gripper for isolated throw testing'
        ),
        DeclareLaunchArgument(
            'auto_throw', default_value='true',
            description='Automatically throw after ball spawns in gripper'
        ),
        DeclareLaunchArgument(
            'wind_back_time', default_value='2.0',
            description='Seconds to hold wind-back position before throwing'
        ),
        DeclareLaunchArgument(
            'throw_time', default_value='0.4',
            description='Seconds for throw motion (keep short for speed)'
        ),
        DeclareLaunchArgument(
            'gripper_frame', default_value='locobot/ee_gripper_link',
            description='TF frame of gripper end effector'
        ),
        DeclareLaunchArgument(
            'ball_name', default_value='ball',
            description='Name of ball entity in Gazebo world'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'
        ),

        Node(
            package='locobot_nodes',
            executable='throw_controller',
            name='throw_controller',
            output='screen',
            parameters=[{
                'test_mode':      LaunchConfiguration('test_mode'),
                'auto_throw':     LaunchConfiguration('auto_throw'),
                'wind_back_time': LaunchConfiguration('wind_back_time'),
                'throw_time':     LaunchConfiguration('throw_time'),
                'gripper_frame':  LaunchConfiguration('gripper_frame'),
                'ball_name':      LaunchConfiguration('ball_name'),
                'use_sim_time':   LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
