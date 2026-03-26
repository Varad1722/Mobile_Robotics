import os
import subprocess
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("interbotix_xslocobot_descriptions")

    urdf_path = os.path.join(pkg, "urdf", "locobot.urdf.xacro")
    world_path = os.path.join(pkg, "worlds", "locobot_world.sdf")

    robot_description = subprocess.check_output([
        'xacro', urdf_path,
        'robot_model:=locobot_wx250s',
        'arm_model:=mobile_wx250s',
        'base_model:=kobuki',
        'robot_name:=locobot',
        'show_lidar:=true',
        'show_gripper_bar:=true',
        'show_gripper_fingers:=true',
    ]).decode('utf-8')

    urdf_out = '/tmp/locobot.urdf'
    with open(urdf_out, 'w') as f:
        f.write(robot_description)

    # Random ball position in the upper half of the room away from robot
    ball_x = round(random.uniform(1.5, 3.5), 2)
    ball_y = round(random.uniform(1.5, 3.5), 2)
    print(f"[INFO] Ball spawning at: x={ball_x}, y={ball_y}")

    return LaunchDescription([
        # Gazebo with custom world and Ogre1 for VM performance
        ExecuteProcess(
            cmd=[
                'gz', 'sim', '-r',
                '--render-engine-gui', 'ogre',
                '--render-engine-server', 'ogre',
                world_path
            ],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
        ),

        # Bridge all key topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        ),

        # Spawn robot at south end facing north after 5 seconds
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'service',
                        '-s', '/world/locobot_world/create',
                        '--reqtype', 'gz.msgs.EntityFactory',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000',
                        '--req', f'sdf_filename: "{urdf_out}", name: "locobot", pose: {{position: {{x: 0, y: -3.5, z: 0.1}}}}'
                    ],
                    output='screen'
                ),
            ]
        ),

        # Move ball to random position after 6 seconds
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'service',
                        '-s', '/world/locobot_world/set_pose',
                        '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000',
                        '--req', f'name: "ball", position: {{x: {ball_x}, y: {ball_y}, z: 0.1}}'
                    ],
                    output='screen'
                ),
            ]
        ),
    ])
