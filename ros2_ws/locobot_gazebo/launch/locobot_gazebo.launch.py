import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    desc_pkg   = get_package_share_directory('interbotix_xslocobot_descriptions')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(desc_pkg, 'worlds', 'locobot_world.sdf')
    urdf_path  = os.path.join(desc_pkg, 'urdf', 'locobot.urdf.xacro')
    mesh_path  = os.path.join(desc_pkg, '..')

    result = subprocess.run([
        'xacro', urdf_path,
        'robot_model:=locobot_wx250s',
        'arm_model:=mobile_wx250s',
        'base_model:=kobuki',
        'robot_name:=locobot',
        'show_lidar:=true',
        'show_gripper_bar:=true',
        'show_gripper_fingers:=true'
    ], capture_output=True, text=True)

    urdf = result.stdout.replace(
        'package://interbotix_xslocobot_descriptions',
        desc_pkg
    )

    resolved_urdf_path = '/tmp/locobot_resolved.urdf'
    with open(resolved_urdf_path, 'w') as f:
        f.write(urdf)

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=mesh_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r ' + world_file}.items(),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf,
                'use_sim_time': True
            }],
        ),

        # Spawn robot after Gazebo loads
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-name', 'locobot',
                        '-file', resolved_urdf_path,
                        '-x', '-4.0', '-y', '-4.0', '-z', '0.15',
                    ],
                    output='screen',
                ),
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/locobot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                ('/model/locobot/tf', '/tf'),
            ],
            output='screen',
        ),

# lidar_bridge removed - using gz_lidar_sim instead


        # Depth camera bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_bridge',
            arguments=[
                '/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            output='screen',
        ),

        # Convert depth image to LaserScan for Nav2
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/depth'),
                ('depth_camera_info', '/camera/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'scan_height': 10,
                'range_min': 0.2,
                'range_max': 10.0,
                'use_sim_time': True,
                'output_frame': 'locobot/depth_camera_link',
            }],
            output='screen',
        ),


# odom_tf_broadcaster removed - using gz bridge /model/locobot/tf

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(os.path.expanduser('~'), 'mobile_manipulation_ws', 'src', 'locobot_gazebo', 'config', 'locobot_nav.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
