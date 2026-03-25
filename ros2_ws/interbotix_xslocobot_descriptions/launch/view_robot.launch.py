import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("interbotix_xslocobot_descriptions"),
        "urdf", "locobot.urdf.xacro"
    )

    robot_description = ParameterValue(Command([
        "xacro ", urdf_path,
        " robot_model:=locobot_wx250s",
        " arm_model:=mobile_wx250s",
        " base_model:=kobuki",
        " robot_name:=locobot",
        " show_lidar:=true",
        " show_gripper_bar:=true",
        " show_gripper_fingers:=true",
    ]), value_type=str)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
        ),
    ])
