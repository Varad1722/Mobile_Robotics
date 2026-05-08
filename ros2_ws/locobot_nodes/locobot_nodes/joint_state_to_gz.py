#!/usr/bin/env python3
"""
joint_state_to_gz.py
====================
Bridges /joint_states from joint_state_publisher_gui
to Gazebo joint position controllers via gz transport.

When you move sliders in joint_state_publisher_gui,
this node sends the commands to Gazebo so the robot
actually moves in the simulation.

Usage:
  ros2 run locobot_nodes joint_state_to_gz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double


# Map from ROS joint name to Gazebo cmd_pos topic
JOINT_MAP = {
    'waist':        '/model/locobot/joint/waist/0/cmd_pos',
    'shoulder':     '/model/locobot/joint/shoulder/0/cmd_pos',
    'elbow':        '/model/locobot/joint/elbow/0/cmd_pos',
    'forearm_roll': '/model/locobot/joint/forearm_roll/0/cmd_pos',
    'wrist_angle':  '/model/locobot/joint/wrist_angle/0/cmd_pos',
    'wrist_rotate': '/model/locobot/joint/wrist_rotate/0/cmd_pos',
    'left_finger':  '/model/locobot/joint/left_finger/0/cmd_pos',
    'right_finger': '/model/locobot/joint/right_finger/0/cmd_pos',
}


class JointStateToGz(Node):

    def __init__(self):
        super().__init__('joint_state_to_gz')

        # Gazebo transport node + publishers
        self._gz = GzNode()
        self._pubs = {}
        for joint, topic in JOINT_MAP.items():
            self._pubs[joint] = self._gz.advertise(topic, Double)

        # Subscribe to joint states from GUI
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10
        )

        self.get_logger().info('Joint State → Gazebo bridge started')
        self.get_logger().info('Move sliders in joint_state_publisher_gui to control arm in Gazebo')

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            # Strip namespace prefix if present
            clean = name.replace('locobot/', '')
            if clean in self._pubs:
                m = Double()
                m.data = float(pos)
                self._pubs[clean].publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToGz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 