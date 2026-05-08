#!/usr/bin/env python3
"""
ee_logger.py
============
Subscribes to /joint_states and continuously logs:
  - Current joint angles
  - End effector position (using FK)

Use alongside joint_state_publisher_gui (RViz sliders) to find
the exact joint angles needed to reach the ball position.

Usage:
  Terminal 1: ros2 run joint_state_publisher_gui joint_state_publisher_gui
  Terminal 2: ros2 run locobot_nodes ee_logger
  Terminal 3: rviz2  (add RobotModel display)

As you move the sliders in joint_state_publisher_gui,
this node logs the joint angles and computed EE position.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ── wx250s Arm Geometry (exact from URDF) ─────────────────────────────────────
UPPER_ARM_X  = 0.04975
UPPER_ARM_Z  = 0.25
FOREARM_LEN  = 0.175 + 0.075        # = 0.25m
WRIST_TO_TIP = 0.065 + 0.043 + 0.023 + 0.027575  # = 0.158575m

# Shoulder position from base_footprint (Kobuki)
SHOULDER_Z = 0.0102 + 0.08823 + 0.0095 + 0.066175 + 0.03865  # = 0.212555m
SHOULDER_X = 0.097277


def fk_wx250s(q1, q2, q3):
    """
    FK for wx250s. Returns full chain positions relative to shoulder.
    q1 = shoulder, q2 = elbow, q3 = wrist_angle
    """
    x_el = UPPER_ARM_X * math.cos(q1) - UPPER_ARM_Z * math.sin(q1)
    z_el = UPPER_ARM_X * math.sin(q1) + UPPER_ARM_Z * math.cos(q1)

    cum12 = q1 + q2
    x_wr  = x_el + FOREARM_LEN * math.cos(cum12)
    z_wr  = z_el + FOREARM_LEN * math.sin(cum12)

    cum123 = q1 + q2 + q3
    x_grip = x_wr + WRIST_TO_TIP * math.cos(cum123)
    z_grip = z_wr + WRIST_TO_TIP * math.sin(cum123)

    return {
        'elbow':  (x_el,   z_el),
        'wrist':  (x_wr,   z_wr),
        'gripper':(x_grip, z_grip),
    }


class EELogger(Node):

    def __init__(self):
        super().__init__('ee_logger')

        # How much a joint must change before logging (avoids spam)
        self.declare_parameter('change_threshold', 0.01)
        self._threshold = self.get_parameter('change_threshold').value

        self._prev_joints = {}

        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_cb,
            10
        )

        self.get_logger().info('=' * 65)
        self.get_logger().info('EE LOGGER STARTED')
        self.get_logger().info('Listening on /joint_states...')
        self.get_logger().info('Move sliders in joint_state_publisher_gui to see FK output.')
        self.get_logger().info(f'Shoulder Z from base_footprint: {SHOULDER_Z:.6f}m')
        self.get_logger().info(f'Max arm reach: {UPPER_ARM_Z+FOREARM_LEN+WRIST_TO_TIP:.4f}m')
        self.get_logger().info('=' * 65)

    def _get_joint(self, msg, name):
        """Get joint position by name from JointState message."""
        # Try with and without 'locobot/' prefix
        for prefix in ['locobot/', '']:
            full_name = prefix + name
            if full_name in msg.name:
                idx = msg.name.index(full_name)
                return msg.position[idx] if idx < len(msg.position) else 0.0
        return 0.0

    def _joint_cb(self, msg):
        """Called on every /joint_states message."""

        # Extract joints
        joints = {
            'waist':        self._get_joint(msg, 'waist'),
            'shoulder':     self._get_joint(msg, 'shoulder'),
            'elbow':        self._get_joint(msg, 'elbow'),
            'forearm_roll': self._get_joint(msg, 'forearm_roll'),
            'wrist_angle':  self._get_joint(msg, 'wrist_angle'),
            'wrist_rotate': self._get_joint(msg, 'wrist_rotate'),
            'left_finger':  self._get_joint(msg, 'left_finger'),
        }

        # Check if any joint changed significantly
        changed = any(
            abs(joints[k] - self._prev_joints.get(k, float('inf'))) > self._threshold
            for k in joints
        )

        if not changed:
            return

        self._prev_joints = joints.copy()

        q1 = joints['shoulder']
        q2 = joints['elbow']
        q3 = joints['wrist_angle']

        # Compute FK
        fk = fk_wx250s(q1, q2, q3)
        x_grip, z_grip = fk['gripper']
        x_el,   z_el   = fk['elbow']
        x_wr,   z_wr   = fk['wrist']

        # World position of gripper (robot at origin, yaw=0)
        ee_x = SHOULDER_X + x_grip
        ee_y = 0.0
        ee_z = SHOULDER_Z + z_grip
        reach = math.sqrt(x_grip**2 + z_grip**2)

        self.get_logger().info('─' * 65)
        self.get_logger().info('JOINT ANGLES:')
        self.get_logger().info(
            f'  waist        = {joints["waist"]:+.4f} rad  '
            f'({math.degrees(joints["waist"]):+.1f}°)'
        )
        self.get_logger().info(
            f'  shoulder(q1) = {q1:+.4f} rad  '
            f'({math.degrees(q1):+.1f}°)'
        )
        self.get_logger().info(
            f'  elbow(q2)    = {q2:+.4f} rad  '
            f'({math.degrees(q2):+.1f}°)'
        )
        self.get_logger().info(
            f'  forearm_roll = {joints["forearm_roll"]:+.4f} rad  '
            f'({math.degrees(joints["forearm_roll"]):+.1f}°)'
        )
        self.get_logger().info(
            f'  wrist(q3)    = {q3:+.4f} rad  '
            f'({math.degrees(q3):+.1f}°)'
        )
        self.get_logger().info(
            f'  wrist_rotate = {joints["wrist_rotate"]:+.4f} rad  '
            f'({math.degrees(joints["wrist_rotate"]):+.1f}°)'
        )
        self.get_logger().info(
            f'  left_finger  = {joints["left_finger"]*1000:.1f} mm'
        )
        self.get_logger().info('')
        self.get_logger().info('FK POSITIONS (relative to shoulder):')
        self.get_logger().info(
            f'  elbow:   x={x_el:+.4f}m  z={z_el:+.4f}m'
        )
        self.get_logger().info(
            f'  wrist:   x={x_wr:+.4f}m  z={z_wr:+.4f}m'
        )
        self.get_logger().info(
            f'  gripper: x={x_grip:+.4f}m  z={z_grip:+.4f}m'
        )
        self.get_logger().info(
            f'  reach from shoulder: {reach:.4f}m'
        )
        self.get_logger().info('')
        self.get_logger().info('END EFFECTOR (from base_footprint, robot at origin):')
        self.get_logger().info(f'  X = {ee_x:+.4f} m  (forward)')
        self.get_logger().info(f'  Y = {ee_y:+.4f} m  (lateral)')
        self.get_logger().info(f'  Z = {ee_z:+.4f} m  (height)')
        self.get_logger().info('')
        self.get_logger().info('COPY-PASTE FOR arm_throw_node.py:')
        self.get_logger().info(
            f'  pick_q1 = {q1:.4f}  # shoulder {math.degrees(q1):.1f}°'
        )
        self.get_logger().info(
            f'  pick_q2 = {q2:.4f}  # elbow    {math.degrees(q2):.1f}°'
        )
        self.get_logger().info(
            f'  pick_q3 = {q3:.4f}  # wrist    {math.degrees(q3):.1f}°'
        )
        self.get_logger().info('─' * 65)


def main(args=None):
    rclpy.init(args=args)
    node = EELogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    