#!/usr/bin/env python3
"""
throw_controller_node.py
========================
Tests the throw functionality in isolation WITHOUT needing the pick-up node.

Test Mode (default):
  - Spawns the ball directly inside the robot's gripper (ee_gripper_link frame)
  - Executes the throw sequence automatically

Real Mode (when pick-up node is done):
  - Listens for a trigger on /throw_trigger (std_msgs/Bool)
  - Executes throw sequence when triggered

Throw Sequence:
  1. Ball spawns at gripper position (test mode only)
  2. Arm moves to throw-ready position (raised + retracted)
  3. Arm flings forward fast (catapult motion)
  4. Gripper opens at peak velocity → ball releases
  5. Arm returns to home position
  6. Logs throw distance (estimated from arm velocity + angle)

Topics:
  Subscribes:  /throw_trigger        (std_msgs/Bool)      — trigger from pick-up node
               /joint_states         (sensor_msgs/JointState) — arm joint feedback
               /tf                   (tf2)                — gripper position
  Publishes:   /joint_group_command  (std_msgs/Float64MultiArray) — arm joint targets
               /throw_status         (std_msgs/String)    — status updates

Arm joint order for wx250s:
  [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
  Gripper: [left_finger, right_finger]
"""

import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState

import tf2_ros

# Gazebo transport for direct world pose lookup
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except ImportError:
    GZ_AVAILABLE = False


# ── Arm Joint Positions (radians) for wx250s ─────────────────────────────────

# Home / safe position
HOME = [0.0, -1.80, 1.55, 0.0, 0.80, 0.0]

# Step 1: Wind-back position — arm pulled back ready to throw
WIND_BACK = [0.0, -0.5, -0.8, 0.0, 1.2, 0.0]

# Step 2: Throw position — arm flings forward fast
THROW_RELEASE = [0.0, 1.2, -0.5, 0.0, -0.8, 0.0]

# Gripper open (release ball) — positive = open
GRIPPER_OPEN   = [0.057, -0.057]   # fully open (metres for finger joints)
GRIPPER_CLOSED = [0.0,    0.0]     # closed / gripping

# Joint names for wx250s
ARM_JOINT_NAMES = [
    'locobot/waist',
    'locobot/shoulder',
    'locobot/elbow',
    'locobot/forearm_roll',
    'locobot/wrist_angle',
    'locobot/wrist_rotate',
]
GRIPPER_JOINT_NAMES = [
    'locobot/left_finger',
    'locobot/right_finger',
]


class ThrowControllerNode(Node):

    def __init__(self):
        super().__init__('throw_controller')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('test_mode',         True)   # spawn ball in gripper
        self.declare_parameter('auto_throw',        True)   # throw immediately in test mode
        self.declare_parameter('wind_back_time',    2.0)    # seconds for wind-back motion
        self.declare_parameter('throw_time',        0.4)    # seconds for throw motion (fast!)
        self.declare_parameter('gripper_delay',     0.15)   # open gripper this far into throw
        self.declare_parameter('ball_name',         'ball')
        self.declare_parameter('gripper_frame',     'locobot/ee_gripper_link')

        self._test_mode     = self.get_parameter('test_mode').value
        self._auto_throw    = self.get_parameter('auto_throw').value
        self._wind_back_t   = self.get_parameter('wind_back_time').value
        self._throw_t       = self.get_parameter('throw_time').value
        self._gripper_delay = self.get_parameter('gripper_delay').value
        self._ball_name     = self.get_parameter('ball_name').value
        self._gripper_frame = self.get_parameter('gripper_frame').value

        # ── State ────────────────────────────────────────────────────────────
        self._joint_states   = {}
        self._throw_done     = False
        self._sequence_step  = 0
        # Robot world position from Gazebo transport (most accurate)
        self._robot_world_x  = 0.0
        self._robot_world_y  = 0.0
        self._robot_world_z  = 0.0
        self._robot_yaw      = 0.0
        self._gz_pose_ready  = False

        # ── Gazebo Transport ─────────────────────────────────────────────────
        if GZ_AVAILABLE:
            self._gz_node = GzNode()
            self._gz_node.subscribe(
                Pose_V,
                '/world/locobot_world/pose/info',
                self._gz_pose_callback
            )
            self.get_logger().info('Gazebo transport connected for robot pose')
        else:
            self.get_logger().warn('Gazebo transport not available')

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publishers ───────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self._arm_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_command', qos
        )
        self._status_pub = self.create_publisher(
            String, '/throw_status', qos
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, qos
        )

        self.create_subscription(
            Bool, '/throw_trigger',
            self._trigger_cb, qos
        )

        self.get_logger().info(
            f'ThrowController started | '
            f'test_mode={self._test_mode} | '
            f'auto_throw={self._auto_throw} | '
            f'gripper_frame={self._gripper_frame}'
        )

        if self._test_mode:
            self.get_logger().info(
                'TEST MODE: Ball will spawn in gripper in 6 seconds (waiting for odom)...'
            )
            # Wait 3s for everything to settle, then spawn ball and throw
            self._spawn_timer = self.create_timer(6.0, self._test_mode_start)
        else:
            self.get_logger().info(
                'REAL MODE: Waiting for /throw_trigger from pick-up node...'
            )

    # ── Gazebo Pose Callback ─────────────────────────────────────────────────

    def _gz_pose_callback(self, msg):
        """Get robot world position directly from Gazebo — most accurate."""
        for pose in msg.pose:
            if pose.name == 'locobot':
                self._robot_world_x = pose.position.x
                self._robot_world_y = pose.position.y
                self._robot_world_z = pose.position.z
                # Extract yaw from quaternion
                q = pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self._robot_yaw = math.atan2(siny, cosy)
                self._gz_pose_ready = True
                break

    # ── Joint State Feedback ──────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_states[name] = pos

    # ── Test Mode: Spawn ball in gripper ─────────────────────────────────────

    def _test_mode_start(self):
        """Called once after 6s — spawns ball at gripper position."""
        # Cancel this repeating timer — one shot only
        self._spawn_timer.cancel()

        # Safety check — make sure Gazebo pose has been received
        if not self._gz_pose_ready:
            self.get_logger().warn(
                'Gazebo pose not received yet! Retrying in 2 seconds...'
            )
            self._spawn_timer = self.create_timer(2.0, self._test_mode_start)
            return

        gripper_pos = self._get_gripper_world_position()
        if gripper_pos is None:
            self.get_logger().warn(
                'Could not get gripper TF position. '
                'Using fallback position (0, -3.5, 0.5). '
                'Make sure the robot is spawned and TF is broadcasting.'
            )
            gx, gy, gz = 0.0, -3.5, 0.5
        else:
            gx, gy, gz = gripper_pos
            self.get_logger().info(
                f'Gripper world position: ({gx:.3f}, {gy:.3f}, {gz:.3f})'
            )

        self._spawn_ball_at(gx, gy, gz)

        if self._auto_throw:
            self.get_logger().info('Auto-throw enabled — starting throw sequence in 1s...')
            self._auto_throw_timer = self.create_timer(1.0, self._start_throw_sequence)

    def _get_gripper_world_position(self):
        """
        Get exact midpoint between left and right finger links.

        From TF measurements with arm in home position:
          left_finger:  (0.528,  0.026, 0.453) from base_link
          right_finger: (0.528, -0.026, 0.453) from base_link
          midpoint:     (0.528,  0.000, 0.453) from base_link

        This midpoint is rotated by the robot yaw and added to
        the robot world position from Gazebo transport.
        """
        try:
            # Get left finger position in base_link frame
            tl = self._tf_buffer.lookup_transform(
                'locobot/base_link',
                'locobot/left_finger_link',
                rclpy.time.Time()
            )
            # Get right finger position in base_link frame
            tr = self._tf_buffer.lookup_transform(
                'locobot/base_link',
                'locobot/right_finger_link',
                rclpy.time.Time()
            )

            # Midpoint between fingers = exact ball placement point
            offset_x = (tl.transform.translation.x + tr.transform.translation.x) / 2.0
            offset_y = (tl.transform.translation.y + tr.transform.translation.y) / 2.0
            offset_z = (tl.transform.translation.z + tr.transform.translation.z) / 2.0

            self.get_logger().info(
                f'Left finger:  ({tl.transform.translation.x:.3f}, {tl.transform.translation.y:.3f}, {tl.transform.translation.z:.3f})'
            )
            self.get_logger().info(
                f'Right finger: ({tr.transform.translation.x:.3f}, {tr.transform.translation.y:.3f}, {tr.transform.translation.z:.3f})'
            )
            self.get_logger().info(
                f'Midpoint:     ({offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f})'
            )

            # Rotate offset by robot yaw to get world-frame offset
            yaw = self._robot_yaw
            rotated_x = offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
            rotated_y = offset_x * math.sin(yaw) + offset_y * math.cos(yaw)

            # Add robot world position from Gazebo
            world_x = self._robot_world_x + rotated_x
            world_y = self._robot_world_y + rotated_y
            world_z = self._robot_world_z + offset_z

            self.get_logger().info(
                f'Robot world: ({self._robot_world_x:.3f}, {self._robot_world_y:.3f}) | '
                f'Yaw: {math.degrees(yaw):.1f}° | '
                f'Ball world pos: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})'
            )
            return world_x, world_y, world_z

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            self.get_logger().warn('Using fallback: robot pos + fixed finger midpoint offset')
            # Fixed offsets from TF measurements: midpoint at (0.528, 0.0, 0.453)
            yaw = self._robot_yaw
            offset_x, offset_y, offset_z = 0.528, 0.0, 0.453
            rotated_x = offset_x * math.cos(yaw) - offset_y * math.sin(yaw)
            rotated_y = offset_x * math.sin(yaw) + offset_y * math.cos(yaw)
            return (
                self._robot_world_x + rotated_x,
                self._robot_world_y + rotated_y,
                self._robot_world_z + offset_z
            )

    def _spawn_ball_at(self, x: float, y: float, z: float):
        """Spawn or move ball to given world coordinates using gz service."""
        self.get_logger().info(
            f'Spawning ball at gripper position: ({x:.3f}, {y:.3f}, {z:.3f})'
        )
        self._publish_status('SPAWNING_BALL')

        # Try set_pose first (ball already exists in world)
        set_pose_cmd = [
            'gz', 'service',
            '-s', '/world/locobot_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req',
            f'name: "{self._ball_name}", '
            f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'
        ]

        try:
            result = subprocess.run(
                set_pose_cmd, capture_output=True, text=True, timeout=5.0
            )
            if 'true' in result.stdout.lower():
                self.get_logger().info(
                    f'Ball moved to gripper position ✅ ({x:.3f}, {y:.3f}, {z:.3f})'
                )
                self._publish_status('BALL_IN_GRIPPER')
                return
        except Exception as e:
            self.get_logger().warn(f'set_pose failed: {e}')

        self.get_logger().error(
            'Could not move ball to gripper. '
            'Make sure the ball exists in the Gazebo world.'
        )

    # ── Trigger from Pick-up Node (Real Mode) ─────────────────────────────────

    def _trigger_cb(self, msg: Bool):
        """Called by pick-up node when ball is gripped and ready to throw."""
        if msg.data and not self._throw_done:
            self.get_logger().info(
                'Throw trigger received from pick-up node! Starting throw...'
            )
            self._start_throw_sequence()

    # ── Throw Sequence ────────────────────────────────────────────────────────

    def _start_throw_sequence(self):
        """Step 1: Move arm to wind-back position."""
        if self._throw_done:
            return
        # Cancel auto-throw timer — one shot only
        if hasattr(self, '_auto_throw_timer'):
            self._auto_throw_timer.cancel()

        self.get_logger().info('=== THROW SEQUENCE STARTED ===')
        self.get_logger().info('Step 1: Moving to wind-back position...')
        self._publish_status('WIND_BACK')

        self._send_arm_command(WIND_BACK)

        # After wind-back settles, execute throw (one shot)
        self._wind_back_timer = self.create_timer(self._wind_back_t, self._execute_throw)

    def _execute_throw(self):
        """Step 2: Fling arm forward fast."""
        # Cancel wind-back timer — one shot only
        self._wind_back_timer.cancel()

        self.get_logger().info('Step 2: THROWING — arm flinging forward!')
        self._publish_status('THROWING')

        self._send_arm_command(THROW_RELEASE)

        # Open gripper slightly into the throw motion for best release (one shot)
        self._gripper_timer = self.create_timer(self._gripper_delay, self._open_gripper)

        # Return home after throw completes (one shot)
        self._home_timer = self.create_timer(self._throw_t + 0.5, self._return_home)

    def _open_gripper(self):
        """Step 3: Open gripper to release ball at peak arm velocity."""
        # Cancel — one shot only
        self._gripper_timer.cancel()
        self.get_logger().info('Step 3: Gripper OPEN — ball released!')
        self._publish_status('BALL_RELEASED')
        self._send_gripper_command(GRIPPER_OPEN)

    def _return_home(self):
        """Step 4: Return arm to home position."""
        # Cancel — one shot only
        self._home_timer.cancel()
        self.get_logger().info('Step 4: Returning arm to home position.')
        self._publish_status('RETURNING_HOME')
        self._send_arm_command(HOME)
        self._throw_done = True
        self.get_logger().info('=== THROW SEQUENCE COMPLETE ===')
        self._publish_status('THROW_COMPLETE')

    # ── Joint Commands ────────────────────────────────────────────────────────

    def _send_arm_command(self, positions: list):
        """Send joint position targets to the arm."""
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self._arm_pub.publish(msg)
        self.get_logger().debug(
            f'Arm command: {[f"{p:.2f}" for p in positions]}'
        )

    def _send_gripper_command(self, positions: list):
        """Send gripper finger position targets."""
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        self._arm_pub.publish(msg)

    # ── Status ────────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f'[STATUS] {status}')


# ── Entry Point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ThrowControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
