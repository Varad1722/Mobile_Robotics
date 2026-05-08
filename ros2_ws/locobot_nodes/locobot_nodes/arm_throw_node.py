#!/usr/bin/env python3
"""
arm_throw_node.py
=================
Full sequence with hardcoded pickup angles from experiment.

Pickup angles determined using joint_state_publisher_gui + ee_logger:
  q1 (shoulder) = 0.6704 rad (38.4°)
  q2 (elbow)    = -0.0007 rad (~0°)
  q3 (wrist)    = -0.0001 rad (~0°)

EE position at pickup angles:
  X = 0.3013m forward from base_footprint
  Z = 0.6932m height from base_footprint

Ball spawns at exact gripper position so pickup is accurate.
"""

import math
import time
import threading
import subprocess
import rclpy
from rclpy.node import Node

from gz.transport13 import Node as GzNode
from gz.msgs10.double_pb2 import Double

from visualization_msgs.msg import Marker, MarkerArray


# ── Hardcoded Pickup Angles (from experiment) ─────────────────────────────────
PICK_Q1 =  0.6704    # shoulder 38.4°
PICK_Q2 = -0.0007    # elbow    ~0°
PICK_Q3 = -0.0001    # wrist    ~0°

# ── Ball Spawn Position (exact EE position at pickup angles) ──────────────────
# From ee_logger: X=0.3013m, Z=0.6932m from base_footprint
BALL_OFFSET_X = 0.3013   # forward from robot base_footprint
BALL_OFFSET_Z = 0.6932   # height from robot base_footprint

# ── Fixed Positions ───────────────────────────────────────────────────────────
WAIST_FIXED     =  0.0
WRIST_ROT_FIXED =  0.0
SHOULDER_HOME   =  0.5
SHOULDER_WINDUP = -1.2
SHOULDER_THROW  =  1.5
ELBOW_HOME      = -1.0
WRIST_HOME      =  0.5

# Gripper (prismatic, meters)
GRIPPER_OPEN   = 0.037
GRIPPER_CLOSED = 0.015
GRIPPER_TIGHT  = 0.016

# Throw elbow range
THROW_ELBOW_MIN = -1.8
THROW_ELBOW_MAX = -0.3
THROW_DIST_MIN  =  0.5
THROW_DIST_MAX  =  5.0

# Slow movement
SLOW_STEPS    = 30
SLOW_DURATION = 4.0


def throw_elbow_angle(distance):
    distance = max(THROW_DIST_MIN, min(THROW_DIST_MAX, distance))
    t = (distance - THROW_DIST_MIN) / (THROW_DIST_MAX - THROW_DIST_MIN)
    return THROW_ELBOW_MIN + t * (THROW_ELBOW_MAX - THROW_ELBOW_MIN)


class ArmThrowNode(Node):

    def __init__(self):
        super().__init__('arm_throw')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('target_distance',  3.5)
        self.declare_parameter('elbow_angle',     -1.0)
        self.declare_parameter('use_manual_angle', False)
        self.declare_parameter('wind_back_time',   2.0)
        self.declare_parameter('throw_speed_time', 0.4)
        self.declare_parameter('ball_name',        'ball')

        self._target_dist   = self.get_parameter('target_distance').value
        self._manual_elbow  = self.get_parameter('elbow_angle').value
        self._use_manual    = self.get_parameter('use_manual_angle').value
        self._wind_back_t   = self.get_parameter('wind_back_time').value
        self._throw_speed_t = self.get_parameter('throw_speed_time').value
        self._ball_name     = self.get_parameter('ball_name').value

        # ── State ────────────────────────────────────────────────────────────
        self._robot_x    = -4.0
        self._robot_y    = -4.0
        self._robot_z    =  0.0
        self._robot_yaw  =  0.0
        self._pose_ready = False
        self._ball_world = (0.0, 0.0, 0.0)

        self._cur_q1 = SHOULDER_HOME
        self._cur_q2 = ELBOW_HOME
        self._cur_q3 = WRIST_HOME
        self._cur_w  = WAIST_FIXED

        # ── RViz marker publisher ─────────────────────────────────────────────
        self._marker_pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )

        # ── Gazebo transport ─────────────────────────────────────────────────
        self._gz = GzNode()
        self._pub_waist   = self._gz.advertise('/model/locobot/joint/waist/0/cmd_pos',        Double)
        self._pub_q1      = self._gz.advertise('/model/locobot/joint/shoulder/0/cmd_pos',     Double)
        self._pub_q2      = self._gz.advertise('/model/locobot/joint/elbow/0/cmd_pos',        Double)
        self._pub_q3      = self._gz.advertise('/model/locobot/joint/wrist_angle/0/cmd_pos',  Double)
        self._pub_wr      = self._gz.advertise('/model/locobot/joint/wrist_rotate/0/cmd_pos', Double)
        self._pub_lf      = self._gz.advertise('/model/locobot/joint/left_finger/0/cmd_pos',  Double)
        self._pub_rf      = self._gz.advertise('/model/locobot/joint/right_finger/0/cmd_pos', Double)

        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            self._gz.subscribe(Pose_V, '/world/locobot_world/pose/info', self._pose_cb)
            self.get_logger().info('Subscribed to Gazebo pose info')
        except Exception as e:
            self.get_logger().warn(f'Pose sub failed: {e}')

        self._throw_elbow = (self._manual_elbow if self._use_manual
                             else throw_elbow_angle(self._target_dist))

        self.get_logger().info('=== ARM THROW NODE STARTED ===')
        self.get_logger().info(
            f'Target: {self._target_dist:.2f}m | '
            f'Throw elbow: {self._throw_elbow:.3f} rad ({math.degrees(self._throw_elbow):.1f}°)'
        )
        self.get_logger().info(
            f'Pickup angles (from experiment): '
            f'q1={PICK_Q1:.4f} q2={PICK_Q2:.4f} q3={PICK_Q3:.4f}'
        )
        self.get_logger().info(
            f'Ball spawn: X={BALL_OFFSET_X:.4f}m Z={BALL_OFFSET_Z:.4f}m '
            f'from base_footprint'
        )

        self._thread = threading.Thread(target=self._run_sequence)
        self._thread.daemon = True
        self._thread.start()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == 'locobot':
                self._robot_x   = pose.position.x
                self._robot_y   = pose.position.y
                self._robot_z   = pose.position.z
                q = pose.orientation
                self._robot_yaw = math.atan2(
                    2*(q.w*q.z + q.x*q.y),
                    1 - 2*(q.y*q.y + q.z*q.z)
                )
                self._pose_ready = True
                break

    # ── Marker ────────────────────────────────────────────────────────────────

    def _publish_ball_marker(self):
        bx, by, bz = self._ball_world
        radius = 0.02
        markers = MarkerArray()

        # Red sphere
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'locobot/base_footprint'
        m.ns = 'arm_throw'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(bx)
        m.pose.position.y = float(by)
        m.pose.position.z = float(bz)
        m.pose.orientation.w = 1.0
        m.scale.x = radius * 2
        m.scale.y = radius * 2
        m.scale.z = radius * 2
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 0.9
        markers.markers.append(m)

        # Label
        lbl = Marker()
        lbl.header.stamp    = m.header.stamp
        lbl.header.frame_id = 'locobot/base_footprint'
        lbl.ns = 'arm_throw'
        lbl.id = 1
        lbl.type = Marker.TEXT_VIEW_FACING
        lbl.action = Marker.ADD
        lbl.pose.position.x = float(bx)
        lbl.pose.position.y = float(by)
        lbl.pose.position.z = float(bz) + radius + 0.05
        lbl.pose.orientation.w = 1.0
        lbl.scale.z = 0.04
        lbl.color.r = 1.0
        lbl.color.g = 1.0
        lbl.color.b = 1.0
        lbl.color.a = 1.0
        lbl.text = f'ball ({bx:.3f}, {by:.3f}, {bz:.3f})'
        markers.markers.append(lbl)

        self._marker_pub.publish(markers)

    # ── Joint Control ─────────────────────────────────────────────────────────

    def _p(self, pub, val):
        msg = Double()
        msg.data = float(val)
        pub.publish(msg)

    def _set_arm(self, w, q1, q2, q3, wr):
        self._p(self._pub_waist, w)
        self._p(self._pub_q1,   q1)
        self._p(self._pub_q2,   q2)
        self._p(self._pub_q3,   q3)
        self._p(self._pub_wr,   wr)
        self._cur_q1 = q1
        self._cur_q2 = q2
        self._cur_q3 = q3
        self._cur_w  = w

    def _set_arm_slow(self, w, q1, q2, q3, wr, steps=SLOW_STEPS, dur=SLOW_DURATION):
        self.get_logger().info(
            f'Slow({dur:.1f}s): q1:{self._cur_q1:.2f}→{q1:.2f} '
            f'q2:{self._cur_q2:.2f}→{q2:.2f} q3:{self._cur_q3:.2f}→{q3:.2f}'
        )
        dt = dur / steps
        for i in range(1, steps+1):
            t = i / steps
            self._p(self._pub_waist, self._cur_w  + t*(w  - self._cur_w))
            self._p(self._pub_q1,   self._cur_q1 + t*(q1 - self._cur_q1))
            self._p(self._pub_q2,   self._cur_q2 + t*(q2 - self._cur_q2))
            self._p(self._pub_q3,   self._cur_q3 + t*(q3 - self._cur_q3))
            self._p(self._pub_wr,   t * wr)
            time.sleep(dt)
        self._cur_q1 = q1
        self._cur_q2 = q2
        self._cur_q3 = q3
        self._cur_w  = w

    def _grip(self, pos):
        self._p(self._pub_lf,  pos)
        self._p(self._pub_rf, -pos)

    def _grip_slow(self, target, current, steps=15, dur=1.5):
        dt = dur / steps
        for i in range(1, steps+1):
            t = i / steps
            self._grip(current + t*(target - current))
            time.sleep(dt)

    # ── Ball Spawning ─────────────────────────────────────────────────────────

    def _spawn_ball(self):
        """Spawn ball at exact gripper position for pickup."""
        t0 = time.time()
        while not self._pose_ready and time.time()-t0 < 5.0:
            time.sleep(0.1)

        yaw = self._robot_yaw

        # Ball world position = robot position + offset rotated by yaw
        bx = self._robot_x + BALL_OFFSET_X * math.cos(yaw)
        by = self._robot_y + BALL_OFFSET_X * math.sin(yaw)
        bz = self._robot_z + BALL_OFFSET_Z

        self._ball_world = (bx, by, bz)

        self.get_logger().info('=' * 60)
        self.get_logger().info('BALL SPAWN:')
        self.get_logger().info(f'  Robot:  ({self._robot_x:.4f}, {self._robot_y:.4f}, {self._robot_z:.4f})')
        self.get_logger().info(f'  Yaw:    {math.degrees(yaw):.2f}°')
        self.get_logger().info(f'  Ball:   ({bx:.4f}, {by:.4f}, {bz:.4f})')
        self.get_logger().info(f'  Pickup: q1={PICK_Q1:.4f} q2={PICK_Q2:.4f} q3={PICK_Q3:.4f}')
        self.get_logger().info('=' * 60)

        cmd = [
            'gz', 'service', '-s', '/world/locobot_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req',
            f'name: "{self._ball_name}", '
            f'position: {{x:{bx:.4f}, y:{by:.4f}, z:{bz:.4f}}}'
        ]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if 'true' in r.stdout.lower():
                self.get_logger().info(f'✅ Ball spawned at ({bx:.4f}, {by:.4f}, {bz:.4f})')
                # Publish marker and keep refreshing
                self._publish_ball_marker()
                self._marker_timer = self.create_timer(2.0, self._publish_ball_marker)
                return True
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')
        return False

    # ── Main Sequence ─────────────────────────────────────────────────────────

    def _run_sequence(self):
        self.get_logger().info('Waiting 2s for Gazebo...')
        time.sleep(2.0)

        # Spawn ball at exact pickup position
        self.get_logger().info('--- SPAWNING BALL ---')
        self._spawn_ball()

        # Wait 5s
        self.get_logger().info('--- WAITING 5s ---')
        for i in range(5, 0, -1):
            self.get_logger().info(f'Pickup in {i}s...')
            time.sleep(1.0)

        # ── PICKUP ────────────────────────────────────────────────────────────

        # Step 1: Open gripper
        self.get_logger().info('PICKUP 1: Opening gripper...')
        self._grip_slow(GRIPPER_OPEN, GRIPPER_CLOSED, steps=10, dur=1.0)

        # Step 2: Move arm slowly to pickup position
        self.get_logger().info(
            f'PICKUP 2: Moving to pickup position '
            f'(q1={PICK_Q1:.4f} q2={PICK_Q2:.4f} q3={PICK_Q3:.4f})...'
        )
        self._set_arm_slow(
            WAIST_FIXED, PICK_Q1, PICK_Q2, PICK_Q3,
            WRIST_ROT_FIXED, steps=SLOW_STEPS, dur=SLOW_DURATION
        )
        time.sleep(0.5)

        # Step 3: Close gripper tight
        self.get_logger().info('PICKUP 3: Closing gripper tight...')
        self._grip_slow(GRIPPER_TIGHT, GRIPPER_OPEN, steps=20, dur=2.0)
        time.sleep(0.5)
        self.get_logger().info('Ball gripped!')

        # Step 4: Lift arm
        self.get_logger().info('PICKUP 4: Lifting arm...')
        self._set_arm_slow(
            WAIST_FIXED, SHOULDER_HOME, ELBOW_HOME,
            WRIST_HOME, WRIST_ROT_FIXED,
            steps=SLOW_STEPS, dur=2.5
        )

        # ── THROW ─────────────────────────────────────────────────────────────

        # Step 5: Wind-back
        self.get_logger().info(
            f'THROW 1: Wind-back | elbow={self._throw_elbow:.3f} rad'
        )
        self._set_arm(
            WAIST_FIXED, SHOULDER_WINDUP,
            self._throw_elbow, WRIST_HOME, WRIST_ROT_FIXED
        )
        time.sleep(self._wind_back_t)

        # Step 6: THROW!
        self.get_logger().info('THROW 2: THROWING! 🚀')
        self._set_arm(
            WAIST_FIXED, SHOULDER_THROW,
            self._throw_elbow, WRIST_HOME, WRIST_ROT_FIXED
        )
        time.sleep(0.15)

        # Step 7: Release
        self.get_logger().info('THROW 3: Releasing ball! 🎯')
        self._grip(GRIPPER_OPEN)
        time.sleep(self._throw_speed_t)

        # Step 8: Return home
        self.get_logger().info('THROW 4: Returning home.')
        self._set_arm(
            WAIST_FIXED, SHOULDER_HOME,
            ELBOW_HOME, WRIST_HOME, WRIST_ROT_FIXED
        )
        self._grip(GRIPPER_OPEN)

        self.get_logger().info('=== SEQUENCE COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = ArmThrowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()