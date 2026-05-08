#!/usr/bin/env python3
# Arm Grasp Node - Varad Jahagirdar
# RAS598 Mobile Robotics, Arizona State University
#
# This node picks up the ball once the robot is aligned in front of it.
# It tilts the camera down to see the ball up close, gets the 3D position
# from the depth camera, computes joint angles using geometric IK, then
# moves the arm step by step to grab the ball and lift it up.
#
# Falls back to Gazebo ground truth if camera cant see the ball.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import subprocess
import math
import time
import numpy as np
import rclpy.parameter

# try to use gazebo transport for ground truth fallback
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False

# real arm link lengths measured from the URDF file (in meters)
L1 = math.sqrt(0.04975**2 + 0.25**2)   # shoulder to elbow (upper arm)
L2 = 0.175 + 0.075                      # elbow to wrist (forearm)
L3 = 0.065 + 0.043 + 0.023 + 0.027575  # wrist to gripper tip (hand)

# camera intrinsic parameters - same as ball_detection node
FX = 277.19  # focal length x
FY = 277.19  # focal length y
CX = 160.0   # image center x
CY = 120.0   # image center y

# where the camera is relative to the arm base (from TF at tilt=0)
CAM_TO_ARM_X = 0.003  # camera is almost directly above arm base
CAM_TO_ARM_Z = 0.302  # camera is 30cm above arm base
TILT_ANGLE   = 0.75   # how far we tilt camera down to see ball (radians)

# where the arm base is relative to robot center (from TF measurements)
ARM_BASE_X = 0.140    # arm base is 14cm forward of robot center
ARM_BASE_Z = 0.108    # arm base is ~11cm above ground
SHOULDER_Z = 0.354825 # shoulder joint is 35cm above arm base

class ArmGrasp(Node):
    def __init__(self):
        super().__init__("arm_grasp")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        # tell other nodes what we are doing
        self.state_pub  = self.create_publisher(String, "/robot_state", 10)
        # listen for ALIGNED signal to start grasping
        self.state_sub  = self.create_subscription(String, "/robot_state", self.state_cb, 10)
        # lock wheels during grasp so robot doesnt move
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # camera topics use BEST_EFFORT - ok to miss frames
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        # get depth image for distance measurements
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, qos)
        # get 3D ball position computed by ball_detection node
        self.ball_3d_sub = self.create_subscription(
            PointStamped, "/ball_camera_pos", self.ball_3d_cb, 10)

        self.bridge = CvBridge()
        self.depth_image = None

        # ball position from camera (in camera frame)
        self.ball_cam_x  = None
        self.ball_cam_y  = None
        self.ball_cam_z  = None

        # robot and ball positions from gazebo (fallback if camera fails)
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.ball_gt_x = None
        self.ball_gt_y = None
        self.ball_gt_z = 0.025  # ball radius - sits at this height on ground
        self.grasping  = False  # prevent running grasp twice

        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("ArmGrasp: Gazebo ground truth active")
        else:
            self.get_logger().error("Gazebo transport NOT available!")

        self.get_logger().info("Arm Grasp node started - waiting for ALIGNED state")

    def depth_cb(self, msg):
        """Store latest depth image for distance lookups."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception:
            pass

    def ball_3d_cb(self, msg):
        """Store latest 3D ball position from camera detection node."""
        self.ball_cam_x = msg.point.x
        self.ball_cam_y = msg.point.y
        self.ball_cam_z = msg.point.z  # forward distance from camera

    def pose_cb(self, msg):
        """Get robot and ball positions from gazebo for fallback IK calculation."""
        for pose in msg.pose:
            if pose.name == "locobot":
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y
                # extract yaw from quaternion to know which way robot is facing
                q = pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.robot_yaw = math.atan2(siny, cosy)
            elif pose.name == "ball":
                self.ball_gt_x = pose.position.x
                self.ball_gt_y = pose.position.y
                self.ball_gt_z = pose.position.z

    def state_cb(self, msg):
        """Start grasping when base alignment says we are lined up with the ball."""
        if msg.data == "ALIGNED" and not self.grasping:
            self.grasping = True  # set flag so we dont start twice
            self.execute_grasp()

    def send_joint(self, joint, angle):
        """Send a joint position command directly to Gazebo via gz topic."""
        topic = f"/model/locobot/joint/{joint}/0/cmd_pos"
        cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {angle}"]
        subprocess.run(cmd, capture_output=True)
        self.get_logger().info(f"  {joint} -> {math.degrees(angle):.1f}deg")

    def tilt_camera_down(self):
        """Tilt the camera down so it can see the ball on the ground when close."""
        self.get_logger().info("Tilting camera down...")
        self.send_joint("tilt", TILT_ANGLE)
        time.sleep(2.0)  # wait for camera to physically move

    def tilt_camera_home(self):
        """Tilt camera back to forward-facing position after grasping."""
        self.send_joint("tilt", 0.0)
        time.sleep(1.0)

    def get_ball_in_arm_frame(self):
        """
        Figure out where the ball is relative to the arm base.
        First tries the depth camera position (more accurate when available).
        Falls back to Gazebo ground truth if camera doesnt see ball.
        Returns (x, y, z) in arm_base_link frame.
        """

        # use camera measurement if ball is at a reasonable distance
        if self.ball_cam_z is not None and 0.1 < self.ball_cam_z < 2.0:
            # rotate camera frame to arm frame accounting for the tilt angle
            # camera tilted down means its z axis points forward+down
            ct = math.cos(TILT_ANGLE)
            st = math.sin(TILT_ANGLE)

            # rotate camera frame by tilt angle around Y axis to get arm frame coords
            x_arm = CAM_TO_ARM_X + self.ball_cam_z * ct - self.ball_cam_y * st
            y_arm = self.ball_cam_x  # left-right doesnt change with tilt
            z_arm = CAM_TO_ARM_Z - self.ball_cam_z * st - self.ball_cam_y * ct

            self.get_logger().info(f"Ball from CAMERA: arm_frame x={x_arm:.3f} y={y_arm:.3f} z={z_arm:.3f}")
            return x_arm, y_arm, z_arm

        # camera failed - use gazebo ground truth instead
        self.get_logger().warn("Camera ball pos not available, using Gazebo GT")
        dx_world = self.ball_gt_x - self.robot_x
        dy_world = self.ball_gt_y - self.robot_y
        # rotate from world frame to robot local frame using robot yaw angle
        cos_yaw = math.cos(-self.robot_yaw)
        sin_yaw = math.sin(-self.robot_yaw)
        x_local = dx_world * cos_yaw - dy_world * sin_yaw - ARM_BASE_X
        y_local = dx_world * sin_yaw + dy_world * cos_yaw
        # z is how far below the shoulder the ball is
        z_local = self.ball_gt_z - (ARM_BASE_Z + SHOULDER_Z)
        self.get_logger().info(f"Ball from GT: arm_frame x={x_local:.3f} y={y_local:.3f} z={z_local:.3f}")
        return x_local, y_local, z_local

    def compute_geometric_ik(self, x_arm, y_arm, z_arm):
        """
        Compute shoulder, elbow and wrist angles to reach a target point.
        Uses geometric 2-link IK (law of cosines) - no library needed.
        The arm is treated as two links (upper arm and forearm) reaching to wrist,
        then wrist points down to align gripper with ball.
        Returns (waist, shoulder, elbow, wrist) angles in radians.
        """
        horiz = x_arm         # horizontal distance forward to target
        vert  = -z_arm        # vertical drop (z_arm is negative below shoulder)

        # waist rotates horizontally to face the ball left/right
        waist = math.atan2(y_arm, x_arm)
        waist = max(-math.pi/2, min(math.pi/2, waist))  # clamp to safe range

        # wrist needs to be at ball position minus the gripper length
        wx = horiz - L3  # how far the wrist needs to reach horizontally
        wz = vert         # how far the wrist needs to reach vertically

        # total distance from shoulder to wrist target
        reach = math.sqrt(wx**2 + wz**2)
        max_reach = L1 + L2 - 0.01  # cant fully extend - leave small margin

        # if target is too far, scale down to max reach
        if reach > max_reach:
            self.get_logger().warn(f"Clamping reach {reach:.3f} to {max_reach:.3f}")
            scale = max_reach / reach
            wx *= scale
            wz *= scale
            reach = max_reach

        # law of cosines to find elbow angle
        cos_elbow = (reach**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))  # clamp to valid acos range
        elbow = math.acos(cos_elbow)

        # shoulder angle - angle to target minus correction for elbow bend
        alpha = math.atan2(wz, wx)  # angle from shoulder to wrist target
        cos_beta = (reach**2 + L1**2 - L2**2) / (2 * reach * L1)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)  # correction angle due to elbow bend
        shoulder = alpha - beta

        # wrist compensates so gripper stays roughly horizontal
        wrist = -(shoulder + elbow)

        self.get_logger().info(
            f"IK: waist={math.degrees(waist):.1f} shoulder={math.degrees(shoulder):.1f} "
            f"elbow={math.degrees(elbow):.1f} wrist={math.degrees(wrist):.1f}")

        return waist, shoulder, elbow, wrist

    def execute_grasp(self):
        """Run the full grasp sequence - tilt camera, compute IK, move arm, grab ball, lift."""
        self.get_logger().info("=== GRASP SEQUENCE START ===")
        self.publish_state("GRASPING")

        # keep sending zero velocity so robot doesnt drift during grasp
        lock_timer = self.create_timer(0.1, lambda: self.cmd_vel_pub.publish(Twist()))
        time.sleep(0.5)

        # step 1 - tilt camera down to see the ball at close range
        self.get_logger().info("Step 1: Tilt camera down")
        self.tilt_camera_down()

        # step 2 - wait a moment for camera to stabilize and ball_detection to update
        self.get_logger().info("Step 2: Waiting for ball detection...")
        time.sleep(2.0)

        # step 3 - get ball position in arm frame (from camera or GT)
        x_arm, y_arm, z_arm = self.get_ball_in_arm_frame()

        # step 4 - compute IK angles for grasp position
        waist, shoulder, elbow, wrist = self.compute_geometric_ik(x_arm, y_arm, z_arm)

        # step 5 - open gripper wide before approaching
        self.get_logger().info("Step 3: Open gripper")
        self.send_joint("left_finger", 0.037)  # max open position
        time.sleep(1.5)

        # step 6 - rotate waist to face ball laterally
        self.get_logger().info("Step 4: Waist")
        self.send_joint("waist", waist)
        time.sleep(1.5)

        # step 7 - move to pre-grasp position 5cm above ball
        # doing elbow then wrist then shoulder keeps weight balanced to avoid tipping
        self.get_logger().info("Step 5: Pre-grasp position")
        pre_z = z_arm + 0.05  # 5cm above actual ball position
        _, pre_shoulder, pre_elbow, pre_wrist = self.compute_geometric_ik(x_arm, y_arm, pre_z)
        self.send_joint("elbow", pre_elbow)
        time.sleep(1.0)
        self.send_joint("wrist_angle", pre_wrist)
        time.sleep(1.0)
        self.send_joint("shoulder", pre_shoulder)
        time.sleep(2.0)

        # step 8 - lower down to ball position
        self.get_logger().info("Step 6: Lower to ball")
        self.send_joint("elbow", elbow)
        time.sleep(1.0)
        # add tiny extra wrist angle to ensure gripper makes contact
        self.send_joint("wrist_angle", wrist + 0.0175)
        time.sleep(1.0)
        self.send_joint("shoulder", shoulder)
        time.sleep(2.0)

        # step 9 - close gripper around the ball
        self.get_logger().info("Step 7: Close gripper")
        self.send_joint("left_finger", 0.015)  # minimum closed position
        time.sleep(1.5)

        # step 10 - lift arm back up to pre-grasp height with ball
        self.get_logger().info("Step 8: Lift up")
        self.send_joint("shoulder", pre_shoulder)
        time.sleep(1.0)
        self.send_joint("elbow", pre_elbow)
        time.sleep(1.0)
        self.send_joint("wrist_angle", pre_wrist)
        time.sleep(1.5)

        # step 11 - fold arm back to home position for safe navigation
        self.get_logger().info("Step 9: Home position")
        self.send_joint("shoulder", 0.0)
        time.sleep(1.0)
        self.send_joint("elbow", 1.5)
        time.sleep(1.0)
        self.send_joint("wrist_angle", -1.2)
        time.sleep(1.0)
        self.tilt_camera_home()  # tilt camera back to forward position

        lock_timer.cancel()  # stop sending zero velocity
        self.get_logger().info("=== GRASP COMPLETE ===")
        self.publish_state("GRASPED")

    def publish_state(self, state):
        """Publish current state so other nodes know what is happening."""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"State -> {state}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmGrasp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
