#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import math
import time
import rclpy.parameter

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False


class ArmGrasp(Node):
    def __init__(self):
        super().__init__("arm_grasp")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        self.state_sub = self.create_subscription(String, "/robot_state", self.state_cb, 10)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.ball_x = None
        self.ball_y = None
        self.ball_z = 0.03

        self.active = False
        self.grasping = False

        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("ArmGrasp: using Gazebo ground truth")
        else:
            self.get_logger().error("Gazebo transport NOT available!")

        self.get_logger().info("Arm Grasp node started — waiting for ALIGNED state")

    def pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == "locobot":
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y
                q = pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.robot_yaw = math.atan2(siny, cosy)
            elif pose.name == "ball":
                self.ball_x = pose.position.x
                self.ball_y = pose.position.y
                self.ball_z = pose.position.z

    def state_cb(self, msg):
        if msg.data == "ALIGNED" and not self.grasping:
            self.active = True
            self.grasping = True
            self.get_logger().info("ALIGNED received — starting grasp sequence!")
            self.execute_grasp()

    def send_joint(self, joint, angle, delay=1.0):
        topic = f"/model/locobot/joint/{joint}/0/cmd_pos"
        cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {angle}"]
        subprocess.run(cmd, capture_output=True)
        self.get_logger().info(f"  Joint {joint} → {math.degrees(angle):.1f}°")
        time.sleep(delay)

    def compute_waist_angle(self):
        if self.ball_x is None:
            return 0.0
        dx = self.ball_x - self.robot_x
        dy = self.ball_y - self.robot_y
        angle_to_ball = math.atan2(dy, dx)
        waist = angle_to_ball - self.robot_yaw
        while waist > math.pi:
            waist -= 2 * math.pi
        while waist < -math.pi:
            waist += 2 * math.pi
        waist = max(-math.pi / 2, min(math.pi / 2, waist))
        return waist

    def compute_reach_angles(self):
        if self.ball_x is None:
            return 0.8, 1.0

        dx = self.ball_x - self.robot_x
        dy = self.ball_y - self.robot_y
        horizontal_dist = math.sqrt(dx**2 + dy**2)

        arm_base_z = 0.36
        target_z = self.ball_z + 0.02
        dz = arm_base_z - target_z

        reach = max(0.15, min(horizontal_dist - 0.05, 0.35))

        L1 = 0.20
        L2 = 0.20
        dist = math.sqrt(reach**2 + dz**2)
        dist = min(dist, L1 + L2 - 0.01)

        cos_elbow = (dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow = math.acos(cos_elbow)

        alpha = math.atan2(dz, reach)
        beta = math.acos(max(-1.0, min(1.0, (dist**2 + L1**2 - L2**2) / (2 * dist * L1))))
        shoulder = -(alpha + beta)

        self.get_logger().info(
            f"IK: reach={reach:.2f}m dz={dz:.2f}m → shoulder={math.degrees(shoulder):.1f}° elbow={math.degrees(elbow):.1f}°"
        )
        return shoulder, elbow

    def execute_grasp(self):
        self.get_logger().info("=== GRASP SEQUENCE START ===")
        self.publish_state("GRASPING")

        waist = self.compute_waist_angle()
        shoulder, elbow = self.compute_reach_angles()

        self.get_logger().info(f"Ball at ({self.ball_x:.2f}, {self.ball_y:.2f}, {self.ball_z:.3f})")
        self.get_logger().info(f"Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}) yaw={math.degrees(self.robot_yaw):.1f}°")
        self.get_logger().info(f"Computed: waist={math.degrees(waist):.1f}° shoulder={math.degrees(shoulder):.1f}° elbow={math.degrees(elbow):.1f}°")

        self.get_logger().info("Step 1: Rotating waist...")
        self.send_joint("waist", waist, delay=1.5)

        self.get_logger().info("Step 2: Extending arm...")
        self.send_joint("shoulder", shoulder, delay=1.0)
        self.send_joint("elbow", elbow, delay=1.0)
        self.send_joint("wrist_angle", -0.5, delay=1.0)
        self.send_joint("forearm_roll", 0.0, delay=0.5)
        self.send_joint("wrist_rotate", 0.0, delay=0.5)

        self.get_logger().info("Step 3: Lowering to ball...")
        self.send_joint("shoulder", shoulder - 0.15, delay=1.0)
        self.send_joint("wrist_angle", -0.3, delay=1.0)

        self.get_logger().info("Step 4: Grasping...")
        self.send_joint("wrist_angle", 0.2, delay=1.0)
        self.send_joint("shoulder", shoulder + 0.1, delay=1.0)

        self.get_logger().info("Step 5: Lifting ball...")
        self.send_joint("shoulder", -0.5, delay=1.2)
        self.send_joint("elbow", 1.0, delay=1.2)
        self.send_joint("wrist_angle", -0.8, delay=1.0)

        self.get_logger().info("=== GRASP COMPLETE ===")
        self.publish_state("GRASPED")

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"State → {state}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmGrasp()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
