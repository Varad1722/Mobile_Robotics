#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import threading
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

# Fixed beacon positions in world frame (x, y)
BEACON_POSITIONS = {
    "beacon_ne": (4.5, 4.5),
    "beacon_se": (4.5, -4.5),
    "beacon_sw": (-4.5, -4.5),
}

NOISE_STDDEV = 0.3  # meters


class BeaconLocalization(Node):
    def __init__(self):
        super().__init__("beacon_localization")

        self.ball_global_pub = self.create_publisher(
            PoseStamped,
            "/ball_global_pose",
            10
        )

        # Gazebo transport node
        self.gz_node = GzNode()
        self.gz_node.subscribe(
            Pose_V,
            "/world/locobot_world/pose/info",
            self.gz_pose_callback
        )

        self.get_logger().info("Beacon Localization Node started")
        self.get_logger().info(f"Noise stddev: {NOISE_STDDEV}m")
        self.get_logger().info(f"Beacons: {list(BEACON_POSITIONS.keys())}")

    def gz_pose_callback(self, msg):
        # Find ball in pose list
        for pose in msg.pose:
            if pose.name == "ball":
                true_x = pose.position.x
                true_y = pose.position.y

                noisy_distances = {}
                for name, (bx, by) in BEACON_POSITIONS.items():
                    true_dist = np.sqrt((true_x - bx)**2 + (true_y - by)**2)
                    noise = np.random.normal(0, NOISE_STDDEV)
                    noisy_dist = max(0.01, true_dist + noise)
                    noisy_distances[name] = noisy_dist

                estimated_pos = self.trilaterate(noisy_distances)

                if estimated_pos is not None:
                    est_x, est_y = estimated_pos

                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "map"
                    pose_msg.pose.position.x = est_x
                    pose_msg.pose.position.y = est_y
                    pose_msg.pose.position.z = 0.1
                    pose_msg.pose.orientation.w = 1.0

                    self.ball_global_pub.publish(pose_msg)

                    self.get_logger().info(
                        f"True: ({true_x:.2f}, {true_y:.2f}) | "
                        f"Estimated: ({est_x:.2f}, {est_y:.2f}) | "
                        f"Error: {np.sqrt((true_x-est_x)**2 + (true_y-est_y)**2):.2f}m"
                    )
                break

    def trilaterate(self, noisy_distances):
        beacons = list(BEACON_POSITIONS.items())
        (name1, (x1, y1)), r1 = beacons[0], noisy_distances[beacons[0][0]]
        (name2, (x2, y2)), r2 = beacons[1], noisy_distances[beacons[1][0]]
        (name3, (x3, y3)), r3 = beacons[2], noisy_distances[beacons[2][0]]

        A = np.array([
            [2*(x2 - x1), 2*(y2 - y1)],
            [2*(x3 - x1), 2*(y3 - y1)]
        ])

        b = np.array([
            r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2,
            r1**2 - r3**2 - x1**2 + x3**2 - y1**2 + y3**2
        ])

        try:
            result = np.linalg.solve(A, b)
            return result[0], result[1]
        except np.linalg.LinAlgError:
            self.get_logger().warn("Trilateration failed - singular matrix")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = BeaconLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
