#!/usr/bin/env python3
# Beacon Localization Node - Dhiren Makwana
# RAS598 Mobile Robotics, Arizona State University
#
# The idea: we have 3 fixed beacons in the arena. By measuring how far the ball
# is from each beacon, we can figure out roughly where the ball is.
# Its like GPS but with only 3 satellites and some measurement noise.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import threading
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

# Where the 3 beacons are physically placed in the arena (x, y) in meters
# These are fixed - they dont move
BEACON_POSITIONS = {
    "beacon_ne": (4.5, 4.5),    # top right corner
    "beacon_se": (4.5, -4.5),   # bottom right corner
    "beacon_sw": (-4.5, -4.5),  # bottom left corner
}

# How much noise to add to fake real-world sensor imperfection (in meters)
NOISE_STDDEV = 0.3

class BeaconLocalization(Node):
    def __init__(self):
        super().__init__("beacon_localization")

        # this is where we send our best guess of where the ball is
        self.ball_global_pub = self.create_publisher(
            PoseStamped,
            "/ball_global_pose",
            10
        )

        # connect to gazebo directly to get positions - faster than going through ROS
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
        # this gets called every time gazebo sends us updated positions of everything
        for pose in msg.pose:
            if pose.name == "ball":
                # grab the actual ball position from gazebo (this is ground truth)
                true_x = pose.position.x
                true_y = pose.position.y

                # now pretend we dont know the exact position
                # measure distance from ball to each beacon and add some noise
                noisy_distances = {}
                for name, (bx, by) in BEACON_POSITIONS.items():
                    # real distance from ball to this beacon
                    true_dist = np.sqrt((true_x - bx)**2 + (true_y - by)**2)
                    # add random noise like a real sensor would have
                    noise = np.random.normal(0, NOISE_STDDEV)
                    # make sure distance stays positive even with noise
                    noisy_dist = max(0.01, true_dist + noise)
                    noisy_distances[name] = noisy_dist

                # now use those noisy distances to estimate where the ball is
                estimated_pos = self.trilaterate(noisy_distances)

                if estimated_pos is not None:
                    est_x, est_y = estimated_pos

                    # package up the estimate and publish it so the navigator can use it
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "map"
                    pose_msg.pose.position.x = est_x
                    pose_msg.pose.position.y = est_y
                    pose_msg.pose.position.z = 0.1
                    pose_msg.pose.orientation.w = 1.0
                    self.ball_global_pub.publish(pose_msg)

                    # log how far off we were from the real position
                    self.get_logger().info(
                        f"True: ({true_x:.2f}, {true_y:.2f}) | "
                        f"Estimated: ({est_x:.2f}, {est_y:.2f}) | "
                        f"Error: {np.sqrt((true_x-est_x)**2 + (true_y-est_y)**2):.2f}m"
                    )
                break  # found the ball, no need to keep looping

    def trilaterate(self, noisy_distances):
        """
        Given distances from 3 beacons, figure out where the ball is.
        The math trick: 3 circles (one around each beacon) should all intersect
        at the ball location. We subtract the equations to get a simple linear
        system instead of solving circles directly.
        """
        beacons = list(BEACON_POSITIONS.items())

        # pull out each beacons position and the measured distance to it
        (name1, (x1, y1)), r1 = beacons[0], noisy_distances[beacons[0][0]]
        (name2, (x2, y2)), r2 = beacons[1], noisy_distances[beacons[1][0]]
        (name3, (x3, y3)), r3 = beacons[2], noisy_distances[beacons[2][0]]

        # subtracting circle equation 1 from 2 and 3 cancels x^2 and y^2 terms
        # leaving us with a 2x2 linear system we can solve easily
        A = np.array([
            [2*(x2 - x1), 2*(y2 - y1)],
            [2*(x3 - x1), 2*(y3 - y1)]
        ])
        b = np.array([
            r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2,
            r1**2 - r3**2 - x1**2 + x3**2 - y1**2 + y3**2
        ])

        try:
            # solve Ax = b for the (x, y) position of the ball
            result = np.linalg.solve(A, b)
            return result[0], result[1]
        except np.linalg.LinAlgError:
            # this fails if beacons are in a straight line (rare but possible)
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
