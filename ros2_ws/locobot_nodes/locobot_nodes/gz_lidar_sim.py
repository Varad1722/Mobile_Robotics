#!/usr/bin/env python3
"""
Dynamic LiDAR simulator using Gazebo pose data via ROS2 bridge.
Subscribes to /model/locobot/tf and /odom to get robot position,
reads obstacle poses from gz transport, publishes LaserScan.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math
import rclpy.parameter

# Model collision radii by name pattern
MODEL_SIZES = {
    'wall_north': (0, 5, 5.0, 0.1),
    'wall_south': (0, -5, 5.0, 0.1),
    'wall_east':  (5, 0, 0.1, 5.0),
    'wall_west':  (-5, 0, 0.1, 5.0),
}

class GzLidarSim(Node):
    def __init__(self):
        super().__init__('gz_lidar_sim')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_stamp = None
        self.obstacles = []

        # Use gz transport for dynamic obstacle detection
        try:
            from gz.transport13 import Node as GzNode
            from gz.msgs10.pose_v_pb2 import Pose_V
            self.gz_node = GzNode()
            self.gz_node.subscribe(
                Pose_V,
                '/world/locobot_world/pose/info',
                self.pose_cb
            )
            self.get_logger().info('Dynamic LiDAR sim started with Gazebo transport')
        except Exception as e:
            self.get_logger().warn(f'Gazebo transport unavailable: {e}')
            self.gz_node = None

    def pose_cb(self, msg):
        obs = []
        for pose in msg.pose:
            name = pose.name.lower()
            if 'locobot' in name or 'ground' in name or 'throw' in name:
                continue
            # Determine radius based on model type
            if 'wall' in name:
                radius = 0.15
            elif 'beacon' in name:
                radius = 0.15
            elif 'ball' in name:
                radius = 0.12
            else:
                radius = 0.35
            obs.append((pose.position.x, pose.position.y, radius))
        if obs:
            self.obstacles = obs

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.odom_stamp = msg.header.stamp

    def publish_scan(self):
        if not self.obstacles or self.odom_stamp is None:
            return

        num_rays = 360
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = (angle_max - angle_min) / num_rays
        range_max = 12.0
        range_min = 0.2

        ranges = []
        for i in range(num_rays):
            angle = angle_min + i * angle_increment + self.robot_yaw
            min_dist = range_max
            ray_dx = math.cos(angle)
            ray_dy = math.sin(angle)

            for ox, oy, radius in self.obstacles:
                dx = ox - self.robot_x
                dy = oy - self.robot_y
                a = ray_dx**2 + ray_dy**2
                b = 2 * (ray_dx * (-dx) + ray_dy * (-dy))
                c = dx**2 + dy**2 - radius**2
                disc = b**2 - 4*a*c
                if disc >= 0:
                    t = (-b - math.sqrt(disc)) / (2*a)
                    if range_min < t < min_dist:
                        min_dist = t

            ranges.append(float(min_dist))

        scan = LaserScan()
        scan.header.stamp = self.odom_stamp
        scan.header.frame_id = 'locobot/laser_frame_link'
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = GzLidarSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
