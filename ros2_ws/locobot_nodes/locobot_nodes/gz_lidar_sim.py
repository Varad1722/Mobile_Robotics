#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import rclpy.parameter

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False

# Known wall definitions [x, y, half_width, half_height]
WALLS = [
    (0.0,  5.0, 5.0, 0.1),   # wall_north
    (0.0, -5.0, 5.0, 0.1),   # wall_south
    (5.0,  0.0, 0.1, 5.0),   # wall_east
    (-5.0, 0.0, 0.1, 5.0),   # wall_west
]

class GzLidarSim(Node):
    def __init__(self):
        super().__init__("gz_lidar_sim")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_world_x = None
        self.robot_world_y = None
        self.odom_stamp = None
        self.circle_obstacles = []  # (x, y, radius)

        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("Dynamic LiDAR sim started with Gazebo transport")
        else:
            self.get_logger().warn("Gazebo transport unavailable")

    def pose_cb(self, msg):
        obs = []
        for pose in msg.pose:
            name = pose.name.lower()
            if name == "locobot":
                self.robot_world_x = pose.position.x
                self.robot_world_y = pose.position.y
            if any(skip in name for skip in ["locobot", "ground", "throw", "link", "visual", "sun", "wall"]):
                continue
            if "beacon" in name:
                radius = 0.3
            elif "ball" in name:
                radius = 0.12
            else:
                radius = 0.4
            obs.append((pose.position.x, pose.position.y, radius))
        if obs:
            self.circle_obstacles = obs

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.odom_stamp = msg.header.stamp

    def ray_box_intersect(self, rx, ry, ray_dx, ray_dy, bx, by, hw, hh):
        """Ray-AABB intersection. Returns t or None."""
        t_min = -float("inf")
        t_max = float("inf")

        # X slab
        if abs(ray_dx) > 1e-9:
            tx1 = (bx - hw - rx) / ray_dx
            tx2 = (bx + hw - rx) / ray_dx
            t_min = max(t_min, min(tx1, tx2))
            t_max = min(t_max, max(tx1, tx2))
        else:
            if rx < bx - hw or rx > bx + hw:
                return None

        # Y slab
        if abs(ray_dy) > 1e-9:
            ty1 = (by - hh - ry) / ray_dy
            ty2 = (by + hh - ry) / ray_dy
            t_min = max(t_min, min(ty1, ty2))
            t_max = min(t_max, max(ty1, ty2))
        else:
            if ry < by - hh or ry > by + hh:
                return None

        if t_max >= t_min and t_min > 0.2:
            return t_min
        return None

    def ray_circle_intersect(self, rx, ry, ray_dx, ray_dy, ox, oy, radius):
        """Ray-circle intersection. Returns t or None."""
        dx = ox - rx
        dy = oy - ry
        a = ray_dx**2 + ray_dy**2
        b = 2 * (ray_dx * (-dx) + ray_dy * (-dy))
        c = dx**2 + dy**2 - radius**2
        disc = b**2 - 4*a*c
        if disc >= 0:
            t = (-b - math.sqrt(disc)) / (2*a)
            if t > 0.2:
                return t
        return None

    def publish_scan(self):
        if self.odom_stamp is None:
            return

        rx = self.robot_world_x if self.robot_world_x is not None else self.robot_x
        ry = self.robot_world_y if self.robot_world_y is not None else self.robot_y

        num_rays = 360
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = (angle_max - angle_min) / num_rays
        range_max = 12.0

        ranges = []
        for i in range(num_rays):
            angle = angle_min + i * angle_increment + self.robot_yaw
            ray_dx = math.cos(angle)
            ray_dy = math.sin(angle)
            min_dist = range_max

            # Check walls (box intersection)
            for bx, by, hw, hh in WALLS:
                t = self.ray_box_intersect(rx, ry, ray_dx, ray_dy, bx, by, hw, hh)
                if t is not None and t < min_dist:
                    min_dist = t

            # Check circle obstacles
            for ox, oy, radius in self.circle_obstacles:
                t = self.ray_circle_intersect(rx, ry, ray_dx, ray_dy, ox, oy, radius)
                if t is not None and t < min_dist:
                    min_dist = t

            ranges.append(float(min_dist))

        scan = LaserScan()
        scan.header.stamp = self.odom_stamp
        scan.header.frame_id = "locobot/laser_frame_link"
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.2
        scan.range_max = range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = GzLidarSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
