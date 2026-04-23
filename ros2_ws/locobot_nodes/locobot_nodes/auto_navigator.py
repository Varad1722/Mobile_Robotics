#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import rclpy.parameter

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False

class AutoNavigator(Node):
    def __init__(self):
        super().__init__("auto_navigator")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.state_pub = self.create_publisher(String, "/robot_state", 10)

        self.ball_sub = self.create_subscription(PoseStamped, "/ball_global_pose", self.ball_cb, 10)
        self.state_sub = self.create_subscription(String, "/robot_state", self.state_cb, 10)

        self.robot_x = -4.0
        self.robot_y = -4.0
        self.ball_x = None
        self.ball_y = None
        self.state = "IDLE"
        self.goal_sent = False

        # Use Gazebo transport for accurate robot position
        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("Using Gazebo transport for robot position")
        else:
            self.get_logger().warn("Gazebo transport not available")

        self.timer = self.create_timer(1.0, self.update)
        self.get_logger().info("Auto Navigator started")

    def pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == "locobot":
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y

    def ball_cb(self, msg):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y
        self.get_logger().info(f"Ball at ({self.ball_x:.2f}, {self.ball_y:.2f})")

    def state_cb(self, msg):
        if msg.data != "NAVIGATING_TO_BALL":
            self.state = msg.data

    def distance_to_ball(self):
        if self.ball_x is None:
            return float("inf")
        return math.sqrt((self.robot_x - self.ball_x)**2 + (self.robot_y - self.ball_y)**2)

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Goal sent: ({x:.2f}, {y:.2f})")

    def update(self):
        state_msg = String()

        if self.state == "IDLE" and self.ball_x is not None and not self.goal_sent:
            self.send_goal(self.ball_x, self.ball_y)
            self.goal_sent = True
            self.state = "NAVIGATING_TO_BALL"
            state_msg.data = "NAVIGATING_TO_BALL"
            self.state_pub.publish(state_msg)
            self.get_logger().info(f"Robot at ({self.robot_x:.2f}, {self.robot_y:.2f}), navigating to ball")

        elif self.state == "NAVIGATING_TO_BALL":
            dist = self.distance_to_ball()
            self.get_logger().info(f"Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}) | Ball: ({self.ball_x:.2f}, {self.ball_y:.2f}) | Dist: {dist:.2f}m")
            if dist < 0.8:
                self.state = "AT_BALL"
                state_msg.data = "AT_BALL"
                self.state_pub.publish(state_msg)

        elif self.state == "NAVIGATE_TO_TARGET":
            self.send_goal(0.0, 0.0)
            self.state = "NAVIGATING_TO_TARGET"
            state_msg.data = "NAVIGATING_TO_TARGET"
            self.state_pub.publish(state_msg)

        elif self.state == "NAVIGATING_TO_TARGET":
            dist = math.sqrt(self.robot_x**2 + self.robot_y**2)
            self.get_logger().info(f"Distance to target: {dist:.2f}m")
            if dist < 0.3:
                self.state = "AT_TARGET"
                state_msg.data = "AT_TARGET"
                self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
