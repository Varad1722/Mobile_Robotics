#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.ball_sub = self.create_subscription(PoseStamped, "/ball_global_pose", self.ball_cb, 10)
        self.state_sub = self.create_subscription(String, "/robot_state", self.state_cb, 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.ball_detected_sub = self.create_subscription(Bool, "/ball_detected", self.ball_detected_cb, qos)

        self.robot_x = -4.0
        self.robot_y = -4.0
        self.ball_x = None
        self.ball_y = None
        self.state = "IDLE"
        self.goal_sent = False
        self.ball_visible = False

        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("Using Gazebo transport for robot position")

        self.timer = self.create_timer(0.5, self.update)
        self.get_logger().info("Auto Navigator started")

    def pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == "locobot":
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y

    def ball_cb(self, msg):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y

    def ball_detected_cb(self, msg):
        self.ball_visible = msg.data

    def state_cb(self, msg):
        if msg.data != "NAVIGATING_TO_BALL":
            self.state = msg.data

    def distance_to_ball(self):
        if self.ball_x is None:
            return float("inf")
        return math.sqrt((self.robot_x - self.ball_x)**2 + (self.robot_y - self.ball_y)**2)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def send_goal(self, x, y, target_x=None, target_y=None):
        import math
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y

        # Face toward target if provided
        if target_x is not None and target_y is not None:
            angle = math.atan2(target_y - y, target_x - x)
            goal.pose.orientation.z = math.sin(angle / 2.0)
            goal.pose.orientation.w = math.cos(angle / 2.0)
        else:
            goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Goal sent: ({x:.2f}, {y:.2f})")

    def update(self):
        state_msg = String()

        if self.state == "IDLE" and self.ball_x is not None and not self.goal_sent:
            import math
            # Stop 1m before ball, facing toward it
            dx = self.ball_x - self.robot_x
            dy = self.ball_y - self.robot_y
            dist = math.sqrt(dx**2 + dy**2)
            if dist > 1.0:
                goal_x = self.ball_x - 1.0 * (dx / dist)
                goal_y = self.ball_y - 1.0 * (dy / dist)
            else:
                goal_x = self.robot_x
                goal_y = self.robot_y
            self.send_goal(goal_x, goal_y, self.ball_x, self.ball_y)
            self.goal_sent = True
            self.state = "NAVIGATING_TO_BALL"
            state_msg.data = "NAVIGATING_TO_BALL"
            self.state_pub.publish(state_msg)
            self.get_logger().info(f"Navigating to ball at ({self.ball_x:.2f}, {self.ball_y:.2f})")

        elif self.state == "NAVIGATING_TO_BALL":
            dist = self.distance_to_ball()
            self.get_logger().info(f"Dist: {dist:.2f}m | Ball visible: {self.ball_visible}")

            # Stop when close enough (visible AND within 1.5m, OR within 0.8m)
            if (self.ball_visible and dist < 1.5) or dist < 0.8:
                self.stop_robot()  # Stop immediately
                self.state = "AT_BALL"
                state_msg.data = "AT_BALL"
                self.state_pub.publish(state_msg)
                self.get_logger().info(f"Ball found! Stopping. Dist:{dist:.2f}m Visible:{self.ball_visible}")

        elif self.state == "NAVIGATE_TO_TARGET":
            self.send_goal(0.0, 0.0)
            self.state = "NAVIGATING_TO_TARGET"
            state_msg.data = "NAVIGATING_TO_TARGET"
            self.state_pub.publish(state_msg)

        elif self.state == "NAVIGATING_TO_TARGET":
            dist = math.sqrt(self.robot_x**2 + self.robot_y**2)
            if dist < 0.3:
                self.stop_robot()
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
