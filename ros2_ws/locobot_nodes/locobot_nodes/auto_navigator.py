#!/usr/bin/env python3
# Auto Navigator Node - Dhiren Makwana
# RAS598 Mobile Robotics, Arizona State University
#
# This node is the brain of the navigation pipeline.
# It tells the robot where to go using Nav2 and decides when to stop.
# It works like a traffic controller - changes states based on what is happening.
#
# Flow: IDLE -> NAVIGATING_TO_BALL -> AT_BALL -> (base_alignment takes over)
#       then later: NAVIGATE_TO_TARGET -> NAVIGATING_TO_TARGET -> AT_TARGET

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import rclpy.parameter

# try to use gazebo ground truth - much more reliable than odometry which drifts
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False

class AutoNavigator(Node):
    def __init__(self):
        super().__init__("auto_navigator")
        # use gazebo clock so timing matches simulation speed
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        # send navigation goals to Nav2
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        # tell other nodes what state we are in
        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        # stop the robot by sending zero velocity
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # listen for ball position estimate from beacon localization
        self.ball_sub = self.create_subscription(PoseStamped, "/ball_global_pose", self.ball_cb, 10)
        # listen for state changes from other nodes (like ALIGNED from base_alignment)
        self.state_sub = self.create_subscription(String, "/robot_state", self.state_cb, 10)

        # camera topics use BEST_EFFORT because we dont care if we miss a frame
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        # know if ball is visible in camera right now
        self.ball_detected_sub = self.create_subscription(Bool, "/ball_detected", self.ball_detected_cb, qos)

        # start somewhere far - will get updated from gazebo immediately
        self.robot_x = -4.0
        self.robot_y = -4.0
        self.ball_x = None
        self.ball_y = None
        self.state = "IDLE"
        self.goal_sent = False   # make sure we only send one goal at a time
        self.ball_visible = False

        # hook into gazebo to get exact robot position
        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("Using Gazebo transport for robot position")

        # check and update state every 0.5 seconds
        self.timer = self.create_timer(0.5, self.update)
        self.get_logger().info("Auto Navigator started")

    def pose_cb(self, msg):
        """Get robot position directly from gazebo - way more accurate than odometry."""
        for pose in msg.pose:
            if pose.name == "locobot":
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y

    def ball_cb(self, msg):
        """Store the latest ball position estimate from beacon localization."""
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y

    def ball_detected_cb(self, msg):
        """Track whether the camera can currently see the ball."""
        self.ball_visible = msg.data

    def state_cb(self, msg):
        """React to state changes from other nodes - but ignore NAVIGATING_TO_BALL
        so we dont overwrite our own state while we are driving."""
        if msg.data != "NAVIGATING_TO_BALL":
            self.state = msg.data

    def distance_to_ball(self):
        """How far is the robot from the ball right now."""
        if self.ball_x is None:
            return float("inf")  # no ball info yet, pretend its infinitely far
        return math.sqrt((self.robot_x - self.ball_x)**2 + (self.robot_y - self.ball_y)**2)

    def stop_robot(self):
        """Send zero velocity to stop the robot immediately."""
        twist = Twist()  # all zeros by default
        self.cmd_vel_pub.publish(twist)

    def send_goal(self, x, y, target_x=None, target_y=None):
        """Tell Nav2 to drive to position (x, y), optionally facing toward a target point."""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y

        # if we want to face toward something (like the ball), compute the angle
        if target_x is not None and target_y is not None:
            angle = math.atan2(target_y - y, target_x - x)
            # convert angle to quaternion - only need z and w for 2D rotation
            goal.pose.orientation.z = math.sin(angle / 2.0)
            goal.pose.orientation.w = math.cos(angle / 2.0)
        else:
            goal.pose.orientation.w = 1.0  # no rotation, face forward

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Goal sent: ({x:.2f}, {y:.2f})")

    def update(self):
        """Main loop - runs every 0.5s and decides what to do based on current state."""
        state_msg = String()

        if self.state == "IDLE" and self.ball_x is not None and not self.goal_sent:
            # we know where the ball is and havent started moving yet
            # send nav2 to a point 1m in front of the ball so we dont crash into it
            dx = self.ball_x - self.robot_x
            dy = self.ball_y - self.robot_y
            dist = math.sqrt(dx**2 + dy**2)
            if dist > 1.0:
                # back off 1m from the ball along the approach direction
                goal_x = self.ball_x - 1.0 * (dx / dist)
                goal_y = self.ball_y - 1.0 * (dy / dist)
            else:
                # already close enough, just stay here
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

            # stop if ball is visible and we are close enough, or if we are very close regardless
            if (self.ball_visible and dist < 1.5) or dist < 0.8:
                self.stop_robot()
                self.state = "AT_BALL"
                state_msg.data = "AT_BALL"
                self.state_pub.publish(state_msg)
                self.get_logger().info(f"Ball found! Stopping. Dist:{dist:.2f}m Visible:{self.ball_visible}")

        elif self.state == "NAVIGATE_TO_TARGET":
            # after grasping, drive back to the throw target at the center of the arena
            self.send_goal(0.0, 0.0)
            self.state = "NAVIGATING_TO_TARGET"
            state_msg.data = "NAVIGATING_TO_TARGET"
            self.state_pub.publish(state_msg)

        elif self.state == "NAVIGATING_TO_TARGET":
            # check if we made it to the center
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
