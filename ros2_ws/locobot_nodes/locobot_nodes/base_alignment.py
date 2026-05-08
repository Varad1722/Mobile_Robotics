#!/usr/bin/env python3
# Base Alignment Node - Dhiren Makwana
# RAS598 Mobile Robotics, Arizona State University
#
# After Nav2 gets the robot close to the ball, this node takes over.
# It does the fine alignment - rotate to face ball exactly, then
# drive forward slowly until we are at the right distance to grab it.
# Uses Gazebo ground truth positions so there is no drift error.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import rclpy.parameter

# try gazebo transport for ground truth - odometry drifts too much for this
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False

class BaseAlignment(Node):
    def __init__(self):
        super().__init__("base_alignment")
        # use simulation clock
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        # send velocity commands to move/rotate the robot
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # tell other nodes when alignment is done
        self.state_pub = self.create_publisher(String, "/robot_state", 10)
        # listen for when we should activate (AT_BALL state)
        self.state_sub = self.create_subscription(String, "/robot_state", self.state_cb, 10)

        # track robot and ball positions
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # which direction robot is facing in radians
        self.ball_x = None
        self.ball_y = None
        self.active = False  # only run when AT_BALL state is received

        # subscribe to gazebo world poses
        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("Base Alignment using Gazebo ground truth")

        # run alignment logic at 10 Hz for smooth control
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Base Alignment node started")

    def pose_cb(self, msg):
        """Get robot and ball positions straight from gazebo every frame."""
        for pose in msg.pose:
            if pose.name == "locobot":
                self.robot_x = pose.position.x
                self.robot_y = pose.position.y
                # pull yaw angle out of the quaternion orientation
                # this tells us which direction the robot is facing
                q = pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.robot_yaw = math.atan2(siny, cosy)
            elif pose.name == "ball":
                self.ball_x = pose.position.x
                self.ball_y = pose.position.y

    def state_cb(self, msg):
        """Turn on when navigator says we are at the ball, turn off when done."""
        if msg.data == "AT_BALL":
            self.active = True
            self.get_logger().info(f"Alignment activated! Robot:({self.robot_x:.2f},{self.robot_y:.2f}) Ball:({self.ball_x:.2f},{self.ball_y:.2f})")
        elif msg.data in ["ALIGNED", "GRASPING", "DONE"]:
            # stop moving once alignment is done or grasp has started
            self.active = False
            self.stop()

    def stop(self):
        """Send zero velocity to stop the robot."""
        self.cmd_vel_pub.publish(Twist())

    def update(self):
        """Main alignment loop - rotate to face ball then drive forward to it."""
        # do nothing if not active or no ball position yet
        if not self.active or self.ball_x is None:
            return

        # vector from robot to ball
        dx = self.ball_x - self.robot_x
        dy = self.ball_y - self.robot_y
        dist = math.sqrt(dx**2 + dy**2)

        # what angle do we need to face to look at the ball
        angle_to_ball = math.atan2(dy, dx)

        # how far off are we from facing the ball
        angle_error = angle_to_ball - self.robot_yaw

        # wrap angle to [-pi, pi] so we always turn the short way
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        self.get_logger().info(f"Dist:{dist:.2f}m AngleErr:{math.degrees(angle_error):.1f}deg")

        twist = Twist()
        if abs(angle_error) > 0.1:
            # more than ~6 degrees off - rotate in place first before moving
            twist.angular.z = 0.5 * angle_error  # proportional turn speed
            twist.linear.x = 0.0
        elif dist > 0.535:
            # facing ball but still too far - drive forward slowly
            twist.linear.x = 0.1
            twist.angular.z = 0.3 * angle_error  # small correction while moving
        else:
            # close enough and facing ball - stop and declare aligned
            self.stop()
            self.active = False
            state_msg = String()
            state_msg.data = "ALIGNED"
            self.state_pub.publish(state_msg)
            self.get_logger().info("ALIGNED! Ready to grasp.")
            return

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BaseAlignment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
