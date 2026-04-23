#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy.parameter

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    GZ_AVAILABLE = True
except Exception:
    GZ_AVAILABLE = False

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.tf_broadcaster = TransformBroadcaster(self)
        self.stamp = None
        self.x = -4.0
        self.y = -4.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # Subscribe to clock for timestamps
        from rosgraph_msgs.msg import Clock
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_cb, 10)

        if GZ_AVAILABLE:
            self.gz_node = GzNode()
            self.gz_node.subscribe(Pose_V, "/world/locobot_world/pose/info", self.pose_cb)
            self.get_logger().info("OdomTF: Using Gazebo ground truth pose")
        else:
            from nav_msgs.msg import Odometry
            self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
            self.get_logger().warn("OdomTF: Falling back to odometry")

        self.timer = self.create_timer(0.05, self.publish_tf)

    def clock_cb(self, msg):
        self.stamp = msg.clock

    def pose_cb(self, msg):
        for pose in msg.pose:
            if pose.name == "locobot":
                self.x = pose.position.x
                self.y = pose.position.y
                self.qx = pose.orientation.x
                self.qy = pose.orientation.y
                self.qz = pose.orientation.z
                self.qw = pose.orientation.w

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.qx = msg.pose.pose.orientation.x
        self.qy = msg.pose.pose.orientation.y
        self.qz = msg.pose.pose.orientation.z
        self.qw = msg.pose.pose.orientation.w
        self.stamp = msg.header.stamp

    def publish_tf(self):
        if self.stamp is None:
            return
        t = TransformStamped()
        t.header.stamp = self.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "locobot/base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
