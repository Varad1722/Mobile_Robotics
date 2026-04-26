#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.parameter
import subprocess

class ArmHome(Node):
    def __init__(self):
        super().__init__("arm_home")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.sent = False
        self.timer = self.create_timer(2.0, self.send_home)
        self.get_logger().info("Arm Home node started")

    def send_home(self):
        if self.sent:
            return

        joints = {
            "shoulder": 0.0,
            "elbow": 1.5,
            "waist": 0.0,
            "forearm_roll": 0.0,
            "wrist_angle": -1.2,
            "wrist_rotate": 0.0,
        }

        for joint, angle in joints.items():
            topic = f"/model/locobot/joint/{joint}/0/cmd_pos"
            cmd = ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {angle}"]
            subprocess.run(cmd, capture_output=True)
            self.get_logger().info(f"Sent {joint} = {angle}")

        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = ArmHome()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
