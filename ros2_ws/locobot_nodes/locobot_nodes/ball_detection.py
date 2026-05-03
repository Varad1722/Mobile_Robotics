#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import rclpy.parameter

class BallDetection(Node):
    def __init__(self):
        super().__init__("ball_detection")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image, "/camera/color", self.image_cb, qos)

        self.detected_pub = self.create_publisher(Bool, "/ball_detected", 10)
        self.pixel_pub = self.create_publisher(Point, "/ball_pixel_pos", 10)
        self.debug_pub = self.create_publisher(Image, "/ball_detection_image", 10)

        self.get_logger().info("Ball Detection node started")

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Orange color HSV range
        lower_orange = np.array([10, 150, 150])
        upper_orange = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = Bool()
        pixel_pos = Point()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 50:  # minimum area filter
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    detected.data = True
                    pixel_pos.x = float(cx)
                    pixel_pos.y = float(cy)
                    pixel_pos.z = float(area)

                    # Draw on debug image
                    cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                    cv2.drawContours(cv_image, [largest], -1, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Ball ({cx},{cy}) area:{area:.0f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    self.get_logger().info(f"Ball detected at pixel ({cx}, {cy}) area={area:.0f}")
            else:
                detected.data = False
        else:
            detected.data = False

        self.detected_pub.publish(detected)
        self.pixel_pub.publish(pixel_pos)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
