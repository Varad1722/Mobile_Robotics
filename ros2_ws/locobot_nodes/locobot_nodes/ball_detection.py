#!/usr/bin/env python3
# Ball Detection Node - Dhiren Makwana
# RAS598 Mobile Robotics, Arizona State University
#
# Watches the robot camera feed and finds the magenta ball in the image.
# Also uses the depth camera to figure out how far away the ball is in 3D.
# Publishes the ball pixel position, whether its detected, and its 3D position
# in camera frame so the arm grasp node can use it for picking up the ball.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
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

        # opencv bridge to convert ROS image messages to numpy arrays
        self.bridge = CvBridge()

        # camera intrinsic parameters - needed to convert pixel + depth to 3D point
        # these come from /camera/camera_info topic (fx=fy=277.19, cx=160, cy=120)
        self.fx = 277.19  # focal length in x (pixels)
        self.fy = 277.19  # focal length in y (pixels)
        self.cx = 160.0   # principal point x - center of image width
        self.cy = 120.0   # principal point y - center of image height

        # store the latest depth image so we can look up depth at ball pixel
        self.depth_image = None

        # camera topics use BEST_EFFORT - ok to drop frames, we just need latest
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscribe to color camera for ball detection
        self.image_sub = self.create_subscription(Image, "/camera/color", self.image_cb, qos)
        # subscribe to depth camera to get distance to ball
        self.depth_sub = self.create_subscription(Image, "/camera/depth", self.depth_cb, qos)

        # publish True/False whether ball is currently visible
        self.detected_pub  = self.create_publisher(Bool, "/ball_detected", 10)
        # publish ball pixel location (u, v) and area in image
        self.pixel_pub     = self.create_publisher(Point, "/ball_pixel_pos", 10)
        # publish annotated camera image for debugging in rqt
        self.debug_pub     = self.create_publisher(Image, "/ball_detection_image", 10)
        # publish ball 3D position in camera frame (x, y, z in meters)
        self.ball_3d_pub   = self.create_publisher(PointStamped, "/ball_camera_pos", 10)

        self.get_logger().info("Ball Detection node started")

    def depth_cb(self, msg):
        """Store the latest depth image so we can look up depth values later."""
        try:
            # convert to 32-bit float array where each value is distance in meters
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth bridge error: {e}")

    def image_cb(self, msg):
        """Process each color camera frame to find the magenta ball."""
        try:
            # convert ROS image to BGR numpy array opencv can work with
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        # convert to HSV color space - much easier to filter colors than BGR
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # magenta wraps around the HSV hue circle so we need two ranges
        # range 1: high hue values (pink/magenta end)
        lower_mag1 = np.array([140, 100, 100])
        upper_mag1 = np.array([180, 255, 255])
        # range 2: low hue values (red/magenta start)
        lower_mag2 = np.array([0, 100, 100])
        upper_mag2 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_mag1, upper_mag1)
        mask2 = cv2.inRange(hsv, lower_mag2, upper_mag2)
        # combine both ranges into one mask
        mask = cv2.bitwise_or(mask1, mask2)

        # clean up the mask - remove small noise blobs and fill gaps
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel) # fill small holes

        # find the blobs of magenta color in the image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected  = Bool()
        pixel_pos = Point()

        if contours:
            # take the biggest blob - most likely the ball
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 50:  # ignore tiny specks that are probably not the ball
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    # compute centroid of the blob (center of mass)
                    u = int(M["m10"] / M["m00"])  # pixel x
                    v = int(M["m01"] / M["m00"])  # pixel y

                    detected.data = True
                    pixel_pos.x = float(u)
                    pixel_pos.y = float(v)
                    pixel_pos.z = float(area)  # area tells us roughly how close the ball is

                    # use depth camera to get actual distance to ball in meters
                    if self.depth_image is not None:
                        try:
                            # look up depth value at the ball pixel location
                            z = float(self.depth_image[v, u])
                            if z > 0.0 and z < 10.0:  # sanity check - ignore zero or huge values
                                # unproject pixel + depth to 3D point using camera intrinsics
                                # this is the standard pinhole camera model equation
                                x_cam = (u - self.cx) * z / self.fx
                                y_cam = (v - self.cy) * z / self.fy

                                # publish 3D ball position in camera frame
                                ball_3d = PointStamped()
                                ball_3d.header.frame_id = "camera"
                                ball_3d.point.x = x_cam  # left/right from camera center
                                ball_3d.point.y = y_cam  # up/down from camera center
                                ball_3d.point.z = z      # forward distance from camera
                                self.ball_3d_pub.publish(ball_3d)
                                self.get_logger().info(f"Ball 3D cam frame: x={x_cam:.3f} y={y_cam:.3f} z={z:.3f}")
                        except Exception as e:
                            pass  # depth lookup can fail at edges, just skip

                    # draw detection results on the debug image
                    cv2.circle(cv_image, (u, v), 10, (0, 255, 0), -1)
                    cv2.drawContours(cv_image, [largest], -1, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Ball ({u},{v}) area:{area:.0f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                detected.data = False  # blob too small to be the ball
        else:
            detected.data = False  # no magenta blobs found at all

        # publish results
        self.detected_pub.publish(detected)
        self.pixel_pub.publish(pixel_pos)
        # send annotated image for viewing in rqt_image_view
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
