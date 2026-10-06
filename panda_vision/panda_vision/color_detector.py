#!/usr/bin/env python3
"""Color detector with geometric back-projection and adaptive segmentation.

Publishes "<color>,<x>,<y>,<z>" on /color_coordinates, where (x, y, z) is the
object position in the robot base frame. Unlike the original detector there
are no hand-tuned depth, scale or per-colour offsets: each pixel is turned
into a ray from the camera's TF pose and intersected with the table plane.
"""
import time

import cv2
import rclpy
import tf2_ros
import tf_transformations
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from panda_vision import camera_geometry as cg
from panda_vision.adaptive_hsv import segment_colors


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.declare_parameter('base_frame', 'panda_link0')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('horizontal_fov', 1.0)
        # Height of the plane the detected pixel centroids lie on (top
        # face of the objects) in the base frame.
        self.declare_parameter('plane_z', 0.0)
        self.declare_parameter('min_area', 20)
        self.declare_parameter('show_window', True)
        # Cap on processed frames per second; the camera runs faster than needed.
        self.declare_parameter('max_rate_hz', 5.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.fov = float(self.get_parameter('horizontal_fov').value)
        self.plane_z = float(self.get_parameter('plane_z').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.show = bool(self.get_parameter('show_window').value)
        self.min_period = 1.0 / float(self.get_parameter('max_rate_hz').value)
        self.last_processed = 0.0

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.get_logger().info(
            f"Geometric color detector up (plane_z={self.plane_z:.3f} m)")

    def camera_pose(self):
        t = self.tf_buffer.lookup_transform(
            self.base_frame, self.camera_frame, rclpy.time.Time(),
            timeout=Duration(seconds=1.0))
        T = tf_transformations.quaternion_matrix([
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w])
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z]
        return T

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self.last_processed < self.min_period:
            return
        self.last_processed = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        try:
            T = self.camera_pose()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        fx, fy, cx, cy = cg.intrinsics_from_fov(w, h, self.fov)
        for color_id, blobs in segment_colors(
                frame, min_area=self.min_area).items():
            for (u, v, _area) in blobs:
                ray = cg.pixel_rays([[u, v]], fx, fy, cx, cy,
                                    cg.R_CAM_FROM_OPTICAL_TOP_DOWN)
                x, y, z = cg.intersect_plane(T, ray, self.plane_z)[0]
                self.coords_pub.publish(
                    String(data=f"{color_id},{x:.4f},{y:.4f},{z:.4f}"))
                cv2.rectangle(frame, (int(u) - 12, int(v) - 12),
                              (int(u) + 12, int(v) + 12), (0, 255, 255), 2)
                cv2.putText(frame, color_id, (int(u) - 12, int(v) - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if self.show:
            try:
                cv2.imshow("Color Detection", frame)
                cv2.waitKey(1)
            except cv2.error:
                self.show = False


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
