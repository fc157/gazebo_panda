#!/usr/bin/env python3
"""
aruco_detector.py

ROS 2 Python node for real-time Aruco marker detection from a hand-eye camera.

Subscribes:
  - /hand_camera/image_raw   (sensor_msgs/Image)   — RGB camera feed

Publishes:
  - /detected_aruco          (detected_aruco)       — All detected markers in one msg
  - /detected_aruco_image    (sensor_msgs/Image)    — Debug image with annotations

The detected_aruco message has the following fields:
  - header.stamp       — timestamp
  - marker_ids         — int32[]   list of detected marker IDs
  - poses              — Pose[]    estimated pose of each marker in camera frame
  - marker_sizes       — float64[] physical size (side length) of each marker

Camera intrinsic parameters are read from ROS 2 parameters or use defaults
matching the Gazebo hand_camera configuration:
  - width: 640, height: 480
  - fx: 554.0, fy: 554.0, cx: 320.0, cy: 240.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---- Parameters ----
        self.declare_parameter('camera_topic', '/hand_camera/image_raw')
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        self.declare_parameter('marker_size', 0.2)          # 0.2m = 20cm
        self.declare_parameter('publish_debug_image', True)

        # Camera intrinsic parameters (matching Gazebo hand_camera)
        self.declare_parameter('camera_fx', 554.0)
        self.declare_parameter('camera_fy', 554.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        camera_topic = self.get_parameter('camera_topic').value
        dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # Camera matrix and distortion coefficients
        fx = self.get_parameter('camera_fx').value
        fy = self.get_parameter('camera_fy').value
        cx = self.get_parameter('camera_cx').value
        cy = self.get_parameter('camera_cy').value
        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0,  0,  1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ---- Setup Aruco dictionary ----
        aruco_dict_map = {
            'DICT_4X4_50':     aruco.DICT_4X4_50,
            'DICT_4X4_100':    aruco.DICT_4X4_100,
            'DICT_4X4_250':    aruco.DICT_4X4_250,
            'DICT_4X4_1000':   aruco.DICT_4X4_1000,
            'DICT_5X5_50':     aruco.DICT_5X5_50,
            'DICT_5X5_100':    aruco.DICT_5X5_100,
            'DICT_5X5_250':    aruco.DICT_5X5_250,
            'DICT_5X5_1000':   aruco.DICT_5X5_1000,
            'DICT_6X6_50':     aruco.DICT_6X6_50,
            'DICT_6X6_100':    aruco.DICT_6X6_100,
            'DICT_6X6_250':    aruco.DICT_6X6_250,       # default
            'DICT_6X6_1000':   aruco.DICT_6X6_1000,
            'DICT_7X7_50':     aruco.DICT_7X7_50,
            'DICT_7X7_100':    aruco.DICT_7X7_100,
            'DICT_7X7_250':    aruco.DICT_7X7_250,
            'DICT_7X7_1000':   aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL,
        }
        dict_id = aruco_dict_map.get(dict_name, aruco.DICT_6X6_250)
        self.aruco_dict = aruco.Dictionary_get(dict_id)
        self.parameters = aruco.DetectorParameters_create()

        # ---- Bridge ----
        self.bridge = CvBridge()

        # ---- Subscribers ----
        self.sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.get_logger().info(
            f'Subscribing to: {camera_topic} (dict={dict_name})')

        # ---- Publishers ----
        # Use a custom message format: we publish a PoseArray-like structure
        # via multiple topics for compatibility
        self.marker_pose_pub = self.create_publisher(
            PoseStamped, '/detected_aruco_single', 10)
        self.marker_ids_pub = self.create_publisher(
            Image, '/detected_aruco_image', 10) if self.publish_debug else None

        # We'll use a simple representation: publish individual pose per marker ID
        # + a combined visualization image

        self.get_logger().info('Aruco detector node initialized')

    def image_callback(self, msg):
        """Process each incoming camera frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Detect Aruco markers
        corners, ids, rejected = aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.parameters)

        # Draw detected markers for debug image
        debug_image = cv_image.copy()

        if ids is not None and len(ids) > 0:
            # Draw bounding boxes and IDs
            aruco.drawDetectedMarkers(debug_image, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                rvec = rvecs[i][0]  # rotation vector (3,)
                tvec = tvecs[i][0]  # translation vector (3,)

                # Draw axis
                aruco.drawAxis(debug_image, self.camera_matrix,
                               self.dist_coeffs, rvec, tvec, self.marker_size * 0.5)

                # Build PoseStamped in camera optical frame
                pose_msg = PoseStamped()
                pose_msg.header = Header(
                    stamp=msg.header.stamp,
                    frame_id='panda_camera_optical_frame'
                )

                # Translation: tvec is in camera optical frame
                pose_msg.pose.position = Point(
                    x=float(tvec[0]),
                    y=float(tvec[1]),
                    z=float(tvec[2])
                )

                # Convert rotation vector to quaternion
                rot_mat, _ = cv2.Rodrigues(rvec)
                # OpenCV rotation matrix -> quaternion
                # The axis directions: camera optical frame (x-right, y-down, z-forward)
                q = self._rotmat_to_quat(rot_mat)
                pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                # Add marker ID as frame_id suffix for identification
                pose_msg.header.frame_id = f'panda_camera_optical_frame/marker_{marker_id}'

                # Publish individual marker pose
                self.marker_pose_pub.publish(pose_msg)

                # Log detection
                self.get_logger().info(
                    f'Detected Aruco ID={marker_id} '
                    f'pos=({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f})',
                    throttle_duration_sec=1.0)

        else:
            self.get_logger().debug('No Aruco markers detected', throttle_duration_sec=2.0)

        # Publish debug image
        if self.publish_debug and self.marker_ids_pub:
            try:
                debug_ros = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_ros.header = msg.header
                self.marker_ids_pub.publish(debug_ros)
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge error (debug): {e}')

    def _rotmat_to_quat(self, R):
        """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()