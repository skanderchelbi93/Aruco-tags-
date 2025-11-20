import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(R: np.ndarray):
    """
    Convert a proper 3x3 rotation matrix to (x, y, z, w) quaternion.
    Robust version that avoids NaNs for small numerical errors.
    """
    trace = np.trace(R)

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0  # s = 4*qw
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        # Find the largest diagonal element and proceed accordingly
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    return qx, qy, qz, qw


class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_pose_node")

        self.bridge = CvBridge()

        # Subscribe to RealSense color stream
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10
        )

        # Subscribe to camera calibration info
        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10
        )

        # Publish annotated image to RViz2
        self.image_pub = self.create_publisher(
            Image,
            "/aruco/annotated_image",
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera intrinsics (filled later)
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco detector (OpenCV contrib)
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL
        )
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.get_logger().info("Aruco multi-marker + TF node started.")

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            marker_size = 0.05  # meters

            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i],
                    marker_size,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                rvec_i = rvec[0][0]
                tvec_i = tvec[0][0]

                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec_i,
                    tvec_i,
                    0.05
                )

                self.publish_tf(ids[i][0], rvec_i, tvec_i, msg.header.stamp)

        # publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        # show OpenCV view
        cv2.imshow("Aruco Detection (OpenCV View)", frame)
        cv2.waitKey(1)

    def publish_tf(self, marker_id, rvec, tvec, stamp):
        # Rotation OpenCV -> matrix
        R_cv, _ = cv2.Rodrigues(rvec)

        # OpenCV camera frame -> ROS optical frame
        R_fix = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
        ])

        R = R_cv @ R_fix

        if np.isnan(R).any():
            self.get_logger().warn(f"NaN in rotation matrix for marker {marker_id}, skipping TF")
            return

        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        if any(np.isnan(v) for v in (qx, qy, qz, qw)):
            self.get_logger().warn(f"NaN quaternion for marker {marker_id}, skipping TF")
            return

        # translation fix for ROS optical frame
        t = np.array([tvec[0], -tvec[1], -tvec[2]])

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "camera_color_optical_frame"
        tf_msg.child_frame_id = f"aruco_marker_{marker_id}"

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])

        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
