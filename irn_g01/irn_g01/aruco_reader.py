from typing import Literal

import cv2
import numpy as np
import rclpy
import time
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from .literals import CAMERA_FRAME, EMPTY_MESSAGE, quaternion_to_rpy, rvec_to_quaternion


DEFAULT_MARKER_LENGTH = 0.18
DEFAULT_ARUCO_ID = 372
DEFAULT_ARUCO_DICT = "DICT_4X4_1000"

DEFAULT_MAX_MARKER_LOST_TIME = 0.0
DEFAULT_MAX_MARKER_LOST_FRAMES = 10
DEFAULT_LOST_CONDITION: Literal["frame", "time", "AND", "OR"] = "OR"

DEFAULT_DETECT_FRAMES_THRESHOLD = 1
DEFAULT_USE_ACTIVATION_MECHANIC_WHEN_LOST = True
DEFAULT_ALWAYS_CHECK_CAMERA_PARAMETERS = False
DEFAULT_OUTPUT_RATE_HZ = 5.0


def get_aruco_dictionary(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module not available. Install opencv-contrib-python.")

    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")

    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))


def create_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()

    return cv2.aruco.DetectorParameters_create()


def create_aruco_detector(aruco_dict, aruco_params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    return None


def detect_markers(detector, aruco_dict, aruco_params, frame):
    if detector is not None:
        return detector.detectMarkers(frame)

    return cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)


def image_to_frame(msg, bridge):
    if isinstance(msg, CompressedImage):
        return bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def make_empty_pose(stamp, frame_id: str) -> PoseStamped:
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.pose.position.x = EMPTY_MESSAGE.pose.position.x
    message.pose.position.y = EMPTY_MESSAGE.pose.position.y
    message.pose.position.z = EMPTY_MESSAGE.pose.position.z
    message.pose.orientation.x = EMPTY_MESSAGE.pose.orientation.x
    message.pose.orientation.y = EMPTY_MESSAGE.pose.orientation.y
    message.pose.orientation.z = EMPTY_MESSAGE.pose.orientation.z
    message.pose.orientation.w = EMPTY_MESSAGE.pose.orientation.w
    return message


class ArucoReader(Node):
    def __init__(self):
        super().__init__("aruco_reader")

        self.declare_parameter("image_topic", "/oakd/rgb/preview/image_raw/compressed")
        self.declare_parameter("compressed_image", True)
        self.declare_parameter("camera_info_topic", "/oakd/rgb/preview/camera_info")
        self.declare_parameter("pose_topic", "/aruco/pose")
        self.declare_parameter("camera_frame", CAMERA_FRAME)
        self.declare_parameter("marker_length", DEFAULT_MARKER_LENGTH)
        self.declare_parameter("aruco_id", DEFAULT_ARUCO_ID)
        self.declare_parameter("aruco_dictionary", DEFAULT_ARUCO_DICT)
        self.declare_parameter("max_marker_lost_time", DEFAULT_MAX_MARKER_LOST_TIME)
        self.declare_parameter("max_marker_lost_frames", DEFAULT_MAX_MARKER_LOST_FRAMES)
        self.declare_parameter("lost_condition", DEFAULT_LOST_CONDITION)
        self.declare_parameter("detect_frames_threshold", DEFAULT_DETECT_FRAMES_THRESHOLD)
        self.declare_parameter("output_rate_hz", DEFAULT_OUTPUT_RATE_HZ)
        self.declare_parameter(
            "use_activation_mechanic_when_lost",
            DEFAULT_USE_ACTIVATION_MECHANIC_WHEN_LOST,
        )
        self.declare_parameter(
            "always_check_camera_parameters",
            DEFAULT_ALWAYS_CHECK_CAMERA_PARAMETERS,
        )

        self._image_topic = self.get_parameter("image_topic").value
        self._compressed_image = bool(self.get_parameter("compressed_image").value)
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        self._pose_topic = self.get_parameter("pose_topic").value
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._marker_length = float(self.get_parameter("marker_length").value)
        self._aruco_id = int(self.get_parameter("aruco_id").value)
        self._aruco_dictionary_name = self.get_parameter("aruco_dictionary").value
        self._detect_frames_threshold = max(
            1,
            int(self.get_parameter("detect_frames_threshold").value),
        )
        self._max_marker_lost_frames = max(
            1,
            int(self.get_parameter("max_marker_lost_frames").value),
        )
        self._publish_period_sec = 1.0 / max(
            0.1,
            float(self.get_parameter("output_rate_hz").value),
        )
        self._use_activation_mechanic_when_lost = bool(
            self.get_parameter("use_activation_mechanic_when_lost").value
        )
        self._always_check_camera_parameters = bool(
            self.get_parameter("always_check_camera_parameters").value
        )

        self._image_msg_type = CompressedImage if self._compressed_image else Image
        self.bridge = CvBridge()
        self._aruco_dict = get_aruco_dictionary(self._aruco_dictionary_name)
        self._aruco_params = create_detector_parameters()
        self._aruco_detector = create_aruco_detector(self._aruco_dict, self._aruco_params)
        self._camera_matrix = None
        self._dist_coeffs = None

        half_length = self._marker_length / 2.0
        self._obj_points = np.array(
            [
                [-half_length, half_length, 0.0],
                [half_length, half_length, 0.0],
                [half_length, -half_length, 0.0],
                [-half_length, -half_length, 0.0],
            ],
            dtype=np.float32,
        )

        self._image_subscription = self.create_subscription(
            self._image_msg_type,
            self._image_topic,
            self._image_callback,
            1,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            1,
        )
        self._pose_publisher = self.create_publisher(PoseStamped, self._pose_topic, 10)

        self._seen_frames = 0
        self._missed_frames = 0
        self._lost_published = False
        self._last_valid_publish_time = None
        self._activation_flag = not self._use_activation_mechanic_when_lost

        self.get_logger().info(
            "Nodo ArucoReader avviato. "
            f"image_topic={self._image_topic}, compressed={self._compressed_image}, "
            f"camera_frame={self._camera_frame}, marker_id={self._aruco_id}, "
            f"output_rate={1.0 / self._publish_period_sec:.1f}Hz, "
            f"lost_frames={self._max_marker_lost_frames}"
        )

    def _camera_info_callback(self, msg: CameraInfo):
        if self._camera_matrix is not None:
            new_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if not np.array_equal(self._camera_matrix, new_matrix):
                self.get_logger().warn("Camera matrix cambiata. Aggiorno i parametri.")

        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)

        self.get_logger().info(
            f"Camera matrix ricevuta:\n{self._camera_matrix}\n"
            f"Distorsione: {self._dist_coeffs}"
        )

        if not self._always_check_camera_parameters:
            self.destroy_subscription(self._camera_info_subscription)

    def _estimate_pose(self, selected_corners):
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                selected_corners,
                self._marker_length,
                self._camera_matrix,
                self._dist_coeffs,
            )
            return np.asarray(rvecs).reshape(-1, 3)[0], np.asarray(tvecs).reshape(-1, 3)[0]

        image_points = selected_corners[0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            self._obj_points,
            image_points,
            self._camera_matrix,
            self._dist_coeffs,
            flags=getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE),
        )
        if not ok:
            raise RuntimeError("solvePnP ha fallito")

        return np.asarray(rvec).reshape(3), np.asarray(tvec).reshape(3)

    def _image_callback(self, msg):
        if self._camera_matrix is None or self._dist_coeffs is None:
            self.get_logger().warn(
                "CameraInfo non ancora ricevuta, salto il frame.",
                throttle_duration_sec=2.0,
            )
            return

        try:
            frame = image_to_frame(msg, self.bridge)
        except Exception as error:
            self.get_logger().error(f"Conversione immagine fallita: {error}")
            return

        corners, ids, _ = detect_markers(
            self._aruco_detector,
            self._aruco_dict,
            self._aruco_params,
            frame,
        )
        ids_flat = ids.flatten() if ids is not None else np.array([])
        marker_detected = len(ids_flat) > 0 and self._aruco_id in ids_flat

        if marker_detected:
            now = time.monotonic()
            reacquired_after_loss = self._lost_published
            self._missed_frames = 0
            self._seen_frames += 1
            if not reacquired_after_loss and self._seen_frames < self._detect_frames_threshold:
                self.get_logger().info(
                    f"Marker {self._aruco_id} rilevato "
                    f"{self._seen_frames}/{self._detect_frames_threshold}."
                )
                # Visualizzazione OpenCV disattivata: il nodo deve solo pubblicare topic.
                # cv2.imshow("ArUco Camera View", frame)
                # cv2.waitKey(1)
                return

            self._activation_flag = True
            should_publish = (
                reacquired_after_loss
                or self._last_valid_publish_time is None
                or now - self._last_valid_publish_time >= self._publish_period_sec
            )
            if should_publish:
                self._publish_marker_pose(msg, frame, corners, ids_flat)
                self._last_valid_publish_time = now
                self._lost_published = False
        else:
            self._seen_frames = 0
            if self._activation_flag:
                self._missed_frames += 1
                if (
                    self._missed_frames >= self._max_marker_lost_frames
                    and not self._lost_published
                ):
                    self._pose_publisher.publish(
                        make_empty_pose(self.get_clock().now().to_msg(), self._camera_frame)
                    )
                    self._lost_published = True
                    self._last_valid_publish_time = None
                    self.get_logger().warn(
                        f"Marker non rilevato per {self._missed_frames} frame: "
                        "pubblicato un solo messaggio vuoto."
                    )

        # Visualizzazione OpenCV disattivata: evita finestre e carico grafico durante i test.
        # cv2.imshow("ArUco Camera View", frame)
        # cv2.waitKey(1)

    def _publish_marker_pose(self, msg, frame, corners, ids_flat):
        marker_index = int(np.where(ids_flat == self._aruco_id)[0][0])
        selected_corners = corners[marker_index]

        try:
            rvec, tvec = self._estimate_pose(selected_corners)
        except Exception as error:
            self.get_logger().error(f"Stima posa ArUco fallita: {error}")
            return

        # Disegno di marker/assi disattivato insieme alla finestra di debug.
        # cv2.aruco.drawDetectedMarkers(frame, corners, ids_flat.reshape(-1, 1))
        # if hasattr(cv2, "drawFrameAxes"):
        #     cv2.drawFrameAxes(
        #         frame,
        #         self._camera_matrix,
        #         self._dist_coeffs,
        #         rvec,
        #         tvec,
        #         self._marker_length * 0.5,
        #     )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self._camera_frame
        pose_msg.pose = rvec_tvec_to_pose(rvec, tvec)
        self._pose_publisher.publish(pose_msg)

        _, _, yaw = quaternion_to_rpy(pose_msg.pose.orientation)
        self.get_logger().info(
            f"Marker {self._aruco_id}: "
            f"x={pose_msg.pose.position.x:.3f}m "
            f"y={pose_msg.pose.position.y:.3f}m "
            f"z={pose_msg.pose.position.z:.3f}m "
            f"yaw={np.degrees(yaw):.1f} deg"
        )


def rvec_tvec_to_pose(rvec, tvec) -> Pose:
    qx, qy, qz, qw = rvec_to_quaternion(rvec)

    pose = Pose()
    pose.position.x = float(tvec[0])
    pose.position.y = float(tvec[1])
    pose.position.z = float(tvec[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def main(args=None):
    rclpy.init(args=args)
    node = ArucoReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Chiusura...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
