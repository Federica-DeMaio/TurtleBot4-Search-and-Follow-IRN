import math

import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.time import Time as RclpyTime


EMPTY_MESSAGE = PoseStamped()
EMPTY_MESSAGE.pose.position.x = 0.0
EMPTY_MESSAGE.pose.position.y = 0.0
EMPTY_MESSAGE.pose.position.z = 0.0
EMPTY_MESSAGE.pose.orientation.x = 0.0
EMPTY_MESSAGE.pose.orientation.y = 0.0
EMPTY_MESSAGE.pose.orientation.z = 0.0
EMPTY_MESSAGE.pose.orientation.w = 1.0

CAMERA_FRAME = "oakd_rgb_camera_optical_frame"
MAX_TRANSFORM_WAIT_TIME: float = 2.0

AVOID_ARUCO_ANGLE = False
TARGET_DISTANCE = 0.60
GOAL_DISTANCE_TOLERANCE = 0.10
TARGET_OFFSET = GOAL_DISTANCE_TOLERANCE
ANGLE_OFFSET = math.radians(15)

CAMERA_POSITION_UNCERTAINTY = 0.05
CAMERA_ANGLE_UNCERTAINTY = math.radians(35)
SIGMA_ACCEL = 1.5
SIGMA_ALPHA = 1.5


class TransformException(Exception):
    """Raised when a required TF transform is not available."""


def kalman_dependencies():
    try:
        from filterpy.common import Q_discrete_white_noise
        from filterpy.kalman import KalmanFilter
        from scipy.linalg import block_diag
    except ImportError as error:
        raise RuntimeError(
            "filterpy/scipy non sono installati. Servono solo per i nodi che "
            "usano KalmanTracker, come compute_pose o predictive_follower. "
            "Installa con: python3 -m pip install --user filterpy scipy"
        ) from error

    return KalmanFilter, Q_discrete_white_noise, block_diag


def is_aruco_pose_empty(msg: PoseStamped) -> bool:
    return (
        msg.pose.position.x == EMPTY_MESSAGE.pose.position.x
        and msg.pose.position.y == EMPTY_MESSAGE.pose.position.y
        and msg.pose.position.z == EMPTY_MESSAGE.pose.position.z
        and msg.pose.orientation.x == EMPTY_MESSAGE.pose.orientation.x
        and msg.pose.orientation.y == EMPTY_MESSAGE.pose.orientation.y
        and msg.pose.orientation.z == EMPTY_MESSAGE.pose.orientation.z
        and msg.pose.orientation.w == EMPTY_MESSAGE.pose.orientation.w
    )


def normalize_angle(angle: float) -> float:
    """Riporta un angolo equivalente nell'intervallo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def rvec_to_quaternion(rvec) -> tuple[float, float, float, float]:
    """Converte rotation vector (Rodrigues) in quaternione (x, y, z, w)."""
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    trace = (
        rotation_matrix[0, 0]
        + rotation_matrix[1, 1]
        + rotation_matrix[2, 2]
    )
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return (
            (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s,
            (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s,
            (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s,
            0.25 / s,
        )

    if (
        rotation_matrix[0, 0] > rotation_matrix[1, 1]
        and rotation_matrix[0, 0] > rotation_matrix[2, 2]
    ):
        s = 2.0 * np.sqrt(
            1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]
        )
        return (
            0.25 * s,
            (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s,
            (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s,
            (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s,
        )

    if rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = 2.0 * np.sqrt(
            1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]
        )
        return (
            (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s,
            0.25 * s,
            (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s,
            (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s,
        )

    s = 2.0 * np.sqrt(
        1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]
    )
    return (
        (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s,
        (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s,
        0.25 * s,
        (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s,
    )


def quaternion_to_rpy(q: Quaternion) -> tuple[float, float, float]:
    """
    Converte un quaternione (x, y, z, w)
    in roll, pitch, yaw (radianti).
    """
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_to_yaw(q: Quaternion) -> float:
    """Estrae solo lo yaw da un quaternione."""
    return quaternion_to_rpy(q)[2]


def yaw_to_quaternion(yaw) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def linear_angle_distances(old_pose: Pose, new_pose: Pose) -> tuple[float, float]:
    """Determines the linear and angular distances between two poses."""
    dx = new_pose.position.x - old_pose.position.x
    dy = new_pose.position.y - old_pose.position.y
    distance = math.sqrt(dx**2 + dy**2)
    old_yaw = quaternion_to_yaw(old_pose.orientation)
    new_yaw = quaternion_to_yaw(new_pose.orientation)
    angle_diff = normalize_angle(new_yaw - old_yaw)
    return distance, angle_diff


class KalmanTracker:
    """
    Stato: [x, vx, y, vy, yaw, omega]
    Misura: [x, y, yaw] dall'ArUco
    Modello a velocita costante, lineare: niente EKF.
    """

    def __init__(self):
        self._kf = self._build_filter()
        self._last_stamp: RclpyTime | None = None
        self._measurement_count = 0

    def _build_filter(self):
        KalmanFilter, _q_discrete_white_noise, _block_diag = kalman_dependencies()
        kf = KalmanFilter(dim_x=6, dim_z=3)

        kf.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
            ]
        )

        kf.R = np.diag(
            [
                CAMERA_POSITION_UNCERTAINTY**2,
                CAMERA_POSITION_UNCERTAINTY**2,
                CAMERA_ANGLE_UNCERTAINTY**2,
            ]
        )

        kf.F = np.eye(6)
        kf.Q = np.eye(6)
        kf.P = np.diag([1.0, 10.0, 1.0, 10.0, 0.5, 5.0])

        return kf

    def _F(self, dt: float) -> np.ndarray:
        """Matrice di transizione a velocita costante."""
        return np.array(
            [
                [1, dt, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, dt, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 1],
            ]
        )

    def _Q(self, dt: float) -> np.ndarray:
        """Rumore di processo: modello ad accelerazione come rumore bianco."""
        _kalman_filter, Q_discrete_white_noise, block_diag = kalman_dependencies()
        q_xy = Q_discrete_white_noise(dim=2, dt=dt, var=SIGMA_ACCEL**2)
        q_ang = Q_discrete_white_noise(dim=2, dt=dt, var=SIGMA_ALPHA**2)
        return block_diag(q_xy, q_xy, q_ang)

    def _do_predict(self, dt: float) -> Pose:
        self._kf.F = self._F(dt)
        self._kf.Q = self._Q(dt)
        self._kf.predict()
        return self.estimated_pose

    def update(self, pose: PoseStamped) -> None:
        """Chiamare a ogni frame ArUco valido: predict(dt) + update."""
        stamp = RclpyTime.from_msg(pose.header.stamp)

        yaw = quaternion_to_yaw(pose.pose.orientation)
        saved_yaw = self._kf.x[4] if self._last_stamp is not None else yaw

        if self._last_stamp is not None:
            yaw = saved_yaw + normalize_angle(yaw - saved_yaw)

        z = np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                yaw,
            ]
        )

        if self._last_stamp is None:
            self._kf.x = np.array([z[0], 0.0, z[1], 0.0, yaw, 0.0])
            self._last_stamp = stamp
            self._measurement_count = 1
            return

        dt = (stamp - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            get_logger("irn_g01").error(
                f"Stamp non crescente: {self._last_stamp} -> {stamp}"
            )
            return

        self._do_predict(dt)
        self._kf.update(z)
        self._last_stamp = stamp
        self._measurement_count += 1

    def predict_only(self, current_time: RclpyTime) -> Pose:
        """
        Chiamare quando il marker non e' visibile.
        Propaga lo stato usando le velocita stimate, senza correzione.
        L'incertezza (P) cresce con il tempo trascorso.
        """
        if self._last_stamp is None:
            raise ValueError(
                "Impossibile fare la predizione: nessun timestamp valido presente."
            )

        dt = (current_time - self._last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            get_logger("irn_g01").error(
                f"Stamp non crescente: {self._last_stamp} -> {current_time}"
            )
            raise ValueError("Timestamp non crescente: impossibile predire.")

        prediction = self._do_predict(dt)
        self._last_stamp = current_time
        return prediction

    @property
    def estimated_pose(self) -> Pose:
        if self._last_stamp is None:
            raise ValueError(
                "Impossibile stimare la posizione: nessun timestamp valido presente."
            )

        pose = Pose()
        pose.position.x = float(self._kf.x[0])
        pose.position.y = float(self._kf.x[2])
        pose.position.z = 0.0
        yaw = float(self._kf.x[4])
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = yaw_to_quaternion(yaw)
        return pose

    @property
    def measurement_count(self) -> int:
        return self._measurement_count

    def reset(self) -> None:
        self._kf = self._build_filter()
        self._last_stamp = None
        self._measurement_count = 0


def get_position(tf_buffer: tf2_ros.Buffer) -> Pose:
    """
    Returns the current robot position in map using TF.

    Raises an exception if the transform is not available.
    """
    try:
        transform = tf_buffer.lookup_transform(
            "map",
            "base_link",
            RclpyTime(),
            timeout=Duration(seconds=MAX_TRANSFORM_WAIT_TIME),
        )
    except Exception as error:
        raise TransformException(str(error)) from error

    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation = transform.transform.rotation
    return pose
