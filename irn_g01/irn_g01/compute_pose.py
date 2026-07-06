import math
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from std_msgs.msg import Empty
import tf2_geometry_msgs  # Registra le trasformazioni PoseStamped dentro tf2.

from .literals import (
    ANGLE_OFFSET,
    CAMERA_FRAME,
    GOAL_DISTANCE_TOLERANCE,
    KalmanTracker,
    MAX_TRANSFORM_WAIT_TIME,
    TARGET_DISTANCE,
    TransformException,
    get_position,
    is_aruco_pose_empty,
    linear_angle_distances,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


RETREAT_WHEN_TOO_CLOSE = False
POSE_TRANSFORM_TIMEOUT = 0.05
MIN_VISIBLE_DETECTIONS_FOR_NAVIGATION = 3
MIN_KALMAN_MEASUREMENTS_FOR_PREDICTION = 5


class ComputePose(Node):
    def __init__(self):
        super().__init__("compute_pose")
        self._tracker = KalmanTracker()
        self._subscription = self.create_subscription(
            PoseStamped,
            "/aruco/pose",
            self.pose_callback,
            10,
        )
        self._visible_goal_publisher = self.create_publisher(PoseStamped, "/visible_goal", 10)
        self._predicted_goal_publisher = self.create_publisher(PoseStamped, "/predicted_goal", 10)
        self._target_lost_publisher = self.create_publisher(Empty, "/target_lost", 10)
        self._target_hold_publisher = self.create_publisher(Empty, "/target_hold", 10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._timer_tf = self.create_timer(
            MAX_TRANSFORM_WAIT_TIME + 2.0,
            self._check_transform_available,
        )
        self._tf_ready = False
        self._prediction_sent_for_current_loss = False
        self._target_hold_published = False
        self._last_approach_direction_xy: tuple[float, float] | None = None
        self._visible_detection_count = 0

        self.get_logger().info(
            "ComputePose node initialized. Publishing /visible_goal, "
            "/predicted_goal, /target_lost and /target_hold"
        )

    def _check_transform_available(self):
        try:
            if self._tf_buffer.can_transform(
                "map",
                CAMERA_FRAME,
                RclpyTime(),
                timeout=Duration(seconds=MAX_TRANSFORM_WAIT_TIME),
            ):
                self.get_logger().info(
                    "TF {} -> map disponibile, pronto a trasformare pose.".format(
                        CAMERA_FRAME
                    )
                )
                self._tf_ready = True
                self._timer_tf.cancel()
            else:
                self.get_logger().warn(
                    "TF {} -> map non disponibile, ritentando...".format(CAMERA_FRAME)
                )
        except Exception as error:
            self.get_logger().warn(
                f"Errore durante l'attesa del TF: {str(error)}. Ritentando..."
            )

    def _map_pose_log(self, label: str, pose: Pose) -> str:
        yaw = math.degrees(quaternion_to_yaw(pose.orientation))
        return (
            f"{label}: x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, "
            f"z={pose.position.z:.3f}, "
            f"yaw={yaw:.1f} deg"
        )

    def _approach_log(self, approach_x: float, approach_y: float) -> str:
        approach_yaw = math.degrees(math.atan2(approach_y, approach_x))
        return (
            "approach marker->robot: "
            f"x={approach_x:.3f}, y={approach_y:.3f}, yaw={approach_yaw:.1f} deg"
        )

    def pose_callback(self, msg: PoseStamped):
        if not self._tf_ready:
            return

        if is_aruco_pose_empty(msg):
            self._handle_marker_lost(msg)
            return

        self._handle_marker_visible(msg)

    def _handle_marker_lost(self, msg: PoseStamped):
        if self._prediction_sent_for_current_loss:
            return

        self._visible_detection_count = 0
        if self._tracker.measurement_count < MIN_KALMAN_MEASUREMENTS_FOR_PREDICTION:
            self.get_logger().warn(
                "Marker perso senza abbastanza misure per predire: "
                f"{self._tracker.measurement_count}/"
                f"{MIN_KALMAN_MEASUREMENTS_FOR_PREDICTION}."
            )
            self._prediction_sent_for_current_loss = True
            self._target_hold_published = False
            self._target_lost_publisher.publish(Empty())
            self._tracker.reset()
            return

        try:
            predicted_marker_pose = self._tracker.predict_only(
                RclpyTime.from_msg(msg.header.stamp)
            )
        except Exception as error:
            self.get_logger().warn(
                f"Marker perso senza una predizione valida: {str(error)}"
            )
            self._prediction_sent_for_current_loss = True
            self._target_hold_published = False
            self._target_lost_publisher.publish(Empty())
            self._tracker.reset()
            return

        marker_msg = PoseStamped()
        marker_msg.header.stamp = msg.header.stamp
        marker_msg.header.frame_id = "map"
        marker_msg.pose = predicted_marker_pose

        try:
            robot_pose = get_position(self._tf_buffer)
        except Exception:
            robot_pose = None

        approach_x, approach_y = approach_direction_from_marker_to_robot(
            marker_msg.pose,
            robot_pose,
            self._last_approach_direction_xy,
        )

        self.get_logger().warn(
            "DEBUG predicted map | "
            f"{self._map_pose_log('aruco', marker_msg.pose)} | "
            f"{self._approach_log(approach_x, approach_y)}"
        )

        goal_msg = self._robot_goal_from_marker_pose(marker_msg, approach_x, approach_y)
        self._target_hold_published = False
        self._predicted_goal_publisher.publish(goal_msg)
        self._prediction_sent_for_current_loss = True
        self._tracker.reset()

        self.get_logger().warn(
            "Marker perso: pubblicata una sola posa predetta "
            f"x={goal_msg.pose.position.x:.2f}, "
            f"y={goal_msg.pose.position.y:.2f}, "
            f"yaw={math.degrees(quaternion_to_yaw(goal_msg.pose.orientation)):.1f}"
        )

    def _handle_marker_visible(self, msg: PoseStamped):
        self._prediction_sent_for_current_loss = False
        self.get_logger().info(
            f"Received ArUco pose: x{msg.pose.position.x:.2f}, "
            f"y{msg.pose.position.y:.2f}, z{msg.pose.position.z:.2f}"
        )

        try:
            robot_pose = get_position(self._tf_buffer)
            marker_pose_msg = self._marker_pose_in_map(msg)
        except Exception as error:
            self.get_logger().warn(f"Impossibile calcolare posa ArUco in map: {error}")
            return

        # Quando l'ArUco e' visibile, il goal nasce dalla misura reale appena
        # calcolata. Il Kalman viene aggiornato solo come memoria interna, utile
        # alla prossima eventuale perdita del marker.
        approach_x, approach_y = approach_direction_from_marker_to_robot(
            marker_pose_msg.pose,
            robot_pose,
            self._last_approach_direction_xy,
        )
        self._last_approach_direction_xy = (approach_x, approach_y)

        self.get_logger().info(
            "DEBUG visible map | "
            f"{self._map_pose_log('aruco', marker_pose_msg.pose)} | "
            f"{self._map_pose_log('robot', robot_pose)} | "
            f"{self._approach_log(approach_x, approach_y)}"
        )

        goal_msg = self._robot_goal_from_marker_pose(marker_pose_msg, approach_x, approach_y)
        self._tracker.update(marker_pose_msg)
        self._visible_detection_count += 1

        linear_distance, angle_distance = linear_angle_distances(
            robot_pose,
            goal_msg.pose,
        )
        if linear_distance < GOAL_DISTANCE_TOLERANCE and abs(angle_distance) < ANGLE_OFFSET:
            self._publish_target_hold_once(
                "Marker troppo vicino e gia ben orientato: pubblico target_hold."
            )
            return

        if not RETREAT_WHEN_TOO_CLOSE:
            robot_to_marker = linear_angle_distances(
                robot_pose,
                marker_pose_msg.pose,
            )[0]
            goal_to_marker = linear_angle_distances(
                goal_msg.pose,
                marker_pose_msg.pose,
            )[0]
            if goal_to_marker > robot_to_marker:
                self._publish_target_hold_once(
                    "Marker troppo vicino: evito retromarcia e pubblico target_hold."
                )
                return

        if self._visible_detection_count < MIN_VISIBLE_DETECTIONS_FOR_NAVIGATION:
            self._publish_target_hold_once(
                "Marker visibile: stop immediato, attendo detection "
                f"{MIN_VISIBLE_DETECTIONS_FOR_NAVIGATION} prima del goal Nav2 "
                f"({self._visible_detection_count}/"
                f"{MIN_VISIBLE_DETECTIONS_FOR_NAVIGATION})."
            )
            return

        self._target_hold_published = False
        self._visible_goal_publisher.publish(goal_msg)

        self.get_logger().info(
            "Marker visibile, goal robot pubblicato da misura reale: "
            f"x={goal_msg.pose.position.x:.2f}, "
            f"y={goal_msg.pose.position.y:.2f}, "
            f"yaw={math.degrees(quaternion_to_yaw(goal_msg.pose.orientation)):.1f}"
        )

    def _publish_target_hold_once(self, reason: str):
        if self._target_hold_published:
            return

        self.get_logger().info(reason)
        self._target_hold_publisher.publish(Empty())
        self._target_hold_published = True

    def _marker_pose_in_map(self, msg: PoseStamped) -> PoseStamped:
        source_frame = msg.header.frame_id or CAMERA_FRAME
        stamped = PoseStamped()
        stamped.header.frame_id = source_frame
        stamped.header.stamp = msg.header.stamp
        stamped.pose = msg.pose
        source_time = RclpyTime.from_msg(msg.header.stamp)

        if self._tf_buffer.can_transform(
            "map",
            source_frame,
            source_time,
            timeout=Duration(seconds=POSE_TRANSFORM_TIMEOUT),
        ):
            self.get_logger().info(
                "DEBUG marker TF | uso timestamp immagine | "
                f"source_frame={source_frame}, "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            )
            marker_msg = self._tf_buffer.transform(
                stamped,
                "map",
                timeout=Duration(seconds=POSE_TRANSFORM_TIMEOUT),
            )
            marker_msg.pose.position.z = 0.0
            return marker_msg

        # Le immagini della camera possono arrivare con timestamp leggermente
        # fuori dalla cache TF. Per il follow preferiamo usare il TF piu recente
        # invece di bloccare il nodo o perdere tutte le detection.
        self.get_logger().warn(
            "DEBUG marker TF | timestamp immagine non trasformabile, uso TF piu recente | "
            f"source_frame={source_frame}, "
            f"stamp_originale={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        )
        stamped.header.stamp = RclpyTime().to_msg()
        if not self._tf_buffer.can_transform(
            "map",
            source_frame,
            RclpyTime(),
            timeout=Duration(seconds=POSE_TRANSFORM_TIMEOUT),
        ):
            raise TransformException(f"Transform {source_frame} -> map non disponibile")

        marker_msg = self._tf_buffer.transform(
            stamped,
            "map",
            timeout=Duration(seconds=POSE_TRANSFORM_TIMEOUT),
        )
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.pose.position.z = 0.0
        return marker_msg

    def _robot_goal_from_marker_pose(
        self,
        marker_msg: PoseStamped,
        approach_x: float,
        approach_y: float,
    ) -> PoseStamped:
        goal_x = marker_msg.pose.position.x + TARGET_DISTANCE * approach_x
        goal_y = marker_msg.pose.position.y + TARGET_DISTANCE * approach_y
        goal_yaw = math.atan2(
            marker_msg.pose.position.y - goal_y,
            marker_msg.pose.position.x - goal_x,
        )
        qx, qy, qz, qw = yaw_to_quaternion(goal_yaw)

        goal_msg = PoseStamped()
        goal_msg.header = marker_msg.header
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return goal_msg

    def _transform_marker_pose_to_map(self, marker_pose: PoseStamped | Pose) -> Pose:
        """Transforms the marker pose from frame to map frame using TF."""
        try:
            if isinstance(marker_pose, PoseStamped):
                return self._marker_pose_in_map(marker_pose).pose

            stamped = PoseStamped()
            stamped.header.frame_id = CAMERA_FRAME
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.pose = marker_pose
            return self._marker_pose_in_map(stamped).pose

        except Exception as error:
            self.get_logger().warn(
                f"Errore durante la trasformazione del marker in mappa: {str(error)}"
            )
            raise TransformException(
                f"Errore durante la trasformazione del marker in mappa: {str(error)}"
            )


def marker_normal_xy(marker_orientation: Quaternion) -> tuple[float, float]:
    """Restituisce la normale +Z del marker proiettata sul piano mappa."""
    x = marker_orientation.x
    y = marker_orientation.y
    z = marker_orientation.z
    w = marker_orientation.w

    normal_x = 2.0 * (x * z + w * y)
    normal_y = 2.0 * (y * z - w * x)
    norm = math.hypot(normal_x, normal_y)
    if norm < 1e-6:
        marker_yaw = quaternion_to_yaw(marker_orientation)
        return math.cos(marker_yaw), math.sin(marker_yaw)

    return normal_x / norm, normal_y / norm


def approach_direction_from_marker_to_robot(
    marker_map_pose: Pose,
    robot_pose: Pose | None,
    fallback_direction: tuple[float, float] | None = None,
) -> tuple[float, float]:
    """Calcola la direzione orizzontale stabile dal marker verso il robot."""
    if robot_pose is not None:
        dx = robot_pose.position.x - marker_map_pose.position.x
        dy = robot_pose.position.y - marker_map_pose.position.y
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            return dx / norm, dy / norm

    if fallback_direction is not None:
        return fallback_direction

    marker_yaw = quaternion_to_yaw(marker_map_pose.orientation)
    return math.cos(marker_yaw), math.sin(marker_yaw)


def marker_approach_direction_xy(
    marker_map_pose: Pose,
    robot_pose: Pose | None = None,
) -> tuple[float, float]:
    """Sceglie il verso della normale che sta dal lato corrente del robot."""
    normal_x, normal_y = marker_normal_xy(marker_map_pose.orientation)

    if robot_pose is None:
        return normal_x, normal_y

    marker_to_robot_x = robot_pose.position.x - marker_map_pose.position.x
    marker_to_robot_y = robot_pose.position.y - marker_map_pose.position.y
    if math.hypot(marker_to_robot_x, marker_to_robot_y) < 1e-6:
        return normal_x, normal_y

    if normal_x * marker_to_robot_x + normal_y * marker_to_robot_y < 0.0:
        return -normal_x, -normal_y

    return normal_x, normal_y


def compute_front_pose(marker_map_pose: Pose, distance: float) -> Pose:
    """
    Calcola una posa davanti al marker usando la normale +Z del marker in mappa.
    """
    normal_x, normal_y = marker_approach_direction_xy(marker_map_pose)

    front = Pose()
    front.position.x = marker_map_pose.position.x + normal_x * distance
    front.position.y = marker_map_pose.position.y + normal_y * distance
    front.position.z = 0.0

    opposed_yaw = math.atan2(
        marker_map_pose.position.y - front.position.y,
        marker_map_pose.position.x - front.position.x,
    )
    qx, qy, qz, qw = yaw_to_quaternion(opposed_yaw)
    front.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    return front


def main(args=None):
    rclpy.init(args=args)
    node = ComputePose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
