import math
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Empty

from .literals import (
    ANGLE_OFFSET,
    GOAL_DISTANCE_TOLERANCE,
    linear_angle_distances,
    quaternion_to_yaw,
)


LOST_SPIN_RATE_HZ = 5.0
LOST_ANGULAR_SPEED = 0.30
LOCAL_COSTMAP_CLEAR_SERVICE = "/local_costmap/clear_entirely_local_costmap"
FOLLOW_COMPARE_DISTANCE_TOLERANCE = 0.20
FOLLOW_COMPARE_ANGLE_TOLERANCE = math.radians(25)


class State(Enum):
    IDLE = 0
    FOLLOWING_VISIBLE = 1
    FOLLOWING_PREDICTED = 2
    LOST = 3


def compare_poses(
    pose1: Pose,
    pose2: Pose,
    target_offset: float = GOAL_DISTANCE_TOLERANCE,
    angle_offset: float = ANGLE_OFFSET,
) -> bool:
    linear_distance, angle_distance = linear_angle_distances(pose1, pose2)
    return linear_distance < target_offset and abs(angle_distance) < angle_offset


class PredictiveFollowerFSM(Node):
    def __init__(self):
        super().__init__("PredictiveFollowerFSM")
        self.declare_parameter("behavior_tree", "")
        self._behavior_tree = str(self.get_parameter("behavior_tree").value)

        self.get_logger().info("PredictiveFollowerFSM node started.")
        if self._behavior_tree:
            self.get_logger().info(f"Using behavior tree: {self._behavior_tree}")
        else:
            self.get_logger().warn(
                "No behavior_tree parameter set; bt_navigator will use its default tree."
            )

        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            LOCAL_COSTMAP_CLEAR_SERVICE,
        )
        self._direct_control = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        while not self._navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server non disponibile.")

        self._visible_goal_listener = self.create_subscription(
            PoseStamped,
            "/visible_goal",
            self._visible_goal_callback,
            10,
        )
        self._predicted_goal_listener = self.create_subscription(
            PoseStamped,
            "/predicted_goal",
            self._predicted_goal_callback,
            10,
        )
        self._target_lost_listener = self.create_subscription(
            Empty,
            "/target_lost",
            self._target_lost_callback,
            10,
        )
        self._target_hold_listener = self.create_subscription(
            Empty,
            "/target_hold",
            self._target_hold_callback,
            10,
        )
        self._lost_spin_timer = self.create_timer(
            1.0 / LOST_SPIN_RATE_HZ,
            self._lost_spin_tick,
        )
        self._lost_spin_timer.cancel()

        self._state = State.IDLE
        self._goal_sequence = 0
        self._current_goal_handle: ClientGoalHandle | None = None
        self._active_goal_pose: Pose | None = None
        self._active_goal_is_predicted = False
        self._last_goal_pose: Pose | None = None
        self._target_visible = False
        self._pending_predicted_goal: Pose | None = None
        self._clearing_after_lost_reacquire = False
        self._reacquired_visible_goal: Pose | None = None
        self._lost_spin_active = False

    def set_state(self, new_state: State):
        if self._state == new_state:
            return

        self.get_logger().info(f"State transition: {self._state.name} -> {new_state.name}")
        self._state = new_state

    def _has_navigation_state(self) -> bool:
        return (
            self._state in (State.FOLLOWING_VISIBLE, State.FOLLOWING_PREDICTED, State.LOST)
            or self._current_goal_handle is not None
            or self._active_goal_pose is not None
        )

    def _lost_spin_tick(self):
        if self._state != State.LOST:
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = LOST_ANGULAR_SPEED
        self._direct_control.publish(twist)

    def _publish_zero_twist(self):
        self._direct_control.publish(Twist())

    def _start_lost_spin(self):
        if self._lost_spin_active:
            return

        self._lost_spin_active = True
        self._lost_spin_timer.reset()
        self._lost_spin_tick()

    def _stop_lost_spin(self):
        if not self._lost_spin_active:
            return

        self._lost_spin_timer.cancel()
        self._lost_spin_active = False
        self._publish_zero_twist()

    def _clear_active_navigation(self, cancel_active_goal: bool = True):
        self._goal_sequence += 1

        if cancel_active_goal and self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()

        self._current_goal_handle = None
        self._active_goal_pose = None
        self._active_goal_is_predicted = False

    def _clear_local_costmap(self, reason: str, done_callback=None):
        if not self._clear_local_costmap_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                f"Local costmap clear non disponibile ({LOCAL_COSTMAP_CLEAR_SERVICE}): {reason}"
            )
            if done_callback is not None:
                done_callback()
            return

        self.get_logger().info(f"Pulizia local costmap: {reason}")
        future = self._clear_local_costmap_client.call_async(
            ClearEntireCostmap.Request()
        )
        future.add_done_callback(
            lambda result: self._clear_local_costmap_done(
                result,
                reason,
                done_callback,
            )
        )

    def _clear_local_costmap_done(self, future: Future, reason: str, done_callback=None):
        try:
            future.result()
            self.get_logger().info(f"Local costmap pulita: {reason}")
        except Exception as error:
            self.get_logger().warn(
                f"Pulizia local costmap fallita ({reason}): {error}"
            )

        if done_callback is not None:
            done_callback()

    def _enter_idle(
        self,
        reason: str,
        cancel_active_goal: bool = True,
        remember_current_goal: bool = False,
    ):
        self.get_logger().info(reason)
        self._stop_lost_spin()
        if (
            remember_current_goal
            and self._active_goal_pose is not None
        ):
            self._last_goal_pose = self._active_goal_pose

        self._clear_active_navigation(cancel_active_goal=cancel_active_goal)
        self._pending_predicted_goal = None
        self._publish_zero_twist()
        self.set_state(State.IDLE)

    def _enter_lost(self, reason: str, cancel_active_goal: bool = True):
        if self._state == State.LOST:
            return

        self.get_logger().warn(reason)
        self._target_visible = False
        self._pending_predicted_goal = None
        self._last_goal_pose = None
        self._stop_lost_spin()
        self._clear_active_navigation(cancel_active_goal=cancel_active_goal)
        self.set_state(State.LOST)
        self._start_lost_spin()

    def _pose_log(self, pose: Pose) -> str:
        yaw = math.degrees(quaternion_to_yaw(pose.orientation))
        return f"x:{pose.position.x:.2f} y:{pose.position.y:.2f} yaw:{yaw:.1f} deg"

    def _send_navigation_goal(self, target: Pose, predicted: bool):
        self._stop_lost_spin()

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = target

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        nav_goal.behavior_tree = self._behavior_tree

        self._goal_sequence += 1
        sequence = self._goal_sequence
        self._active_goal_pose = target
        self._active_goal_is_predicted = predicted
        self.set_state(State.FOLLOWING_PREDICTED if predicted else State.FOLLOWING_VISIBLE)

        self.get_logger().info(
            "Invio goal al bt_navigator: "
            f"{self._pose_log(target)} predicted={predicted}"
        )

        self._navigate_client.send_goal_async(nav_goal).add_done_callback(
            lambda future: self._navigate_goal_response_callback(
                future,
                sequence,
                predicted,
            )
        )

    def _handle_goal(self, target: Pose, predicted: bool):
        if self._state == State.IDLE:
            if (
                not predicted
                and self._last_goal_pose is not None
                and compare_poses(self._last_goal_pose, target)
            ):
                return

            if self._active_goal_pose is not None and compare_poses(
                self._active_goal_pose,
                target,
            ):
                return

            if not predicted:
                self._last_goal_pose = None

            self._send_navigation_goal(target, predicted=predicted)
            return

        if self._state in (State.FOLLOWING_VISIBLE, State.FOLLOWING_PREDICTED):
            if self._active_goal_pose is not None and compare_poses(
                self._active_goal_pose,
                target,
                target_offset=FOLLOW_COMPARE_DISTANCE_TOLERANCE,
                angle_offset=FOLLOW_COMPARE_ANGLE_TOLERANCE,
            ):
                return

            self._send_navigation_goal(target, predicted=predicted)
            return

        if self._state == State.LOST:
            return

    def _navigate_goal_response_callback(
        self,
        future: Future,
        sequence: int,
        predicted: bool,
    ):
        goal_handle: ClientGoalHandle = future.result()  # type: ignore
        if sequence != self._goal_sequence:
            self.get_logger().info("Risposta di un vecchio NavigateToPose ignorata.")
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal rifiutato dal bt_navigator.")
            self._current_goal_handle = None
            self._handle_navigation_failure(
                "NavigateToPose rifiutato dopo perdita ArUco: entro in LOST."
            )
            return

        self._current_goal_handle = goal_handle
        self._active_goal_is_predicted = predicted
        self.get_logger().info("NavigateToPose goal accettato.")
        goal_handle.get_result_async().add_done_callback(
            lambda result: self._navigate_result_callback(result, sequence)
        )

    def _navigate_result_callback(self, future: Future, sequence: int):
        if sequence != self._goal_sequence:
            self.get_logger().info("Risultato di un vecchio NavigateToPose ignorato.")
            return

        response = future.result()
        status = response.status
        self._current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("NavigateToPose goal succeeded.")
            self._handle_navigation_success()
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("NavigateToPose goal canceled.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("NavigateToPose goal aborted.")
        else:
            self.get_logger().error(f"NavigateToPose terminato con status {status}.")

        self._handle_navigation_failure(
            "NavigateToPose fallito dopo perdita ArUco: entro in LOST."
        )

    def _handle_navigation_success(self):
        reached_goal_pose = self._active_goal_pose
        reached_predicted = self._active_goal_is_predicted
        self._active_goal_pose = None
        self._active_goal_is_predicted = False

        if reached_predicted and not self._target_visible:
            self._enter_lost(
                "Predicted goal raggiunto senza riacquisire ArUco: avvio LOST spin.",
                cancel_active_goal=False,
            )
            return

        if not self._target_visible:
            if self._process_pending_predicted_goal():
                return

            self._enter_lost(
                "Goal raggiunto dopo perdita ArUco senza predicted goal pendente: entro in LOST.",
                cancel_active_goal=False,
            )
            return

        if reached_goal_pose is not None:
            self._last_goal_pose = reached_goal_pose

        self.set_state(State.IDLE)

    def _handle_navigation_failure(self, lost_reason: str):
        self._active_goal_pose = None
        self._active_goal_is_predicted = False

        def after_clear():
            if self._process_pending_predicted_goal():
                return

            if not self._target_visible:
                self._enter_lost(lost_reason, cancel_active_goal=False)
                return

            self.set_state(State.IDLE)

        self._clear_local_costmap(
            "fallimento NavigateToPose nel follow",
            after_clear,
        )

    def _process_pending_predicted_goal(self) -> bool:
        if self._pending_predicted_goal is None:
            return False

        target = self._pending_predicted_goal
        self._pending_predicted_goal = None
        self.get_logger().info(
            "Uso predicted goal pendente: "
            f"{self._pose_log(target)}"
        )
        self._handle_goal(target, predicted=True)
        return True

    def _process_reacquired_visible_goal_after_clear(self):
        target = self._reacquired_visible_goal
        self._reacquired_visible_goal = None
        self._clearing_after_lost_reacquire = False

        if self._state == State.LOST:
            self.set_state(State.IDLE)

        if target is not None:
            self._handle_goal(target, predicted=False)

    def _visible_goal_callback(self, msg: PoseStamped):
        self._target_visible = True
        self._pending_predicted_goal = None

        if self._clearing_after_lost_reacquire:
            self._reacquired_visible_goal = msg.pose
            return

        if self._state == State.LOST:
            self._stop_lost_spin()
            self._reacquired_visible_goal = msg.pose
            self._clearing_after_lost_reacquire = True
            self._clear_local_costmap(
                "ArUco riacquisito dopo LOST",
                self._process_reacquired_visible_goal_after_clear,
            )
            return

        if self._state == State.IDLE:
            self._handle_goal(msg.pose, predicted=False)
            return

        self._handle_goal(msg.pose, predicted=False)

    def _predicted_goal_callback(self, msg: PoseStamped):
        self._target_visible = False
        self._last_goal_pose = None

        if self._state == State.LOST:
            self.get_logger().info("Predicted goal ignorato mentre sono in LOST.")
            return

        self._pending_predicted_goal = msg.pose
        if self._state == State.FOLLOWING_VISIBLE:
            self.get_logger().info(
                "Predicted goal salvato come pending: aspetto la fine del visible goal corrente."
            )
            return

        self._process_pending_predicted_goal()

    def _target_lost_callback(self, _msg: Empty):
        self._target_visible = False

        if self._state == State.FOLLOWING_PREDICTED:
            return

        if self._process_pending_predicted_goal():
            return

        self._enter_lost("Marker perso senza predizione valida: entro in LOST.")

    def _target_hold_callback(self, _msg: Empty):
        self._target_visible = True
        self._pending_predicted_goal = None

        if not self._has_navigation_state():
            return

        self._enter_idle(
            "Target visibile ma gia raggiunto/troppo vicino: cancello goal e resto in IDLE.",
            cancel_active_goal=True,
            remember_current_goal=True,
        )


def main(args=None):
    rclpy.init(args=args)
    predictive_follower_fsm = PredictiveFollowerFSM()
    try:
        rclpy.spin(predictive_follower_fsm)
    finally:
        predictive_follower_fsm.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
