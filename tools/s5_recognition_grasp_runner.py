#!/usr/bin/env python3
"""Run the first S5 recognition-to-grasp regression in Isaac Sim.

The runner deliberately keeps the integration boundary explicit:

``yolov8_detection`` -> ``grasp_point_detection`` -> TF -> RLP approach
-> gripper close -> ``attached_object_publisher`` -> RLP retreat -> release.

It is intended for the small ``carrobo-isaac`` ``SCENE=rlp`` scene, not the
competition arena.  The simulator's optional ``GRASP_ATTACH=1`` mode exposes
the physical grasp truth on ``/rlp_validation/grasp_state``; that is used as
an independent check beside the ROS gripper action and attached-object state.

Run this from the Apptainer shell after the common workspace setup and while
Isaac Sim, the HSR manipulation stack, the perception services, and the RLP
node are running.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from typing import Any

import rclpy
import tf2_geometry_msgs  # noqa: F401  PoseStamped TF conversions
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from hsrb_interface import Robot
from hsrb_rlp_interface_py import geometry
from hsrb_rlp_interface_py.robot_local_planner import RobotLocalPlanner
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningSceneWorld,
    RobotState,
    RobotTrajectory,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from grasp_point_detection_interfaces.srv import GraspPointService
from tmc_planning_msgs.msg import ConstraintsStatus, RobotLocalPlannerStatus
from yolov8_detection_interfaces.srv import ObjectDetectionService


ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
HAND_FRAME = "hand_palm_link"
ENVIRONMENT_TOPIC = "collision_environment_server/transformed_environment"
ENVIRONMENT_CONTROL_TOPIC = "collision_environment_server/collision_object"
ATTACH_TOPIC = "/attached_object_publisher/attaching_object_info"
ATTACHED_TOPIC = "/attached_object_publisher/attached_object"
RELEASE_TOPIC = "/attached_object_publisher/releasing_object_name"
SIM_GRASP_TOPIC = "/rlp_validation/grasp_state"

DONE_STATUSES = {ConstraintsStatus.SATISFIED, ConstraintsStatus.PREEMPTED}
FAILURE_STATUSES = {
    RobotLocalPlannerStatus.GENERATION_FAILURE,
    RobotLocalPlannerStatus.EVALUATION_FAILURE,
    RobotLocalPlannerStatus.VALIDATION_FAILURE,
    RobotLocalPlannerStatus.OPTIMIZATION_FAILURE,
}

NEUTRAL_POSE = {
    "arm_lift_joint": 0.0,
    "arm_flex_joint": 0.0,
    "arm_roll_joint": 0.0,
    "wrist_flex_joint": -1.57,
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": 0.0,
}


class Capture:
    """Capture planner, scene, attachment, and simulator diagnostics."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._lock = threading.Lock()
        self.status_events: list[RobotLocalPlannerStatus] = []
        self.attached: RobotState | None = None
        self.environment: PlanningSceneWorld | None = None
        self.sim_state: dict[str, Any] | None = None
        self.sim_history: list[tuple[float, dict[str, Any]]] = []

        node.create_subscription(
            RobotLocalPlannerStatus,
            "hsrb_robot_local_planner/planner_status",
            self._status_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            RobotTrajectory,
            "hsrb_robot_local_planner/planned_trajectory",
            lambda _msg: None,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            RobotState,
            ATTACHED_TOPIC,
            self._attached_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            PlanningSceneWorld,
            ENVIRONMENT_TOPIC,
            self._environment_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            String,
            SIM_GRASP_TOPIC,
            self._sim_state_callback,
            qos_profile_sensor_data,
        )

    def clear_status(self) -> None:
        with self._lock:
            self.status_events.clear()

    def _status_callback(self, msg: RobotLocalPlannerStatus) -> None:
        with self._lock:
            self.status_events.append(msg)
            self.status_events = self.status_events[-300:]

    def _attached_callback(self, msg: RobotState) -> None:
        with self._lock:
            self.attached = msg

    def _environment_callback(self, msg: PlanningSceneWorld) -> None:
        with self._lock:
            self.environment = msg

    def _sim_state_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self.sim_state = payload
            self.sim_history.append((time.monotonic(), payload))
            self.sim_history = self.sim_history[-500:]

    def status_for(self, goal_id: str) -> tuple[int | None, int | None]:
        with self._lock:
            for msg in reversed(self.status_events):
                for status in msg.constraints_statuses:
                    if status.id == goal_id:
                        return msg.planner_status, status.value
        return None, None

    def planner_statuses_for(self, goal_id: str) -> list[int]:
        with self._lock:
            return [
                msg.planner_status
                for msg in self.status_events
                if any(status.id == goal_id for status in msg.constraints_statuses)
            ]

    def attached_ids(self) -> list[str]:
        with self._lock:
            if self.attached is None:
                return []
            return [
                item.object.id
                for item in self.attached.attached_collision_objects
            ]

    def environment_ids(self) -> list[str]:
        with self._lock:
            if self.environment is None:
                return []
            return [item.id for item in self.environment.collision_objects]

    def latest_sim_state(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self.sim_state) if self.sim_state is not None else None

    def sim_history_copy(self) -> list[tuple[float, dict[str, Any]]]:
        with self._lock:
            return [(stamp, dict(state)) for stamp, state in self.sim_history]


class FramePublisher:
    """Continuously publish the detected object and grasp target TF frames."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._node = node
        self._broadcaster = TransformBroadcaster(node)
        self._lock = threading.Lock()
        self._poses: dict[str, geometry.Pose] = {}
        self._timer = node.create_timer(0.1, self._publish)

    def set_pose(self, child_frame: str, pose_odom: geometry.Pose) -> None:
        with self._lock:
            self._poses[child_frame] = pose_odom

    def _publish(self) -> None:
        with self._lock:
            poses = list(self._poses.items())
        if not poses:
            return
        stamp = self._node.get_clock().now().to_msg()
        messages = []
        for child_frame, target in poses:
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = ODOM_FRAME
            msg.child_frame_id = child_frame
            msg.transform.translation.x = target.pos.x
            msg.transform.translation.y = target.pos.y
            msg.transform.translation.z = target.pos.z
            msg.transform.rotation.x = target.ori.x
            msg.transform.rotation.y = target.ori.y
            msg.transform.rotation.z = target.ori.z
            msg.transform.rotation.w = target.ori.w
            messages.append(msg)
        self._broadcaster.sendTransform(messages)


def _rotate_vector(quaternion: Any, vector: Any) -> tuple[float, float, float]:
    qx, qy, qz, qw = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    vx, vy, vz = vector.x, vector.y, vector.z
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _multiply_quaternions(lhs: Any, rhs: Any) -> geometry.Quaternion:
    return geometry.Quaternion(
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
    )


def _compose_odom_pose(
    odom_to_ref: Any,
    ref_to_hand: geometry.Pose,
) -> geometry.Pose:
    rotated = _rotate_vector(odom_to_ref.rotation, ref_to_hand.pos)
    return geometry.Pose(
        geometry.Vector3(
            odom_to_ref.translation.x + rotated[0],
            odom_to_ref.translation.y + rotated[1],
            odom_to_ref.translation.z + rotated[2],
        ),
        _multiply_quaternions(odom_to_ref.rotation, ref_to_hand.ori),
    )


def _lookup_transform(
    node: RobotLocalPlanner,
    target_frame: str,
    source_frame: str,
) -> Any:
    return node._tf2_buffer.lookup_transform(
        target_frame,
        source_frame,
        Time(),
    ).transform


def _transform_pose(
    node: RobotLocalPlanner,
    stamped: PoseStamped,
    target_frame: str,
) -> PoseStamped:
    """Transform using the detection timestamp, then retry with latest TF."""
    last_error: Exception | None = None
    for _ in range(3):
        try:
            return node._tf2_buffer.transform(stamped, target_frame)
        except Exception as exc:  # tf2 exception types vary by distro
            last_error = exc
            time.sleep(0.15)

    latest = PoseStamped()
    latest.header.frame_id = stamped.header.frame_id
    latest.header.stamp = Time().to_msg()
    latest.pose = stamped.pose
    try:
        return node._tf2_buffer.transform(latest, target_frame)
    except Exception as exc:
        raise RuntimeError(
            f"TF transform {stamped.header.frame_id} -> {target_frame} failed: "
            f"{last_error or exc}"
        ) from exc


def _wait_future(future: Any, timeout_sec: float) -> Any:
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise TimeoutError(f"service call timed out after {timeout_sec:.1f}s")
    if future.exception() is not None:
        raise RuntimeError(str(future.exception()))
    return future.result()


def _wait_for_goal(
    capture: Capture,
    goal_id: str,
    timeout_sec: float,
) -> tuple[int | None, int | None, list[int], float]:
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        planner_status, constraint_status = capture.status_for(goal_id)
        planner_statuses = capture.planner_statuses_for(goal_id)
        if constraint_status in DONE_STATUSES:
            return planner_status, constraint_status, planner_statuses, time.monotonic() - start
        if any(status in FAILURE_STATUSES for status in planner_statuses):
            return planner_status, constraint_status, planner_statuses, time.monotonic() - start
        time.sleep(0.05)
    planner_status, constraint_status = capture.status_for(goal_id)
    return (
        planner_status,
        constraint_status,
        capture.planner_statuses_for(goal_id),
        time.monotonic() - start,
    )


def _wait_for_joints(
    node: RobotLocalPlanner,
    targets: dict[str, float],
    timeout_sec: float,
    tolerance: float = 0.08,
) -> dict[str, Any]:
    start = time.monotonic()
    last: dict[str, Any] = {"physical_converged": False}
    while time.monotonic() - start < timeout_sec:
        current = node._joint_state_sub.get_joint_state(list(targets))
        errors = {
            name: (
                abs(current[name] - target)
                if name in current and current[name] is not None
                else None
            )
            for name, target in targets.items()
        }
        last = {
            "joint_errors": errors,
            "physical_converged": bool(
                len(errors) == len(targets)
                and all(error is not None and error <= tolerance for error in errors.values())
            ),
        }
        if last["physical_converged"]:
            last["physical_wait_wall_sec"] = time.monotonic() - start
            return last
        time.sleep(0.1)
    last["physical_wait_wall_sec"] = time.monotonic() - start
    return last


def _wait_for_hand(
    node: RobotLocalPlanner,
    target_odom: geometry.Pose,
    timeout_sec: float,
    tolerance: float = 0.07,
) -> dict[str, Any]:
    start = time.monotonic()
    stable_since: float | None = None
    last: dict[str, Any] = {"physical_converged": False}
    while time.monotonic() - start < timeout_sec:
        try:
            tf = _lookup_transform(node, ODOM_FRAME, HAND_FRAME)
            error = math.sqrt(
                (tf.translation.x - target_odom.pos.x) ** 2
                + (tf.translation.y - target_odom.pos.y) ** 2
                + (tf.translation.z - target_odom.pos.z) ** 2
            )
            last = {
                "position_error": error,
                "hand_position_odom": [
                    tf.translation.x,
                    tf.translation.y,
                    tf.translation.z,
                ],
                "physical_converged": False,
            }
            if error <= tolerance:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.4:
                    last["physical_converged"] = True
                    last["physical_wait_wall_sec"] = time.monotonic() - start
                    return last
            else:
                stable_since = None
        except Exception:
            stable_since = None
        time.sleep(0.1)
    last["physical_wait_wall_sec"] = time.monotonic() - start
    return last


def _move_joints(
    node: RobotLocalPlanner,
    capture: Capture,
    label: str,
    targets: dict[str, float],
    timeout_sec: float,
) -> dict[str, Any]:
    capture.clear_status()
    start = time.monotonic()
    goal_id = node.move_to_joint_positions(targets, normalized_velocity=0.5)
    planner_status, constraint_status, planner_statuses, wait_sec = _wait_for_goal(
        capture, goal_id, timeout_sec
    )
    physical = _wait_for_joints(node, targets, max(1.0, timeout_sec - wait_sec))
    # After a goal has reached SATISFIED, the planner may publish the
    # idle/CONSTRAINTS_EMPTY status (-1) in the newest message while retaining
    # SUCCESS in the goal's status history.  S3/S4 runners use the same
    # history-based normalization.
    if RobotLocalPlannerStatus.SUCCESS in planner_statuses:
        planner_status = RobotLocalPlannerStatus.SUCCESS
    return {
        "step": label,
        "goal_id": goal_id,
        "targets": targets,
        "planner_status": planner_status,
        "planner_statuses": planner_statuses,
        "constraint_status": constraint_status,
        "status_wait_wall_sec": wait_sec,
        "wall_sec": time.monotonic() - start,
        **physical,
    }


def _move_pose(
    node: RobotLocalPlanner,
    capture: Capture,
    label: str,
    target: geometry.Pose,
    timeout_sec: float,
    reference_frame: str = ODOM_FRAME,
) -> dict[str, Any]:
    if reference_frame == ODOM_FRAME:
        target_odom = target
    else:
        target_odom = _compose_odom_pose(
            _lookup_transform(node, ODOM_FRAME, reference_frame),
            target,
        )
    capture.clear_status()
    start = time.monotonic()
    goal_id = node.move_end_effector_pose(
        target,
        ref_frame_id=reference_frame,
        normalized_velocity=0.5,
    )
    planner_status, constraint_status, planner_statuses, wait_sec = _wait_for_goal(
        capture, goal_id, timeout_sec
    )
    physical = _wait_for_hand(
        node,
        target_odom,
        max(1.0, timeout_sec - wait_sec),
    )
    if RobotLocalPlannerStatus.SUCCESS in planner_statuses:
        planner_status = RobotLocalPlannerStatus.SUCCESS
    return {
        "step": label,
        "goal_id": goal_id,
        "target_frame": reference_frame,
        "target": {
            "x": target.pos.x,
            "y": target.pos.y,
            "z": target.pos.z,
            "roll": math.pi,
        },
        "target_odom": {
            "x": target_odom.pos.x,
            "y": target_odom.pos.y,
            "z": target_odom.pos.z,
        },
        "planner_status": planner_status,
        "planner_statuses": planner_statuses,
        "constraint_status": constraint_status,
        "status_wait_wall_sec": wait_sec,
        "wall_sec": time.monotonic() - start,
        **physical,
    }


def _move_passed(result: dict[str, Any]) -> bool:
    return bool(
        result.get("planner_status") == RobotLocalPlannerStatus.SUCCESS
        and result.get("constraint_status") in DONE_STATUSES
        and result.get("physical_converged", False)
    )


def _reset_world(node: RobotLocalPlanner, settle_sec: float) -> bool:
    client = node.create_client(Empty, "/isaac/reset_world")
    if not client.wait_for_service(timeout_sec=10.0):
        return False
    future = client.call_async(Empty.Request())
    try:
        _wait_future(future, 30.0)
    except Exception:
        return False
    time.sleep(settle_sec)
    return True


def _wait_for_topic_subscriber(publisher: Any, timeout_sec: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        time.sleep(0.1)
    return False


def _wait_for_environment(
    capture: Capture,
    object_id: str,
    present: bool,
    timeout_sec: float = 8.0,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        found = object_id in capture.environment_ids()
        if found == present:
            return True
        time.sleep(0.1)
    return False


def _publish_environment(
    publisher: Any,
    capture: Capture,
    object_msg: CollisionObject,
    present: bool,
    repeat: int = 10,
) -> bool:
    if not _wait_for_topic_subscriber(publisher):
        return False
    if present:
        message = object_msg
    else:
        message = CollisionObject()
        message.id = object_msg.id
        message.operation = CollisionObject.REMOVE
    for _ in range(repeat):
        publisher.publish(message)
        time.sleep(0.1)
    return _wait_for_environment(capture, object_msg.id, present)


def _wait_for_attached(
    capture: Capture,
    object_id: str,
    present: bool,
    timeout_sec: float = 8.0,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        found = object_id in capture.attached_ids()
        if found == present:
            return True
        time.sleep(0.1)
    return False


def _wait_for_sim_grasp(
    capture: Capture,
    attached: bool,
    timeout_sec: float = 10.0,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        state = capture.latest_sim_state()
        if state is not None and bool(state.get("attached", False)) == attached:
            return True
        time.sleep(0.1)
    return False


def _sim_carried_motion(capture: Capture, attached: bool = True) -> float | None:
    positions: list[tuple[float, float, float]] = []
    for _stamp, state in capture.sim_history_copy():
        if bool(state.get("attached", False)) != attached:
            continue
        path = state.get("object_path", "")
        if not path:
            continue
        for obj in state.get("objects", []):
            if obj.get("path") != path:
                continue
            position = obj.get("position")
            if isinstance(position, list) and len(position) == 3:
                positions.append(tuple(float(value) for value in position))
    if len(positions) < 2:
        return None
    first, last = positions[0], positions[-1]
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, last)))


def _collision_object(
    object_id: str,
    pose_odom: PoseStamped,
    size: Any,
) -> CollisionObject:
    msg = CollisionObject()
    msg.header.frame_id = ODOM_FRAME
    msg.id = object_id
    msg.pose = pose_odom.pose
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [
        max(0.01, float(size.x)),
        max(0.01, float(size.y)),
        max(0.01, float(size.z)),
    ]
    primitive_pose = Pose()
    primitive_pose.orientation.w = 1.0
    msg.primitives = [primitive]
    msg.primitive_poses = [primitive_pose]
    msg.operation = CollisionObject.ADD
    return msg


def _attached_object(object_msg: CollisionObject) -> AttachedCollisionObject:
    msg = AttachedCollisionObject()
    msg.link_name = HAND_FRAME
    msg.touch_links = [
        HAND_FRAME,
        "hand_l_distal_link",
        "hand_r_distal_link",
    ]
    msg.object = object_msg
    msg.object.operation = CollisionObject.ADD
    return msg


def _publish_release(publisher: Any, object_id: str, repeat: int = 8) -> None:
    msg = String()
    msg.data = object_id
    for _ in range(repeat):
        publisher.publish(msg)
        time.sleep(0.1)


def _gripper_command(gripper: Any, *args: Any) -> tuple[bool, str]:
    try:
        result = gripper.command(*args)
        return True, str(result)
    except Exception as exc:  # HSR interface action errors are runtime-specific
        return False, str(exc)


def _print_event(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, ensure_ascii=False, allow_nan=False), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="apple")
    parser.add_argument("--object-id", default="s5_detected_object")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--max-distance", type=float, default=2.0)
    parser.add_argument(
        "--observation-pan",
        type=float,
        default=-0.65,
        help="head pan used to keep the front apple clear of the hand in SCENE=rlp",
    )
    parser.add_argument("--pregrasp-offset", type=float, default=0.10)
    parser.add_argument("--approach-distance", type=float, default=0.05)
    parser.add_argument("--timeout-sec", type=float, default=55.0)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = RobotLocalPlanner()
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    capture = Capture(node)
    frames = FramePublisher(node)
    environment_pub = node.create_publisher(CollisionObject, ENVIRONMENT_CONTROL_TOPIC, 10)
    attach_pub = node.create_publisher(AttachedCollisionObject, ATTACH_TOPIC, 10)
    release_pub = node.create_publisher(String, RELEASE_TOPIC, 10)
    detect_client = node.create_client(ObjectDetectionService, "/yolov8_detection/service")
    grasp_client = node.create_client(GraspPointService, "/grasp_point_detection/service")
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot = None
    gripper = None
    whole_body = None
    attached_in_planner = False
    object_msg: CollisionObject | None = None
    result: dict[str, Any] = {
        "stage": "S5",
        "target": args.target,
        "object_id": args.object_id,
        "pass": False,
        "steps": {},
    }

    _print_event({
        "event": "start",
        "stage": "S5",
        "target": args.target,
        "object_id": args.object_id,
        "detection_service": "/yolov8_detection/service",
        "grasp_service": "/grasp_point_detection/service",
        "tf_frames": ["s5_detected_object", "s5_grasp_target"],
        "attach_topic": ATTACH_TOPIC,
        "attached_topic": ATTACHED_TOPIC,
        "sim_grasp_topic": SIM_GRASP_TOPIC,
    })

    try:
        if not detect_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("物体検出サービスが見つかりません")
        if not grasp_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("把持点推定サービスが見つかりません")

        robot = Robot()
        gripper = robot.get("gripper")
        whole_body = robot.get("whole_body")

        result["reset_ok"] = _reset_world(node, args.reset_settle_sec)
        if not result["reset_ok"]:
            raise RuntimeError("Isaac Sim reset_world に失敗しました")

        open_ok, open_message = _gripper_command(gripper, 1.0)
        result["gripper_open_before"] = {"ok": open_ok, "result": open_message}
        # The head/arm setup is a perception-camera operation.  Use the HSR
        # whole-body interface here so the camera is actually in the requested
        # view before asking the detector for a frame.  The grasp approach,
        # contact pose, and attached-object retreat below are all sent through
        # RLP and are the S5 motion under test.
        whole_body.move_to_neutral(sync=True)
        result["steps"]["neutral"] = {
            "controller": "hsrb_interface",
            "targets": NEUTRAL_POSE,
            **_wait_for_joints(node, NEUTRAL_POSE, 5.0),
        }
        if not result["steps"]["neutral"].get("physical_converged", False):
            raise RuntimeError("neutral姿勢への移動に失敗しました")

        observation_pose = dict(NEUTRAL_POSE)
        observation_pose["head_pan_joint"] = args.observation_pan
        observation_pose["head_tilt_joint"] = math.radians(-50.0)
        whole_body.move_to_joint_positions(observation_pose, sync=True)
        result["steps"]["observation_pose"] = {
            "controller": "hsrb_interface",
            "targets": observation_pose,
            **_wait_for_joints(node, observation_pose, 5.0),
        }
        if not result["steps"]["observation_pose"].get("physical_converged", False):
            raise RuntimeError("観察姿勢への移動に失敗しました")
        time.sleep(1.5)

        detection_attempts: list[list[str]] = []
        detections = None
        candidates = []
        for _attempt in range(1, 6):
            detect_request = ObjectDetectionService.Request()
            detect_request.confidence_th = args.confidence
            detection_response = _wait_future(
                detect_client.call_async(detect_request),
                20.0,
            )
            detections = detection_response.detections
            labels = [bbox.name for bbox in detections.bbox]
            detection_attempts.append(labels)
            candidates = [
                (index, bbox)
                for index, bbox in enumerate(detections.bbox)
                if bbox.name == args.target
            ]
            if not candidates:
                candidates = [
                    (index, bbox)
                    for index, bbox in enumerate(detections.bbox)
                    if args.target.lower() in bbox.name.lower()
                ]
            if candidates:
                break
            time.sleep(0.4)
        if detections is None:
            raise RuntimeError("物体検出サービスが応答しませんでした")
        if not candidates:
            raise RuntimeError(
                f"対象物 {args.target!r} が検出されませんでした: "
                f"{detection_attempts}"
            )
        detection_index, bbox = max(candidates, key=lambda item: item[1].score)
        if detection_index >= len(detections.segments):
            raise RuntimeError("検出bboxと分割maskの個数が一致しません")
        _print_event({
            "event": "detection",
            "target": bbox.name,
            "score": float(bbox.score),
            "bbox": {
                "center": [bbox.x, bbox.y],
                "size": [bbox.w, bbox.h],
            },
            "frame_id": detections.camera_info.header.frame_id,
            "all_labels": [item.name for item in detections.bbox],
            "attempt_labels": detection_attempts,
        })
        result["detection"] = {
            "name": bbox.name,
            "score": float(bbox.score),
            "frame_id": detections.camera_info.header.frame_id,
            "bbox": [bbox.x, bbox.y, bbox.w, bbox.h],
            "attempt_labels": detection_attempts,
        }

        grasp_request = GraspPointService.Request()
        grasp_request.depth = detections.depth
        grasp_request.mask = detections.segments[detection_index]
        grasp_request.camera_info = detections.camera_info
        grasp_request.max_distance = args.max_distance
        grasp_response = _wait_future(
            grasp_client.call_async(grasp_request),
            20.0,
        )
        if not grasp_response.success:
            raise RuntimeError(f"把持点推定に失敗しました: {grasp_response.message}")

        grasp_stamped = PoseStamped()
        grasp_stamped.header = detections.camera_info.header
        grasp_stamped.pose = grasp_response.grasp.pose
        grasp_base = _transform_pose(node, grasp_stamped, BASE_FRAME)
        grasp_odom = _transform_pose(node, grasp_stamped, ODOM_FRAME)

        # The detector returns a horizontal PCA orientation.  For this first
        # flat YCB object, use the same top-down hand orientation as the
        # existing grasp example and keep the detected position unchanged.
        target_base = geometry.pose(
            x=grasp_base.pose.position.x,
            y=grasp_base.pose.position.y,
            z=grasp_base.pose.position.z + args.pregrasp_offset - args.approach_distance,
            ei=math.pi,
        )
        pregrasp_base = geometry.pose(
            x=grasp_base.pose.position.x,
            y=grasp_base.pose.position.y,
            z=grasp_base.pose.position.z + args.pregrasp_offset,
            ei=math.pi,
        )
        target_odom = _compose_odom_pose(
            _lookup_transform(node, ODOM_FRAME, BASE_FRAME),
            target_base,
        )
        pregrasp_odom = _compose_odom_pose(
            _lookup_transform(node, ODOM_FRAME, BASE_FRAME),
            pregrasp_base,
        )
        detected_odom = geometry.Pose(
            geometry.Vector3(
                grasp_odom.pose.position.x,
                grasp_odom.pose.position.y,
                grasp_odom.pose.position.z,
            ),
            geometry.Quaternion(
                grasp_odom.pose.orientation.x,
                grasp_odom.pose.orientation.y,
                grasp_odom.pose.orientation.z,
                grasp_odom.pose.orientation.w,
            ),
        )
        frames.set_pose("s5_detected_object", detected_odom)
        frames.set_pose("s5_grasp_target", target_odom)
        _print_event({
            "event": "grasp_estimate",
            "camera_frame": detections.camera_info.header.frame_id,
            "base_position": [
                grasp_base.pose.position.x,
                grasp_base.pose.position.y,
                grasp_base.pose.position.z,
            ],
            "odom_position": [
                grasp_odom.pose.position.x,
                grasp_odom.pose.position.y,
                grasp_odom.pose.position.z,
            ],
            "size": [
                grasp_response.grasp.size.x,
                grasp_response.grasp.size.y,
                grasp_response.grasp.size.z,
            ],
            "width": float(grasp_response.grasp.width),
            "quality": float(grasp_response.grasp.quality),
            "tf_parent": ODOM_FRAME,
            "tf_frames": ["s5_detected_object", "s5_grasp_target"],
            "pregrasp_base": [pregrasp_base.pos.x, pregrasp_base.pos.y, pregrasp_base.pos.z],
            "grasp_base": [target_base.pos.x, target_base.pos.y, target_base.pos.z],
        })
        result["grasp_estimate"] = {
            "camera_frame": detections.camera_info.header.frame_id,
            "base_position": [
                grasp_base.pose.position.x,
                grasp_base.pose.position.y,
                grasp_base.pose.position.z,
            ],
            "odom_position": [
                grasp_odom.pose.position.x,
                grasp_odom.pose.position.y,
                grasp_odom.pose.position.z,
            ],
            "size": [
                grasp_response.grasp.size.x,
                grasp_response.grasp.size.y,
                grasp_response.grasp.size.z,
            ],
            "width": float(grasp_response.grasp.width),
            "quality": float(grasp_response.grasp.quality),
        }

        object_msg = _collision_object(args.object_id, grasp_odom, grasp_response.grasp.size)
        result["steps"]["environment_add_before_grasp"] = _publish_environment(
            environment_pub,
            capture,
            object_msg,
            present=True,
        )
        if not result["steps"]["environment_add_before_grasp"]:
            raise RuntimeError("把持対象CollisionObjectを環境へ登録できませんでした")

        result["steps"]["pregrasp"] = _move_pose(
            node, capture, "pregrasp", pregrasp_odom, args.timeout_sec
        )
        if not _move_passed(result["steps"]["pregrasp"]):
            raise RuntimeError("pregraspへのRLP移動に失敗しました")

        # Do not make the contact pose collide with its own free-space object.
        # The same geometry is reintroduced as an AttachedCollisionObject once
        # the physical gripper has closed.
        result["steps"]["environment_remove_before_contact"] = _publish_environment(
            environment_pub,
            capture,
            object_msg,
            present=False,
        )
        if not result["steps"]["environment_remove_before_contact"]:
            raise RuntimeError("把持前のCollisionObject解除を確認できませんでした")

        result["steps"]["grasp_pose"] = _move_pose(
            node, capture, "grasp_pose", target_odom, args.timeout_sec
        )
        if not _move_passed(result["steps"]["grasp_pose"]):
            raise RuntimeError("把持姿勢へのRLP移動に失敗しました")

        close_ok, close_message = _gripper_command(gripper, -0.1, 1.0)
        result["gripper_close"] = {"ok": close_ok, "result": close_message}
        result["steps"]["physical_sim_attach"] = _wait_for_sim_grasp(
            capture, attached=True, timeout_sec=12.0
        )
        if not result["steps"]["physical_sim_attach"]:
            raise RuntimeError(
                "グリッパ動作後にSimの物理把持状態(attached=true)を確認できませんでした"
            )

        attached_msg = _attached_object(object_msg)
        if not _wait_for_topic_subscriber(attach_pub):
            raise RuntimeError("attached_object_publisherのattach subscriberがありません")
        for _ in range(10):
            attach_pub.publish(attached_msg)
            time.sleep(0.1)
        result["steps"]["attached_object_add"] = _wait_for_attached(
            capture, args.object_id, present=True
        )
        attached_in_planner = result["steps"]["attached_object_add"]
        if not attached_in_planner:
            raise RuntimeError("attached_object_publisherへの登録を確認できませんでした")

        retreat_odom = geometry.Pose(
            geometry.Vector3(
                target_odom.pos.x,
                target_odom.pos.y,
                target_odom.pos.z + 0.25,
            ),
            target_odom.ori,
        )
        result["steps"]["retreat_with_attached_object"] = _move_pose(
            node, capture, "retreat_with_attached_object", retreat_odom, args.timeout_sec
        )
        if not _move_passed(result["steps"]["retreat_with_attached_object"]):
            raise RuntimeError("Attached Objectを保持した退避動作に失敗しました")
        time.sleep(1.0)
        result["physical_carried_motion_m"] = _sim_carried_motion(capture, attached=True)

        _publish_release(release_pub, args.object_id)
        result["steps"]["attached_object_remove"] = _wait_for_attached(
            capture, args.object_id, present=False
        )
        result["steps"]["environment_restore_after_release"] = _wait_for_environment(
            capture, args.object_id, present=True
        )
        open_ok_after, open_message_after = _gripper_command(gripper, 1.0)
        result["gripper_open_after"] = {
            "ok": open_ok_after,
            "result": open_message_after,
        }
        result["steps"]["physical_sim_release"] = _wait_for_sim_grasp(
            capture, attached=False, timeout_sec=12.0
        )

        result["pass"] = bool(
            result["reset_ok"]
            and result["steps"]["environment_add_before_grasp"]
            and result["steps"]["environment_remove_before_contact"]
            and _move_passed(result["steps"]["pregrasp"])
            and _move_passed(result["steps"]["grasp_pose"])
            and result["steps"]["physical_sim_attach"]
            and result["steps"]["attached_object_add"]
            and _move_passed(result["steps"]["retreat_with_attached_object"])
            and result["steps"]["attached_object_remove"]
            and result["steps"]["environment_restore_after_release"]
            and result["steps"]["physical_sim_release"]
        )
        result["reason"] = "recognition_tf_grasp_attach_retreat_release"
    except Exception as exc:  # keep JSONL diagnostics and clean up in finally
        result["error"] = str(exc)
        _print_event({"event": "error", "stage": "S5", "error": str(exc)})
    finally:
        if attached_in_planner:
            try:
                _publish_release(release_pub, args.object_id, repeat=4)
            except Exception:
                pass
        if gripper is not None:
            _gripper_command(gripper, 1.0)
        try:
            node.publish_empty_constraints()
            time.sleep(0.5)
        except Exception:
            pass
        _print_event({"event": "summary", **result})
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
