#!/usr/bin/env python3
"""Run the first S4 static-environment obstacle regression scenarios.

The validator does not consume a PointCloud or an Octomap.  It consumes a
cached ``moveit_msgs/PlanningSceneWorld`` on the fixed output topic
``collision_environment_server/transformed_environment``.  The running
``collision_environment_server`` owns that output, so this runner registers
boxes through its ``collision_object`` input topic instead of publishing to
the transformed output directly.  This runner is a small, deterministic
scene bridge for that contract and a base-motion test driver around it.

The boxes are expressed in the planner's ``odom`` frame.  The runner registers
them with the RLP and publishes the same snapshot to the Isaac Sim physical
obstacle bridge.  The physical bridge receives each box relative to the robot
pose at the start of the case, so the RLP and Sim coordinate conventions stay
aligned.  The current runner deliberately keeps the scene geometry in one
place so that a mismatch between the RLP collision scene and the Sim scene is
easy to spot in the JSONL output.

Run this from the Apptainer shell after sourcing the workspace and starting
Isaac Sim with ``SCENE=rlp_empty``.  The RLP node itself is expected to be
running in another terminal.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Iterable

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, RobotTrajectory
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String, UInt32

from hsrb_rlp_interface_py.robot_local_planner import RobotLocalPlanner
from tmc_planning_msgs.msg import ConstraintsStatus, RobotLocalPlannerStatus


ENVIRONMENT_TOPIC = "collision_environment_server/transformed_environment"
ENVIRONMENT_CONTROL_TOPIC = "collision_environment_server/collision_object"
PHYSICAL_OBSTACLE_TOPIC = "/rlp_validation/physical_obstacles"
PHYSICAL_OBSTACLE_ACK_TOPIC = "/rlp_validation/physical_obstacles_applied"
PHYSICAL_OBSTACLE_CONTACT_TOPIC = "/rlp_validation/physical_obstacle_contact"
BASE_FRAME = "base_footprint"
ODOM_FRAME = "odom"
DONE_STATUSES = {ConstraintsStatus.SATISFIED, ConstraintsStatus.PREEMPTED}
FAILURE_STATUSES = {
    RobotLocalPlannerStatus.GENERATION_FAILURE,
    RobotLocalPlannerStatus.EVALUATION_FAILURE,
    RobotLocalPlannerStatus.VALIDATION_FAILURE,
    RobotLocalPlannerStatus.OPTIMIZATION_FAILURE,
}

# The radius is only used for a human-readable path-clearance diagnostic.  The
# actual pass/fail collision check remains the RLP validator's robot model.
# Match the HSR-B Sim base collision radius used by carrobo-isaac.  This is
# only a path-reporting approximation; the validator remains authoritative.
DIAGNOSTIC_ROBOT_RADIUS = 0.24


@dataclass(frozen=True)
class Box:
    """An axis-aligned box in odom, with dimensions in x/y/z."""

    name: str
    center: tuple[float, float]
    dimensions: tuple[float, float, float]
    yaw: float = 0.0


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    # Target and obstacle positions are specified in the robot-local frame at
    # the start of the scenario.  _run_scenario resolves them into odom after
    # the robot has reached the go posture.
    target: tuple[float, float, float]
    obstacles: tuple[Box, ...]
    expect_success: bool
    preferred_side: int = 0  # -1: negative y, +1: positive y, 0: either


# Keep the first S4 scene small and interpretable.  The boxes are intentionally
# low so this first obstacle slice exercises base collision checking without
# making random whole-body middle postures collide with a tall object.  The
# broad blocked wall exceeds the generator's +/-1 m intermediate-base search
# range.
SCENARIOS = {
    "avoidable_box": Scenario(
        name="avoidable_box",
        description="正面の箱を左右どちらかへ回避してx=1.2へ到達",
        target=(1.20, 0.00, 0.00),
        obstacles=(
            Box("s4_avoidable_box", (0.55, 0.00), (0.30, 0.40, 0.30)),
        ),
        expect_success=True,
    ),
    "one_side_pass": Scenario(
        name="one_side_pass",
        description="片側を開けた箱を負のy側へ回避して通過",
        target=(1.20, 0.00, 0.00),
        obstacles=(
            Box("s4_one_side_box", (0.55, 0.25), (0.30, 0.40, 0.30)),
        ),
        expect_success=True,
        preferred_side=-1,
    ),
    "blocked_wall": Scenario(
        name="blocked_wall",
        description="正面を広い箱で塞ぎ、計画失敗を返す",
        target=(1.20, 0.00, 0.00),
        obstacles=(
            Box("s4_blocked_wall", (0.55, 0.00), (0.30, 3.00, 0.30)),
        ),
        expect_success=False,
    ),
}


class Capture:
    """Keep status and the latest planned trajectory for each base goal."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._lock = threading.Lock()
        self.status_events: list[RobotLocalPlannerStatus] = []
        self.planned: RobotTrajectory | None = None
        self.planned_history: list[list[tuple[float, float, float]]] = []

        node.create_subscription(
            RobotLocalPlannerStatus,
            "hsrb_robot_local_planner/planner_status",
            self._status_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            RobotTrajectory,
            "hsrb_robot_local_planner/planned_trajectory",
            self._planned_callback,
            qos_profile_sensor_data,
        )

    def clear(self) -> None:
        with self._lock:
            self.status_events.clear()
            self.planned = None
            self.planned_history.clear()

    def _status_callback(self, msg: RobotLocalPlannerStatus) -> None:
        with self._lock:
            self.status_events.append(msg)
            self.status_events = self.status_events[-200:]

    def _planned_callback(self, msg: RobotTrajectory) -> None:
        with self._lock:
            self.planned = msg
            self.planned_history.append(_planned_base_points(msg))
            self.planned_history = self.planned_history[-100:]

    def status_for(self, goal_id: str) -> tuple[int | None, int | None]:
        with self._lock:
            for msg in reversed(self.status_events):
                for status in msg.constraints_statuses:
                    if status.id == goal_id:
                        return msg.planner_status, status.value
        return None, None

    def planner_statuses_for(self, goal_id: str) -> list[int]:
        values: list[int] = []
        with self._lock:
            for msg in self.status_events:
                if any(status.id == goal_id for status in msg.constraints_statuses):
                    values.append(msg.planner_status)
        return values

    def planned_copy(self) -> RobotTrajectory | None:
        with self._lock:
            return self.planned

    def planned_history_copy(self) -> list[list[tuple[float, float, float]]]:
        with self._lock:
            return [list(path) for path in self.planned_history]


class EnvironmentPublisher:
    """Maintain matching RLP and Isaac Sim obstacle snapshots."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._publisher = node.create_publisher(
            CollisionObject,
            ENVIRONMENT_CONTROL_TOPIC,
            10,
        )
        self._physical_publisher = node.create_publisher(
            String,
            PHYSICAL_OBSTACLE_TOPIC,
            10,
        )
        self._physical_lock = threading.Lock()
        self._physical_sequence = 0
        self._physical_acknowledged = 0
        self._physical_contact = False
        node.create_subscription(
            UInt32,
            PHYSICAL_OBSTACLE_ACK_TOPIC,
            self._physical_ack_callback,
            10,
        )
        node.create_subscription(
            Bool,
            PHYSICAL_OBSTACLE_CONTACT_TOPIC,
            self._physical_contact_callback,
            10,
        )
        self._known_ids: set[str] = set()

    def wait_for_subscriber(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if (
                self._publisher.get_subscription_count() > 0
                and self._physical_publisher.get_subscription_count() > 0
            ):
                return True
            time.sleep(0.1)
        return False

    def _physical_ack_callback(self, msg: UInt32) -> None:
        with self._physical_lock:
            self._physical_acknowledged = max(
                self._physical_acknowledged,
                int(msg.data),
            )

    def _physical_contact_callback(self, msg: Bool) -> None:
        if msg.data:
            with self._physical_lock:
                self._physical_contact = True

    def physical_contact_seen(self) -> bool:
        with self._physical_lock:
            return self._physical_contact

    @staticmethod
    def _to_local_box(
        box: Box,
        reference_pose: tuple[float, float, float],
    ) -> dict[str, Any]:
        reference_x, reference_y, reference_yaw = reference_pose
        dx = box.center[0] - reference_x
        dy = box.center[1] - reference_y
        cos_yaw = math.cos(reference_yaw)
        sin_yaw = math.sin(reference_yaw)
        return {
            "id": box.name,
            "center_local": [
                cos_yaw * dx + sin_yaw * dy,
                -sin_yaw * dx + cos_yaw * dy,
            ],
            "dimensions_xyz": list(box.dimensions),
            "yaw_local": box.yaw - reference_yaw,
        }

    @staticmethod
    def _to_collision_object(box: Box) -> CollisionObject:
        object_msg = CollisionObject()
        object_msg.header.frame_id = ODOM_FRAME
        object_msg.id = box.name
        object_msg.pose.position.x = box.center[0]
        object_msg.pose.position.y = box.center[1]
        object_msg.pose.position.z = box.dimensions[2] * 0.5
        object_msg.pose.orientation.z = math.sin(box.yaw * 0.5)
        object_msg.pose.orientation.w = math.cos(box.yaw * 0.5)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(box.dimensions)

        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0

        object_msg.primitives = [primitive]
        object_msg.primitive_poses = [primitive_pose]
        object_msg.operation = CollisionObject.ADD
        return object_msg

    def publish_boxes(
        self,
        boxes: Iterable[Box],
        reference_pose: tuple[float, float, float] | None = None,
        repeat: int = 8,
        period_sec: float = 0.1,
        ack_timeout_sec: float = 10.0,
    ) -> bool:
        boxes = tuple(boxes)
        if boxes and reference_pose is None:
            raise ValueError("reference_pose is required for physical obstacles")
        next_ids = {box.name for box in boxes}
        stale_ids = self._known_ids - next_ids

        for stale_id in stale_ids:
            message = CollisionObject()
            message.id = stale_id
            message.operation = CollisionObject.REMOVE
            self._publisher.publish(message)
            time.sleep(period_sec)

        messages = []
        for box in boxes:
            message = self._to_collision_object(box)
            message.operation = CollisionObject.ADD
            messages.append(message)

        with self._physical_lock:
            self._physical_sequence += 1
            sequence = self._physical_sequence
            self._physical_contact = False
        physical_payload = {
            "sequence": sequence,
            "obstacles": [
                self._to_local_box(box, reference_pose)
                for box in boxes
            ] if boxes else [],
        }
        physical_message = String()
        physical_message.data = json.dumps(
            physical_payload,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )

        for _ in range(max(1, repeat)):
            for message in messages:
                self._publisher.publish(message)
            self._physical_publisher.publish(physical_message)
            time.sleep(period_sec)

        self._known_ids = next_ids
        deadline = time.monotonic() + ack_timeout_sec
        while time.monotonic() < deadline:
            with self._physical_lock:
                if self._physical_acknowledged >= sequence:
                    return True
            time.sleep(0.05)
        return False


def _reset_world(node: RobotLocalPlanner, settle_sec: float) -> bool:
    client = node.create_client(Empty, "/isaac/reset_world")
    if not client.wait_for_service(timeout_sec=10.0):
        return False
    future = client.call_async(Empty.Request())
    deadline = time.monotonic() + 30.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        return False
    time.sleep(settle_sec)
    return True


def _yaw_from_quaternion(rotation: Any) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def _lookup_odom_base_pose(node: RobotLocalPlanner) -> tuple[float, float, float] | None:
    try:
        transform = node._tf2_buffer.lookup_transform(
            ODOM_FRAME,
            BASE_FRAME,
            Time(),
        ).transform
    except Exception:
        return None
    return (
        transform.translation.x,
        transform.translation.y,
        _yaw_from_quaternion(transform.rotation),
    )


def _local_to_odom(
    reference_pose: tuple[float, float, float],
    local_pose: tuple[float, float, float],
) -> tuple[float, float, float]:
    reference_x, reference_y, reference_yaw = reference_pose
    local_x, local_y, local_yaw = local_pose
    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)
    return (
        reference_x + cos_yaw * local_x - sin_yaw * local_y,
        reference_y + sin_yaw * local_x + cos_yaw * local_y,
        reference_yaw + local_yaw,
    )


def _resolve_scenario(
    scenario: Scenario,
    reference_pose: tuple[float, float, float],
) -> Scenario:
    resolved_boxes = tuple(
        Box(
            box.name,
            _local_to_odom(reference_pose, (box.center[0], box.center[1], 0.0))[:2],
            box.dimensions,
            _local_to_odom(reference_pose, (0.0, 0.0, box.yaw))[2],
        )
        for box in scenario.obstacles
    )
    return Scenario(
        name=scenario.name,
        description=scenario.description,
        target=_local_to_odom(reference_pose, scenario.target),
        obstacles=resolved_boxes,
        expect_success=scenario.expect_success,
        preferred_side=scenario.preferred_side,
    )


def _wait_for_base_pose(
    node: RobotLocalPlanner,
    target: tuple[float, float, float],
    timeout_sec: float,
    position_tolerance: float = 0.06,
    yaw_tolerance: float = 0.10,
) -> dict[str, Any]:
    start = time.monotonic()
    stable_since: float | None = None
    last: dict[str, Any] = {
        "position_error": None,
        "yaw_error": None,
        "physical_converged": False,
    }
    while time.monotonic() - start < timeout_sec:
        pose = _lookup_odom_base_pose(node)
        if pose is not None:
            dx = pose[0] - target[0]
            dy = pose[1] - target[1]
            yaw = pose[2]
            yaw_error = math.atan2(
                math.sin(yaw - target[2]),
                math.cos(yaw - target[2]),
            )
            position_error = math.hypot(dx, dy)
            last = {
                "position_error": position_error,
                "yaw_error": abs(yaw_error),
                "pose": [pose[0], pose[1], yaw],
                "physical_converged": False,
            }
            within = position_error <= position_tolerance and abs(yaw_error) <= yaw_tolerance
            if within:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    last["physical_converged"] = True
                    last["physical_wait_wall_sec"] = time.monotonic() - start
                    return last
            else:
                stable_since = None
        time.sleep(0.1)
    last["physical_wait_wall_sec"] = time.monotonic() - start
    return last


def _wait_for_goal(
    capture: Capture,
    goal_id: str,
    timeout_sec: float,
) -> tuple[int | None, int | None, list[int], float]:
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        planner_status, constraint_status = capture.status_for(goal_id)
        statuses = capture.planner_statuses_for(goal_id)
        if constraint_status in DONE_STATUSES:
            return planner_status, constraint_status, statuses, time.monotonic() - start
        time.sleep(0.05)
    planner_status, constraint_status = capture.status_for(goal_id)
    return (
        planner_status,
        constraint_status,
        capture.planner_statuses_for(goal_id),
        time.monotonic() - start,
    )


def _rotate_to_box_frame(x: float, y: float, box: Box) -> tuple[float, float]:
    dx = x - box.center[0]
    dy = y - box.center[1]
    cos_yaw = math.cos(box.yaw)
    sin_yaw = math.sin(box.yaw)
    return cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy


def _signed_box_clearance(x: float, y: float, box: Box) -> float:
    """Approximate base-center clearance from a box minus a footprint radius."""
    local_x, local_y = _rotate_to_box_frame(x, y, box)
    half_x = box.dimensions[0] * 0.5
    half_y = box.dimensions[1] * 0.5
    outside_x = max(abs(local_x) - half_x, 0.0)
    outside_y = max(abs(local_y) - half_y, 0.0)
    if outside_x > 0.0 or outside_y > 0.0:
        distance = math.hypot(outside_x, outside_y)
    else:
        distance = -min(half_x - abs(local_x), half_y - abs(local_y))
    return distance - DIAGNOSTIC_ROBOT_RADIUS


def _line_points_between(
    start: tuple[float, float, float],
    target: tuple[float, float, float],
    count: int = 101,
) -> list[tuple[float, float]]:
    return [
        (
            start[0] + (target[0] - start[0]) * i / (count - 1),
            start[1] + (target[1] - start[1]) * i / (count - 1),
        )
        for i in range(count)
    ]


def _planned_base_points(trajectory: RobotTrajectory | None) -> list[tuple[float, float, float]]:
    if trajectory is None:
        return []
    base = trajectory.multi_dof_joint_trajectory
    if "world_joint" not in base.joint_names:
        return []
    base_index = base.joint_names.index("world_joint")
    points: list[tuple[float, float, float]] = []
    for point in base.points:
        if len(point.transforms) <= base_index:
            continue
        transform = point.transforms[base_index]
        yaw = _yaw_from_quaternion(transform.rotation)
        points.append((transform.translation.x, transform.translation.y, yaw))
    return points


def _path_diagnostics(
    scenario: Scenario,
    trajectory: RobotTrajectory | None,
    start_pose: tuple[float, float, float],
    planned_history: list[list[tuple[float, float, float]]] | None = None,
) -> dict[str, Any]:
    direct_points = _line_points_between(start_pose, scenario.target)
    direct_clearance = min(
        (_signed_box_clearance(x, y, box)
         for x, y in direct_points
         for box in scenario.obstacles),
        default=None,
    )
    planned_points = _planned_base_points(trajectory)
    history_points = [
        point
        for path in (planned_history or [])
        for point in path
    ]
    if history_points:
        planned_points_for_diagnostic = history_points
    else:
        planned_points_for_diagnostic = planned_points
    planned_clearance = min(
        (_signed_box_clearance(x, y, box)
         for x, y, _ in planned_points_for_diagnostic
         for box in scenario.obstacles),
        default=None,
    )
    reference_x, reference_y, reference_yaw = start_pose
    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)
    local_planned_points = []
    for x, y, yaw in planned_points_for_diagnostic:
        dx = x - reference_x
        dy = y - reference_y
        local_planned_points.append(
            (
                cos_yaw * dx + sin_yaw * dy,
                -sin_yaw * dx + cos_yaw * dy,
                yaw - reference_yaw,
            )
        )
    max_abs_y = max((abs(y) for _, y, _ in local_planned_points), default=None)
    preferred_side_observed = None
    if scenario.preferred_side:
        side_points = [y for _, y, _ in local_planned_points if abs(y) > 0.10]
        if side_points:
            preferred_side_observed = (
                -1 if sum(1 for y in side_points if y < 0.0)
                >= sum(1 for y in side_points if y > 0.0)
                else 1
            )
    return {
        "planned_base_points": len(planned_points),
        "planned_trajectory_count": len(planned_history or []),
        "direct_min_clearance_m": direct_clearance,
        "planned_min_clearance_m": planned_clearance,
        "planned_max_abs_y_m": max_abs_y,
        "preferred_side": scenario.preferred_side or None,
        "preferred_side_observed": preferred_side_observed,
    }


def _scenario_pass(
    scenario: Scenario,
    planner_status: int | None,
    constraint_status: int | None,
    planner_statuses: list[int],
    physical: dict[str, Any],
    path: dict[str, Any],
) -> tuple[bool, str]:
    planner_success = RobotLocalPlannerStatus.SUCCESS in planner_statuses
    constraint_done = constraint_status in DONE_STATUSES
    if scenario.expect_success:
        if not physical.get("physical_environment_applied", False):
            return False, "physical_obstacle_snapshot_not_applied"
        if physical.get("physical_obstacle_contact", False):
            return False, "physical_obstacle_contact_detected"
        if not planner_success:
            return False, "expected_success_but_no_planner_success"
        if not constraint_done:
            return False, "expected_success_but_constraints_not_done"
        if not physical.get("physical_converged", False):
            return False, "expected_success_but_base_not_converged"
        if path.get("planned_min_clearance_m") is None:
            return False, "planned_trajectory_missing"
        if path["planned_min_clearance_m"] < -0.02:
            return False, "planned_path_clearance_is_in_collision"
        if path.get("direct_min_clearance_m") is None or path["direct_min_clearance_m"] >= 0.0:
            return False, "scenario_does_not_block_direct_path"
        if path.get("planned_max_abs_y_m") is None or path["planned_max_abs_y_m"] < 0.20:
            return False, "planner_did_not_take_a_detour"
        if (
            scenario.preferred_side
            and path.get("preferred_side_observed") != scenario.preferred_side
        ):
            return False, "planner_selected_the_closed_side"
        return True, "planned_and_executed_detour"

    if planner_success:
        return False, "blocked_scenario unexpectedly_planned_successfully"
    if not any(status in FAILURE_STATUSES for status in planner_statuses):
        return False, "blocked_scenario_did_not_report_planner_failure"
    return True, "planner_rejected_blocked_scene"


def _run_scenario(
    node: RobotLocalPlanner,
    capture: Capture,
    environment: EnvironmentPublisher,
    scenario: Scenario,
    reset_settle_sec: float,
    timeout_sec: float,
) -> dict[str, Any]:
    reset_ok = _reset_world(node, reset_settle_sec)
    clear_ok = environment.publish_boxes(())

    node.has_wait_complete = True
    go_start = time.monotonic()
    go_ok = bool(node.move_to_go())
    go_wall_sec = time.monotonic() - go_start
    node.has_wait_complete = False
    if not reset_ok or not go_ok or not clear_ok:
        environment.publish_boxes(())
        return {
            "scenario": scenario.name,
            "description": scenario.description,
            "reset_ok": reset_ok,
            "go_ok": go_ok,
            "physical_environment_applied": clear_ok,
            "pass": False,
            "reason": "preparation_failed",
        }

    start_pose = _lookup_odom_base_pose(node)
    if start_pose is None:
        environment.publish_boxes(())
        return {
            "scenario": scenario.name,
            "description": scenario.description,
            "reset_ok": reset_ok,
            "go_ok": go_ok,
            "pass": False,
            "reason": "base_pose_unavailable",
        }

    resolved_scenario = _resolve_scenario(scenario, start_pose)
    physical_apply_ok = environment.publish_boxes(
        resolved_scenario.obstacles,
        reference_pose=start_pose,
    )
    if not physical_apply_ok:
        environment.publish_boxes(())
        return {
            "scenario": scenario.name,
            "description": scenario.description,
            "start_pose_odom": list(start_pose),
            "reset_ok": reset_ok,
            "go_ok": go_ok,
            "physical_environment_applied": False,
            "pass": False,
            "reason": "physical_obstacle_snapshot_not_applied",
        }
    capture.clear()
    target_x, target_y, target_yaw = resolved_scenario.target
    start = time.monotonic()
    goal_id = node.move_base_any_frame(
        target_x,
        target_y,
        target_yaw,
        ODOM_FRAME,
    )
    planner_status, constraint_status, planner_statuses, status_wait = _wait_for_goal(
        capture,
        goal_id,
        timeout_sec,
    )
    physical = {
        "physical_converged": False,
        "physical_wait_wall_sec": 0.0,
        "physical_environment_applied": physical_apply_ok,
        "physical_obstacle_contact": environment.physical_contact_seen(),
    }
    if RobotLocalPlannerStatus.SUCCESS in planner_statuses and constraint_status in DONE_STATUSES:
        physical = _wait_for_base_pose(
            node,
            resolved_scenario.target,
            max(1.0, timeout_sec - status_wait),
        )
        physical["physical_environment_applied"] = physical_apply_ok
        physical["physical_obstacle_contact"] = environment.physical_contact_seen()
    path = _path_diagnostics(
        resolved_scenario,
        capture.planned_copy(),
        start_pose,
        capture.planned_history_copy(),
    )
    passed, reason = _scenario_pass(
        scenario,
        planner_status,
        constraint_status,
        planner_statuses,
        physical,
        path,
    )
    result: dict[str, Any] = {
        "scenario": scenario.name,
        "description": scenario.description,
        "start_pose_odom": list(start_pose),
        "obstacles": [
            {
                "id": box.name,
                "frame": ODOM_FRAME,
                "center_xy": list(box.center),
                "dimensions_xyz": list(box.dimensions),
                "yaw": box.yaw,
            }
            for box in resolved_scenario.obstacles
        ],
        "target": {
            "frame": ODOM_FRAME,
            "x": target_x,
            "y": target_y,
            "yaw": target_yaw,
        },
        "reset_ok": reset_ok,
        "go_ok": go_ok,
        "go_wall_sec": go_wall_sec,
        "goal_id": goal_id,
        "planner_status": planner_status,
        "planner_statuses": planner_statuses,
        "constraint_status": constraint_status,
        "status_wait_wall_sec": status_wait,
        "wall_sec": time.monotonic() - start,
        "expected_success": scenario.expect_success,
        "planner_success": RobotLocalPlannerStatus.SUCCESS in planner_statuses,
        "constraint_done": constraint_status in DONE_STATUSES,
        **physical,
        **path,
        "pass": passed,
        "reason": reason,
    }

    node.publish_empty_constraints()
    time.sleep(0.5)
    environment.publish_boxes(())
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        choices=["all", *SCENARIOS.keys()],
        default="all",
    )
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    args = parser.parse_args()

    scenarios = list(SCENARIOS.values()) if args.scenario == "all" else [SCENARIOS[args.scenario]]
    rclpy.init()
    node = RobotLocalPlanner()
    capture = Capture(node)
    environment = EnvironmentPublisher(node)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(json.dumps({
        "event": "start",
        "stage": "S4",
        "environment_topic": ENVIRONMENT_TOPIC,
        "environment_control_topic": ENVIRONMENT_CONTROL_TOPIC,
        "physical_obstacle_topic": PHYSICAL_OBSTACLE_TOPIC,
        "physical_obstacle_ack_topic": PHYSICAL_OBSTACLE_ACK_TOPIC,
        "scenario": args.scenario,
        "trials": args.trials,
    }, ensure_ascii=False))
    results: list[dict[str, Any]] = []
    try:
        time.sleep(2.0)
        subscriber_ok = environment.wait_for_subscriber()
        if not subscriber_ok:
            print(json.dumps({
                "event": "error",
                "stage": "S4",
                "reason": "no_environment_subscriber",
                "environment_control_topic": ENVIRONMENT_CONTROL_TOPIC,
                "physical_obstacle_topic": PHYSICAL_OBSTACLE_TOPIC,
            }, ensure_ascii=False))
            return 2

        for trial in range(1, args.trials + 1):
            trial_results = []
            for scenario in scenarios:
                result = _run_scenario(
                    node,
                    capture,
                    environment,
                    scenario,
                    args.reset_settle_sec,
                    args.timeout_sec,
                )
                result["trial"] = trial
                trial_results.append(result)
                results.append(result)
                print(json.dumps(result, ensure_ascii=False, allow_nan=False))
            print(json.dumps({
                "event": "trial_summary",
                "stage": "S4",
                "trial": trial,
                "passed": sum(1 for result in trial_results if result["pass"]),
                "total": len(trial_results),
                "pass": all(result["pass"] for result in trial_results),
            }, ensure_ascii=False))

        passed = sum(1 for result in results if result["pass"])
        print(json.dumps({
            "event": "summary",
            "stage": "S4",
            "environment_subscriber": subscriber_ok,
            "passed": passed,
            "total": len(results),
            "pass": passed == len(results),
        }, ensure_ascii=False))
    finally:
        node.publish_empty_constraints()
        environment.publish_boxes(())
        time.sleep(0.5)
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if results and all(result["pass"] for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
