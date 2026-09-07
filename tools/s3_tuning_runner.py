#!/usr/bin/env python3
"""Run the fixed S3 approach/retreat scenario and print comparable metrics.

This runner deliberately keeps the object out of RLP's collision environment.
The apple is only a visual/physical Sim target at this stage; attached-object
and environment-bridge tests belong to S4/S5.

Run this from the Apptainer shell after sourcing the workspace and starting
Isaac Sim with ``SCENE=rlp``.  The RLP node itself is expected to be running in
another terminal.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Transform
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import Empty

from hsrb_rlp_interface_py import geometry
from hsrb_rlp_interface_py.robot_local_planner import RobotLocalPlanner
from tmc_planning_msgs.msg import RobotDisplacements, RobotLocalPlannerStatus


DONE_STATUSES = {2, 3}  # ConstraintsStatus.SATISFIED/PREEMPTED
SCENARIO = (
    ("above", "base_footprint", 0.60, 0.00, 0.40, 0.35),
    ("approach", "odom", 0.60, 0.00, 0.18, 0.20),
    ("retreat", "odom", 0.60, 0.00, 0.40, 0.35),
)
GO_POSE = {
    "arm_flex_joint": 0.0,
    "arm_lift_joint": 0.0,
    "arm_roll_joint": -1.57,
    "wrist_flex_joint": -1.57,
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": 0.0,
}


class Capture:
    """Keep the latest telemetry while retaining status events per goal."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._lock = threading.Lock()
        self.status_events: list[RobotLocalPlannerStatus] = []
        self.planned: RobotTrajectory | None = None
        self.generated: DisplayTrajectory | None = None
        self.displacement: RobotDisplacements | None = None

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
        node.create_subscription(
            DisplayTrajectory,
            "hsrb_robot_local_planner/generated_trajectories",
            self._generated_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            RobotDisplacements,
            "hsrb_robot_local_planner/displacements",
            self._displacement_callback,
            qos_profile_sensor_data,
        )

    def clear(self) -> None:
        with self._lock:
            self.status_events.clear()
            self.planned = None
            self.generated = None
            self.displacement = None

    def _status_callback(self, msg: RobotLocalPlannerStatus) -> None:
        with self._lock:
            self.status_events.append(msg)
            self.status_events = self.status_events[-100:]

    def _planned_callback(self, msg: RobotTrajectory) -> None:
        with self._lock:
            self.planned = msg

    def _generated_callback(self, msg: DisplayTrajectory) -> None:
        with self._lock:
            self.generated = msg

    def _displacement_callback(self, msg: RobotDisplacements) -> None:
        with self._lock:
            self.displacement = msg

    def status_for(self, goal_id: str) -> tuple[int | None, int | None]:
        """Return ``(planner_status, constraint_status)`` for a goal."""
        with self._lock:
            for msg in reversed(self.status_events):
                for status in msg.constraints_statuses:
                    if status.id == goal_id:
                        return msg.planner_status, status.value
        return None, None

    def planner_statuses_for(self, goal_id: str) -> list[int]:
        """Return all planner status values observed while a goal was present."""
        values: list[int] = []
        with self._lock:
            for msg in self.status_events:
                if any(status.id == goal_id for status in msg.constraints_statuses):
                    values.append(msg.planner_status)
        return values

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "planned": self.planned,
                "generated": self.generated,
                "displacement": self.displacement,
            }


def _duration(point: Any) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9


def _transform(node: RobotLocalPlanner, target_frame: str, source_frame: str) -> Transform:
    return node._tf2_buffer.lookup_transform(target_frame, source_frame, Time()).transform


def _position_error(tf: Transform, desired: geometry.Pose) -> float:
    dx = tf.translation.x - desired.pos.x
    dy = tf.translation.y - desired.pos.y
    dz = tf.translation.z - desired.pos.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _orientation_error(tf: Transform, desired: geometry.Pose) -> float:
    dot = (
        tf.rotation.x * desired.ori.x
        + tf.rotation.y * desired.ori.y
        + tf.rotation.z * desired.ori.z
        + tf.rotation.w * desired.ori.w
    )
    return 2.0 * math.acos(min(1.0, abs(dot)))


def _rotate_vector(q: Any, vector: Any) -> tuple[float, float, float]:
    """Rotate a 3D vector by a quaternion without adding a TF dependency."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
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


def _compose_odom_pose(odom_to_ref: Transform, ref_to_hand: geometry.Pose) -> geometry.Pose:
    """Compose ``odom->ref`` and ``ref->hand`` as a pose tuple."""
    rotated = _rotate_vector(odom_to_ref.rotation, ref_to_hand.pos)
    return geometry.Pose(
        geometry.Vector3(
            odom_to_ref.translation.x + rotated[0],
            odom_to_ref.translation.y + rotated[1],
            odom_to_ref.translation.z + rotated[2],
        ),
        _multiply_quaternions(odom_to_ref.rotation, ref_to_hand.ori),
    )


def _wait_for_physical_goal(
    node: RobotLocalPlanner,
    reference_frame: str,
    desired: geometry.Pose,
    timeout_sec: float,
    position_tolerance: float = 0.05,
    orientation_tolerance: float = 0.20,
) -> dict[str, Any]:
    """Wait until the controller/TF state is near the requested hand pose."""
    start = time.monotonic()
    stable_since: float | None = None
    last: dict[str, Any] = {"position_error": None, "orientation_error": None}
    while time.monotonic() - start < timeout_sec:
        try:
            tf = _transform(node, reference_frame, "hand_palm_link")
            position_error = _position_error(tf, desired)
            orientation_error = _orientation_error(tf, desired)
            last = {
                "position_error": position_error,
                "orientation_error": orientation_error,
                "hand": [
                    tf.translation.x,
                    tf.translation.y,
                    tf.translation.z,
                ],
            }
            within = (
                position_error <= position_tolerance
                and orientation_error <= orientation_tolerance
            )
            if within:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    last["physical_converged"] = True
                    last["physical_wait_wall_sec"] = time.monotonic() - start
                    return last
            else:
                stable_since = None
        except Exception:
            stable_since = None
        time.sleep(0.1)

    last["physical_converged"] = False
    last["physical_wait_wall_sec"] = time.monotonic() - start
    return last


def _wait_for_joints(
    node: RobotLocalPlanner,
    joint_names: list[str],
    targets: dict[str, float],
    timeout_sec: float = 15.0,
    tolerance: float = 0.06,
) -> bool:
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        current = node._joint_state_sub.get_joint_state(joint_names)
        if len(current) == len(joint_names) and all(
            value is not None and abs(value - targets[name]) <= tolerance
            for name, value in current.items()
        ):
            return True
        time.sleep(0.1)
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


def _wait_for_goal_status(
    capture: Capture,
    goal_id: str,
    timeout_sec: float,
) -> tuple[int | None, int | None, float]:
    start = time.monotonic()
    while time.monotonic() - start < timeout_sec:
        planner_status, constraint_status = capture.status_for(goal_id)
        if constraint_status in DONE_STATUSES:
            return planner_status, constraint_status, time.monotonic() - start
        time.sleep(0.05)
    planner_status, constraint_status = capture.status_for(goal_id)
    return planner_status, constraint_status, time.monotonic() - start


def _run_step(
    node: RobotLocalPlanner,
    capture: Capture,
    label: str,
    frame: str,
    x: float,
    y: float,
    z: float,
    normalized_velocity: float,
    timeout_sec: float,
) -> dict[str, Any]:
    desired = geometry.pose(x=x, y=y, z=z, ei=math.pi)
    if frame == "odom":
        desired_odom = desired
    else:
        desired_odom = _compose_odom_pose(
            _transform(node, "odom", frame),
            desired,
        )
    capture.clear()
    start = time.monotonic()
    goal_id = node.move_end_effector_pose(
        desired,
        ref_frame_id=frame,
        normalized_velocity=normalized_velocity,
    )
    planner_status, constraint_status, status_wait = _wait_for_goal_status(
        capture, goal_id, timeout_sec
    )
    physical = _wait_for_physical_goal(
        node, "odom", desired_odom, max(1.0, timeout_sec - status_wait)
    )
    telemetry = capture.snapshot()
    planner_statuses = capture.planner_statuses_for(goal_id)
    if 1 in planner_statuses:
        planner_status = 1

    result: dict[str, Any] = {
        "step": label,
        "goal_id": goal_id,
        "reference_frame": frame,
        "target": {"x": x, "y": y, "z": z, "roll": math.pi},
        "normalized_velocity": normalized_velocity,
        "planner_status": planner_status,
        "planner_statuses": planner_statuses,
        "constraint_status": constraint_status,
        "status_wait_wall_sec": status_wait,
        "wall_sec": time.monotonic() - start,
        **physical,
    }
    result["target_odom"] = {
        "x": desired_odom.pos.x,
        "y": desired_odom.pos.y,
        "z": desired_odom.pos.z,
    }

    planned = telemetry["planned"]
    if planned is not None:
        trajectory = planned.joint_trajectory
        result["planned_points"] = len(trajectory.points)
        result["planned_duration_sec"] = (
            _duration(trajectory.points[-1]) if trajectory.points else None
        )
        result["planned_joints"] = list(trajectory.joint_names)
    else:
        result["planned_points"] = None
        result["planned_duration_sec"] = None

    generated = telemetry["generated"]
    result["generated_candidates"] = (
        len(generated.trajectory) if generated is not None else None
    )

    displacement = telemetry["displacement"]
    result["min_displacement"] = (
        displacement.min_displacement if displacement is not None else None
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--reset-settle-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    node = RobotLocalPlanner()
    capture = Capture(node)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(json.dumps({"event": "start", "trials": args.trials, "scenario": SCENARIO}))
    results: list[dict[str, Any]] = []
    try:
        time.sleep(2.0)
        for trial in range(1, args.trials + 1):
            reset_ok = _reset_world(node, args.reset_settle_sec)
            node.has_wait_complete = True
            go_start = time.monotonic()
            go_ok = node.move_to_go()
            go_physical = _wait_for_joints(
                node,
                list(GO_POSE),
                GO_POSE,
            )
            go_wall = time.monotonic() - go_start
            time.sleep(1.0)

            trial_result: dict[str, Any] = {
                "trial": trial,
                "reset_ok": reset_ok,
                "go_ok": bool(go_ok),
                "go_physical": go_physical,
                "go_wall_sec": go_wall,
                "steps": [],
            }
            node.has_wait_complete = False
            for step in SCENARIO:
                trial_result["steps"].append(
                    _run_step(node, capture, *step, timeout_sec=args.timeout_sec)
                )
            trial_result["pass"] = bool(
                reset_ok
                and go_ok
                and go_physical
                and all(
                    step["planner_status"] == RobotLocalPlannerStatus.SUCCESS
                    and step["constraint_status"] in DONE_STATUSES
                    and step["physical_converged"]
                    for step in trial_result["steps"]
                )
            )
            results.append(trial_result)
            print(json.dumps(trial_result, allow_nan=False))

        passed = sum(1 for result in results if result["pass"])
        print(json.dumps({"event": "summary", "passed": passed, "total": len(results)}))
    finally:
        node.publish_empty_constraints()
        time.sleep(0.5)
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
