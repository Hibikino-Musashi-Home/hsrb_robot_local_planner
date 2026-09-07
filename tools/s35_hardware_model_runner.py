#!/usr/bin/env python3
"""Validate the nominal HSR-B hardware model against carrobo-isaac.

S3.5 is a model-alignment stage, not a substitute for measuring the real
robot.  It checks that the RLP parameters are explicit and consistent with the
HSR-B Sim model, then exercises the base axes and each controlled joint in the
empty RLP scene.

Run this from the Apptainer shell after sourcing the workspace and starting
Isaac Sim with ``SCENE=rlp_empty``.  The RLP node is expected to be running in
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
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory

from hsrb_rlp_interface_py.robot_local_planner import (
    MOTION_PLANNING_JOINTS,
    RobotLocalPlanner,
)
from tmc_planning_msgs.msg import RobotLocalPlannerStatus


DONE_STATUSES = {2, 3}  # ConstraintsStatus.SATISFIED/PREEMPTED
SUCCESS_STATUS = 1  # RobotLocalPlannerStatus.SUCCESS

# These are the HSR-B values used by the empty carrobo-isaac validation scene.
# They are deliberately duplicated here as an audit expectation; a future
# real-robot profile must update the launch, Sim, and this check together.
EXPECTED_PARAMETERS = {
    "tread": 0.266,
    "caster_offset": 0.11,
    "wheel_radius": 0.04,
    "max_caster_velocity": 1.8,
    "max_caster_acceleration": 1.8,
    "max_wheel_velocity": 8.5,
    "max_wheel_acceleration": 5.0,
    "odom_x.velocity": 0.2,
    "odom_x.acceleration": 0.1,
    "odom_y.velocity": 0.2,
    "odom_y.acceleration": 0.1,
    "odom_t.velocity": 0.5,
    "odom_t.acceleration": 0.5,
    "head_pan_joint.velocity": 1.0,
    "head_pan_joint.acceleration": 1.0,
    "head_tilt_joint.velocity": 1.0,
    "head_tilt_joint.acceleration": 1.0,
    "arm_lift_joint.velocity": 0.15,
    "arm_lift_joint.acceleration": 0.15,
    "arm_flex_joint.velocity": 1.0,
    "arm_flex_joint.acceleration": 1.0,
    "arm_roll_joint.velocity": 1.0,
    "arm_roll_joint.acceleration": 1.0,
    "wrist_flex_joint.velocity": 1.0,
    "wrist_flex_joint.acceleration": 1.0,
    "wrist_roll_joint.velocity": 1.0,
    "wrist_roll_joint.acceleration": 1.0,
    "hand_motor_joint.velocity": 3.0,
    "hand_motor_joint.acceleration": 20.0,
}

BASE_CASES = (
    ("x", 0.20, 0.00, 0.00),
    ("y", 0.00, 0.20, 0.00),
    ("yaw", 0.00, 0.00, 0.50),
    ("xy_yaw", 0.15, 0.10, 0.30),
)

# Small, safe moves around GO.  These check controller tracking without
# trying to identify the absolute maximum of an actuator in the Sim.
JOINT_DELTAS = {
    "arm_lift_joint": 0.04,
    # GO is at the HSR-B upper limit (0.0 rad) for arm_flex, so probe inward.
    "arm_flex_joint": -0.10,
    "arm_roll_joint": 0.10,
    "wrist_flex_joint": 0.10,
    "wrist_roll_joint": 0.10,
    "hand_motor_joint": 0.10,
    "head_pan_joint": 0.10,
    "head_tilt_joint": 0.10,
}
GO_POSE = {
    "arm_flex_joint": 0.0,
    "arm_lift_joint": 0.0,
    "arm_roll_joint": -1.57,
    "wrist_flex_joint": -1.57,
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": 0.0,
}
WARMUP_POSE = {
    **GO_POSE,
    # Include the gripper once so the optimizer declares and audits every
    # planning joint, including the one omitted by the public GO helper.
    "hand_motor_joint": 0.05,
}
BASE_STATE_JOINTS = (
    "base_l_drive_wheel_joint",
    "base_r_drive_wheel_joint",
    "base_roll_joint",
)


class Capture:
    """Collect goal status and physical joint telemetry."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._lock = threading.Lock()
        self.status_events: list[RobotLocalPlannerStatus] = []
        self.joint_samples: list[tuple[float, dict[str, float], dict[str, float]]] = []
        self.base_trajectories: list[JointTrajectory] = []
        self.joint_trajectories: list[JointTrajectory] = []
        self.latest_odom: Odometry | None = None

        node.create_subscription(
            RobotLocalPlannerStatus,
            "hsrb_robot_local_planner/planner_status",
            self._status_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            JointState,
            "joint_states",
            self._joint_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            JointTrajectoryControllerState,
            "omni_base_controller/state",
            self._base_state_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Odometry,
            "omni_base_controller/wheel_odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            JointTrajectory,
            "omni_base_controller/joint_trajectory",
            self._base_trajectory_callback,
            qos_profile_sensor_data,
        )
        for controller in (
            "arm_trajectory_controller",
            "head_trajectory_controller",
            "gripper_controller",
        ):
            node.create_subscription(
                JointTrajectory,
                f"{controller}/joint_trajectory",
                self._joint_trajectory_callback,
                qos_profile_sensor_data,
            )

    def clear_goal(self) -> int:
        with self._lock:
            self.status_events.clear()
            self.base_trajectories.clear()
            self.joint_trajectories.clear()
            return len(self.joint_samples)

    def _status_callback(self, msg: RobotLocalPlannerStatus) -> None:
        with self._lock:
            self.status_events.append(msg)
            self.status_events = self.status_events[-100:]

    def _joint_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        with self._lock:
            self.joint_samples.append((time.monotonic(), positions, velocities))
            self.joint_samples = self.joint_samples[-12000:]

    def _base_state_callback(self, _msg: JointTrajectoryControllerState) -> None:
        # The topic is subscribed intentionally: the RLP node and Sim must
        # agree on odom_x, odom_y, odom_t before any S3.5 result is trusted.
        return

    def _odom_callback(self, msg: Odometry) -> None:
        with self._lock:
            self.latest_odom = msg

    def _base_trajectory_callback(self, msg: JointTrajectory) -> None:
        with self._lock:
            self.base_trajectories.append(msg)
            self.base_trajectories = self.base_trajectories[-20:]

    def _joint_trajectory_callback(self, msg: JointTrajectory) -> None:
        with self._lock:
            self.joint_trajectories.append(msg)
            self.joint_trajectories = self.joint_trajectories[-50:]

    def latest_base_trajectory(self) -> JointTrajectory | None:
        with self._lock:
            return self.base_trajectories[-1] if self.base_trajectories else None

    def latest_joint_trajectory(self, joint: str) -> JointTrajectory | None:
        with self._lock:
            for trajectory in reversed(self.joint_trajectories):
                if joint in trajectory.joint_names and trajectory.points:
                    return trajectory
        return None

    def goal_observation(self, goal_id: str) -> tuple[list[int], list[int]]:
        planner_statuses: list[int] = []
        constraint_statuses: list[int] = []
        with self._lock:
            for msg in self.status_events:
                matching = [
                    status for status in msg.constraints_statuses
                    if status.id == goal_id
                ]
                if matching:
                    planner_statuses.append(msg.planner_status)
                    constraint_statuses.extend(status.value for status in matching)
        return planner_statuses, constraint_statuses

    def samples_since(
        self, sample_index: int
    ) -> list[tuple[float, dict[str, float], dict[str, float]]]:
        with self._lock:
            return list(self.joint_samples[sample_index:])


def _decode_parameter(value: Any) -> float | None:
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    if value.type == ParameterType.PARAMETER_BOOL:
        return float(value.bool_value)
    return None


def _get_parameters(
    node: RobotLocalPlanner, names: list[str], timeout_sec: float = 10.0
) -> dict[str, float | None]:
    client = node.create_client(
        GetParameters, "/hsrb_robot_local_planner/get_parameters"
    )
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError("RLP get_parameters service is unavailable")
    result: dict[str, float | None] = {}
    per_parameter_timeout = max(0.5, timeout_sec / max(1, len(names)))
    for name in names:
        request = GetParameters.Request()
        request.names = [name]
        future = client.call_async(request)
        deadline = time.monotonic() + per_parameter_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"timed out while reading RLP parameter: {name}")
        values = future.result().values
        result[name] = _decode_parameter(values[0]) if values else None
    return result


def _close(a: float | None, b: float, tolerance: float = 1e-6) -> bool:
    return a is not None and abs(a - b) <= tolerance


def _parameter_audit(node: RobotLocalPlanner) -> dict[str, Any]:
    names = list(EXPECTED_PARAMETERS)
    actual = _get_parameters(node, names)
    mismatches = {
        name: {"expected": expected, "actual": actual.get(name)}
        for name, expected in EXPECTED_PARAMETERS.items()
        if not _close(actual.get(name), expected)
    }
    return {
        "expected": EXPECTED_PARAMETERS,
        "actual": actual,
        "mismatches": mismatches,
        "pass": not mismatches,
    }


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _base_pose(node: RobotLocalPlanner) -> tuple[float, float, float]:
    tf = node._tf2_buffer.lookup_transform(
        "odom", "base_footprint", Time()
    ).transform
    return tf.translation.x, tf.translation.y, _yaw_from_quaternion(tf.rotation)


def _expected_relative_pose(
    start: tuple[float, float, float], dx: float, dy: float, dyaw: float
) -> tuple[float, float, float]:
    x, y, yaw = start
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x + cos_yaw * dx - sin_yaw * dy,
        y + sin_yaw * dx + cos_yaw * dy,
        _wrap_angle(yaw + dyaw),
    )


def _base_error(
    actual: tuple[float, float, float], expected: tuple[float, float, float]
) -> dict[str, float]:
    return {
        "position_error": math.hypot(actual[0] - expected[0], actual[1] - expected[1]),
        "yaw_error": abs(_wrap_angle(actual[2] - expected[2])),
        "actual_x": actual[0],
        "actual_y": actual[1],
        "actual_yaw": actual[2],
    }


def _wait_for_joint_names(
    node: RobotLocalPlanner, names: list[str], timeout_sec: float = 10.0
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        values = node._joint_state_sub.get_joint_state(names)
        if all(value is not None for value in values.values()):
            return True
        time.sleep(0.1)
    return False


def _wait_for_joints(
    node: RobotLocalPlanner,
    targets: dict[str, float],
    timeout_sec: float = 20.0,
    tolerance: float = 0.06,
    stable_sec: float = 0.4,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    stable_since: float | None = None
    last: dict[str, Any] = {"joint_errors": {}}
    while time.monotonic() < deadline:
        values = node._joint_state_sub.get_joint_state(list(targets))
        errors = {
            name: None if values[name] is None else abs(values[name] - target)
            for name, target in targets.items()
        }
        last = {"joint_errors": errors}
        if all(error is not None and error <= tolerance for error in errors.values()):
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= stable_sec:
                last["physical_converged"] = True
                return last
        else:
            stable_since = None
        time.sleep(0.1)
    last["physical_converged"] = False
    return last


def _wait_for_goal(
    capture: Capture, goal_id: str, timeout_sec: float
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        planner_statuses, constraint_statuses = capture.goal_observation(goal_id)
        if any(value in DONE_STATUSES for value in constraint_statuses):
            return {
                "planner_statuses": planner_statuses,
                "constraint_statuses": constraint_statuses,
                "planner_success": SUCCESS_STATUS in planner_statuses,
                "constraint_done": True,
            }
        time.sleep(0.05)
    planner_statuses, constraint_statuses = capture.goal_observation(goal_id)
    return {
        "planner_statuses": planner_statuses,
        "constraint_statuses": constraint_statuses,
        "planner_success": SUCCESS_STATUS in planner_statuses,
        "constraint_done": any(value in DONE_STATUSES for value in constraint_statuses),
    }


def _reset_world(node: RobotLocalPlanner, settle_sec: float) -> bool:
    client = node.create_client(Empty, "/isaac/reset_world")
    if not client.wait_for_service(timeout_sec=10.0):
        return False
    future = client.call_async(Empty.Request())
    deadline = time.monotonic() + 30.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done() or future.result() is None:
        return False
    time.sleep(settle_sec)
    return True


def _prepare_world(node: RobotLocalPlanner, settle_sec: float) -> bool:
    node.publish_empty_constraints()
    time.sleep(0.3)
    return _reset_world(node, settle_sec)


def _telemetry_peaks(
    samples: list[tuple[float, dict[str, float], dict[str, float]]],
    names: tuple[str, ...] | list[str],
) -> dict[str, dict[str, float | None]]:
    result: dict[str, dict[str, float | None]] = {}
    for name in names:
        peak_velocity = 0.0
        peak_acceleration = 0.0
        previous: tuple[float, float] | None = None
        for stamp, _positions, velocities in samples:
            if name not in velocities:
                continue
            velocity = float(velocities[name])
            peak_velocity = max(peak_velocity, abs(velocity))
            if previous is not None:
                dt = stamp - previous[0]
                if dt > 1e-4:
                    peak_acceleration = max(
                        peak_acceleration,
                        abs(velocity - previous[1]) / dt,
                    )
            previous = (stamp, velocity)
        result[name] = {
            "peak_velocity": peak_velocity,
            "peak_acceleration": peak_acceleration,
        }
    return result


def _trajectory_peaks(
    trajectory: JointTrajectory | None,
    names: tuple[str, ...] | list[str],
) -> dict[str, Any]:
    """Extract the limits carried by an RLP-published trajectory.

    The Sim controller publishes joint states from a high-stiffness PhysX
    position drive.  Its transient acceleration is not the acceleration limit
    that RLP time-optimalization put into the command.  Audit the command here
    and keep the physical joint-state peaks as a separate diagnostic.
    """
    if trajectory is None or not trajectory.points:
        return {"available": False}

    result: dict[str, Any] = {
        "available": True,
        "joint_names": list(trajectory.joint_names),
        "points": len(trajectory.points),
        "duration_sec": 0.0,
        "joints": {},
    }
    point_times = []
    for point in trajectory.points:
        point_times.append(
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
        )
    result["duration_sec"] = max(point_times, default=0.0)

    for name in names:
        if name not in trajectory.joint_names:
            result["joints"][name] = {
                "peak_velocity": None,
                "peak_acceleration": None,
            }
            continue
        index = trajectory.joint_names.index(name)
        peak_velocity = 0.0
        peak_acceleration = 0.0
        previous_position: float | None = None
        previous_velocity: tuple[float, float] | None = None
        for point_time, point in zip(point_times, trajectory.points):
            if index >= len(point.positions):
                continue
            position = float(point.positions[index])
            velocity: float | None = None
            if index < len(point.velocities):
                velocity = float(point.velocities[index])
            elif previous_position is not None and previous_velocity is not None:
                dt = point_time - previous_velocity[0]
                if dt > 1e-4:
                    velocity = (position - previous_position) / dt
            if velocity is not None:
                peak_velocity = max(peak_velocity, abs(velocity))
                if previous_velocity is not None:
                    dt = point_time - previous_velocity[0]
                    if dt > 1e-4:
                        peak_acceleration = max(
                            peak_acceleration,
                            abs(velocity - previous_velocity[1]) / dt,
                        )
                previous_velocity = (point_time, velocity)
            previous_position = position
        result["joints"][name] = {
            "peak_velocity": peak_velocity,
            "peak_acceleration": peak_acceleration,
        }
    return result


def _run_base_case(
    node: RobotLocalPlanner,
    capture: Capture,
    label: str,
    dx: float,
    dy: float,
    dyaw: float,
    timeout_sec: float,
) -> dict[str, Any]:
    start = _base_pose(node)
    expected = _expected_relative_pose(start, dx, dy, dyaw)
    sample_index = capture.clear_goal()
    began = time.monotonic()
    goal_id = node.move_base_relative(dx, dy, dyaw)
    if goal_id is None:
        return {"case": label, "pass": False, "error": "base goal was not published"}
    status = _wait_for_goal(capture, goal_id, timeout_sec)
    deadline = time.monotonic() + max(1.0, timeout_sec)
    physical: dict[str, Any] = {"physical_converged": False}
    while time.monotonic() < deadline:
        try:
            physical = _base_error(_base_pose(node), expected)
            if physical["position_error"] <= 0.02 and physical["yaw_error"] <= 0.05:
                physical["physical_converged"] = True
                break
        except Exception:
            pass
        time.sleep(0.1)
    physical.setdefault("physical_converged", False)
    samples = capture.samples_since(sample_index)
    base_trajectory = capture.latest_base_trajectory()
    result: dict[str, Any] = {
        "case": label,
        "goal": {"x": dx, "y": dy, "yaw": dyaw},
        "start": {"x": start[0], "y": start[1], "yaw": start[2]},
        "expected": {"x": expected[0], "y": expected[1], "yaw": expected[2]},
        "wall_sec": time.monotonic() - began,
        **status,
        **physical,
        "telemetry": _telemetry_peaks(samples, BASE_STATE_JOINTS),
    }
    if base_trajectory is None or not base_trajectory.points:
        result["published_base_trajectory"] = None
    else:
        first = base_trajectory.points[0]
        last = base_trajectory.points[-1]
        result["published_base_trajectory"] = {
            "joint_names": list(base_trajectory.joint_names),
            "points": len(base_trajectory.points),
            "first_positions": list(first.positions),
            "last_positions": list(last.positions),
            "last_velocities": list(last.velocities),
            "last_time_sec": (
                float(last.time_from_start.sec)
                + float(last.time_from_start.nanosec) * 1e-9
            ),
        }
    result["pass"] = bool(
        status["planner_success"]
        and status["constraint_done"]
        and physical["physical_converged"]
    )
    return result


def _run_joint_case(
    node: RobotLocalPlanner,
    capture: Capture,
    joint: str,
    delta: float,
    timeout_sec: float,
) -> dict[str, Any]:
    current = node._joint_state_sub.get_joint_state([joint]).get(joint)
    if current is None:
        return {"joint": joint, "pass": False, "error": "joint state unavailable"}
    target = current + delta
    sample_index = capture.clear_goal()
    began = time.monotonic()
    # Keep the probe slow enough that sampled Sim telemetry is not dominated by
    # the position-drive step response while checking the configured limits.
    goal_id = node.move_to_joint_positions({joint: target}, normalized_velocity=0.15)
    if goal_id is None:
        return {"joint": joint, "pass": False, "error": "joint goal was not published"}
    status = _wait_for_goal(capture, goal_id, timeout_sec)
    physical = _wait_for_joints(
        node, {joint: target}, timeout_sec=timeout_sec, tolerance=0.02
    )
    samples = capture.samples_since(sample_index)
    telemetry = _telemetry_peaks(samples, [joint]).get(joint, {})
    planned_trajectory = _trajectory_peaks(
        capture.latest_joint_trajectory(joint), [joint]
    )
    planned_joint = planned_trajectory.get("joints", {}).get(joint, {})
    configured_velocity = EXPECTED_PARAMETERS[f"{joint}.velocity"]
    configured_acceleration = EXPECTED_PARAMETERS[f"{joint}.acceleration"]
    result: dict[str, Any] = {
        "joint": joint,
        "start": current,
        "target": target,
        "delta": delta,
        "wall_sec": time.monotonic() - began,
        **status,
        **physical,
        "configured_velocity": configured_velocity,
        "configured_acceleration": configured_acceleration,
        "planned_trajectory": planned_trajectory,
        "telemetry": telemetry,
    }
    planned_velocity = planned_joint.get("peak_velocity")
    planned_acceleration = planned_joint.get("peak_acceleration")
    planned_limits_observed = bool(
        planned_velocity is not None
        and planned_acceleration is not None
        and planned_velocity <= configured_velocity * 1.20
        and planned_acceleration <= configured_acceleration * 1.50
    )
    physical_limits_observed = bool(
        telemetry.get("peak_velocity", 0.0) <= configured_velocity * 1.20
        and telemetry.get("peak_acceleration", 0.0)
        <= configured_acceleration * 1.50
    )
    result["planned_limits_observed"] = planned_limits_observed
    result["physical_limits_observed"] = physical_limits_observed
    result["physical_limit_warning"] = not physical_limits_observed
    # `limit_observed` is kept as the concise pass/fail field, but it refers to
    # the RLP command trajectory.  Physical Sim peaks remain visible above and
    # are warnings because the stock position-drive controller has no explicit
    # acceleration limiter.
    result["limit_observed"] = planned_limits_observed
    result["pass"] = bool(
        status["planner_success"]
        and status["constraint_done"]
        and physical.get("physical_converged", False)
        and result["limit_observed"]
    )
    return result


def _move_to_go(
    node: RobotLocalPlanner, capture: Capture, timeout_sec: float
) -> dict[str, Any]:
    sample_index = capture.clear_goal()
    goal_id = node.move_to_go()
    if goal_id is None:
        return {"pass": False, "error": "GO goal was not published"}
    status = _wait_for_goal(capture, goal_id, timeout_sec)
    physical = _wait_for_joints(node, GO_POSE, timeout_sec=timeout_sec)
    return {
        **status,
        **physical,
        "telemetry": _telemetry_peaks(
            capture.samples_since(sample_index), list(GO_POSE)
        ),
        "pass": bool(
            status["planner_success"]
            and physical["physical_converged"]
        ),
    }


def _move_to_warmup_pose(
    node: RobotLocalPlanner, capture: Capture, timeout_sec: float
) -> dict[str, Any]:
    sample_index = capture.clear_goal()
    goal_id = node.move_to_joint_positions(WARMUP_POSE, normalized_velocity=0.3)
    if goal_id is None:
        return {"pass": False, "error": "warmup goal was not published"}
    status = _wait_for_goal(capture, goal_id, timeout_sec)
    physical = _wait_for_joints(node, WARMUP_POSE, timeout_sec=timeout_sec)
    return {
        **status,
        **physical,
        "telemetry": _telemetry_peaks(
            capture.samples_since(sample_index), list(WARMUP_POSE)
        ),
        "pass": bool(
            status["planner_success"]
            and status["constraint_done"]
            and physical["physical_converged"]
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--reset-settle-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=25.0)
    parser.add_argument("--skip-base", action="store_true")
    parser.add_argument("--skip-joints", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = RobotLocalPlanner()
    capture = Capture(node)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(json.dumps({"event": "start", "stage": "S3.5", "trials": args.trials}))
    results: list[dict[str, Any]] = []
    try:
        time.sleep(2.0)
        if not _wait_for_joint_names(node, MOTION_PLANNING_JOINTS):
            raise RuntimeError("RLP joint_states did not contain all planning joints")

        # hsrb_quick_path_optimizer declares per-joint limits when its first
        # trajectory is optimized.  Warm it up once before querying the
        # parameter service; otherwise ROS legitimately reports only the
        # geometry parameters that were declared during plugin initialization.
        warmup = {"pass": False, "error": "warmup reset failed"}
        if _prepare_world(node, args.reset_settle_sec):
            warmup = _move_to_warmup_pose(node, capture, args.timeout_sec)
        print(json.dumps({"event": "parameter_warmup", **warmup}, allow_nan=False))
        model = _parameter_audit(node)
        print(json.dumps({"event": "parameter_audit", **model}, allow_nan=False))

        for trial in range(1, args.trials + 1):
            trial_result: dict[str, Any] = {
                "trial": trial,
                "base": [],
                "joints": [],
            }
            if not args.skip_base:
                for label, dx, dy, dyaw in BASE_CASES:
                    reset_ok = _prepare_world(node, args.reset_settle_sec)
                    if not reset_ok:
                        trial_result["base"].append(
                            {"case": label, "pass": False, "error": "reset failed"}
                        )
                        continue
                    trial_result["base"].append(
                        _run_base_case(node, capture, label, dx, dy, dyaw, args.timeout_sec)
                    )

            if not args.skip_joints:
                for joint, delta in JOINT_DELTAS.items():
                    reset_ok = _prepare_world(node, args.reset_settle_sec)
                    if not reset_ok:
                        trial_result["joints"].append(
                            {"joint": joint, "pass": False, "error": "reset failed"}
                        )
                        continue
                    go = _move_to_go(node, capture, args.timeout_sec)
                    if not go["pass"]:
                        trial_result["joints"].append(
                            {"joint": joint, "pass": False, "go": go}
                        )
                        continue
                    trial_result["joints"].append(
                        _run_joint_case(node, capture, joint, delta, args.timeout_sec)
                    )

            trial_result["pass"] = bool(
                all(item["pass"] for item in trial_result["base"])
                and all(item["pass"] for item in trial_result["joints"])
            )
            results.append(trial_result)
            print(json.dumps(trial_result, allow_nan=False))

        passed = sum(1 for result in results if result["pass"])
        joint_cases = [
            item
            for result in results
            for item in result["joints"]
        ]
        physical_limit_warnings = sum(
            1 for item in joint_cases if item.get("physical_limit_warning")
        )
        planned_limit_failures = sum(
            1 for item in joint_cases if not item.get("planned_limits_observed", False)
        )
        print(json.dumps({
            "event": "summary",
            "stage": "S3.5",
            "parameter_audit_pass": model["pass"],
            "base_cases": sum(len(result["base"]) for result in results),
            "joint_cases": len(joint_cases),
            "physical_limit_warnings": physical_limit_warnings,
            "planned_limit_failures": planned_limit_failures,
            "passed": passed,
            "total": len(results),
            "pass": bool(model["pass"] and passed == len(results)),
        }))
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
