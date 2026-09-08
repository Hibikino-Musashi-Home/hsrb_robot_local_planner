#!/usr/bin/env python3
"""Run the S6 BridgeA point-cloud/dynamic-obstacle regression.

BridgeA is intentionally kept outside the planner node.  It consumes the
``hma_pcl_reconst2`` PointCloud2 stream, removes the robot/ground ROI, fits a
small number of axis-aligned boxes in ``odom``, and registers the current box
snapshot through ``collision_environment_server/collision_object``.  The
RLP validator does not consume a PointCloud2 or an Octomap directly.

The matching Isaac Sim scene can publish a deterministic moving box on
``/rlp_validation/dynamic_obstacle_truth`` when started with
``SCENE=rlp_dynamic``.  The truth stream is only a test oracle; BridgeA never
uses it to construct the RLP collision object.

Run from the Apptainer shell after the common ROS setup, with the RLP node and
the ``hma_pcl_reconst2`` launch running in other terminals.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from itertools import product
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from hsrb_interface import Robot
from moveit_msgs.msg import CollisionObject
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from tmc_planning_msgs.msg import ConstraintsStatus, RobotLocalPlannerStatus

from hsrb_rlp_interface_py.robot_local_planner import RobotLocalPlanner

from s4_obstacle_runner import (
    Capture,
    _reset_world,
    _lookup_odom_base_pose,
    _wait_for_base_pose,
    _wait_for_goal,
)


POINT_CLOUD_TOPIC = "/hma_pcl_reconst/depth_registered/points"
ENVIRONMENT_CONTROL_TOPIC = "collision_environment_server/collision_object"
SIM_TRUTH_TOPIC = "/rlp_validation/dynamic_obstacle_truth"
SIM_CONTACT_TOPIC = "/rlp_validation/dynamic_obstacle_contact"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
DYNAMIC_OBJECT_ID = "s6_dynamic_obstacle"


def _emit(event: str, **values: Any) -> None:
    print(json.dumps({"event": event, **values}, ensure_ascii=False,
                     allow_nan=False), flush=True)


@dataclass(frozen=True)
class BridgeConfig:
    input_topic: str = POINT_CLOUD_TOPIC
    object_id: str = DYNAMIC_OBJECT_ID
    voxel_size: float = 0.05
    point_stride: int = 8
    min_cluster_points: int = 30
    min_cluster_height: float = 0.06
    min_x: float = 0.35
    max_x: float = 1.60
    min_y: float = -1.00
    max_y: float = 1.00
    min_z: float = 0.04
    max_z: float = 1.10
    self_radius: float = 0.46
    padding_xy: float = 0.07
    padding_z: float = 0.04
    min_dimension_xy: float = 0.16
    min_dimension_z: float = 0.20
    max_dimension_xy: float = 0.90
    stale_timeout_sec: float = 1.0
    update_period_sec: float = 0.20


@dataclass(frozen=True)
class DetectedBox:
    center_xy: tuple[float, float]
    dimensions_xyz: tuple[float, float, float]
    bottom_z: float
    point_count: int
    voxel_count: int
    source_frame: str
    cloud_stamp: float
    processing_wall_sec: float


def _stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rotation_matrix(q: Any) -> np.ndarray:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float32)


def _lookup_transform(tf_buffer: Any, target: str, source: str,
                      stamp: Any) -> tuple[Any, str]:
    """Return a transform and whether it used the message or latest time."""
    requested = Time.from_msg(stamp) if _stamp_to_float(stamp) > 0.0 else Time()
    try:
        return tf_buffer.lookup_transform(
            target, source, requested, timeout=Duration(seconds=0.10)), "message"
    except Exception:
        # Sim and camera publishers can differ by one rendered frame.  A
        # latest TF fallback keeps BridgeA alive while retaining the mode in
        # diagnostics instead of silently pretending the timestamps matched.
        return tf_buffer.lookup_transform(
            target, source, Time(), timeout=Duration(seconds=0.10)), "latest"


def _point_cloud_xyz(cloud: PointCloud2, stride: int) -> np.ndarray:
    points = point_cloud2.read_points(
        cloud, field_names=["x", "y", "z"], skip_nans=True)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    xyz = np.asarray(xyz, dtype=np.float32)
    stride = max(1, int(stride))
    return xyz[::stride]


def _transform_points(
    xyz: np.ndarray,
    transform: Any,
) -> np.ndarray:
    rotation = _rotation_matrix(transform.transform.rotation)
    translation = np.array([
        float(transform.transform.translation.x),
        float(transform.transform.translation.y),
        float(transform.transform.translation.z),
    ], dtype=np.float32)
    return xyz @ rotation.T + translation


def _cluster_indices(points: np.ndarray, voxel_size: float) -> list[np.ndarray]:
    """Find connected 2-D voxel components while retaining all z samples."""
    if len(points) == 0:
        return []
    grid = np.floor(points[:, :2] / max(1e-3, voxel_size)).astype(np.int32)
    cells, inverse = np.unique(grid, axis=0, return_inverse=True)
    cell_lookup = {tuple(cell): index for index, cell in enumerate(cells)}
    neighbours = list(product((-1, 0, 1), repeat=2))
    visited: set[int] = set()
    components: list[np.ndarray] = []
    for start in range(len(cells)):
        if start in visited:
            continue
        pending = [start]
        visited.add(start)
        component: list[int] = []
        while pending:
            current = pending.pop()
            component.append(current)
            cx, cy = (int(value) for value in cells[current])
            for dx, dy in neighbours:
                if dx == 0 and dy == 0:
                    continue
                other = cell_lookup.get((cx + dx, cy + dy))
                if other is not None and other not in visited:
                    visited.add(other)
                    pending.append(other)
        point_indices = np.flatnonzero(np.isin(inverse, component))
        components.append(point_indices)
    return components


def detect_box(
    cloud: PointCloud2,
    tf_buffer: Any,
    config: BridgeConfig,
) -> tuple[DetectedBox | None, dict[str, Any]]:
    """Convert one PointCloud2 message into the largest plausible object box."""
    started = time.monotonic()
    xyz = _point_cloud_xyz(cloud, config.point_stride)
    if len(xyz) == 0:
        return None, {"input_points": 0, "roi_points": 0}

    try:
        odom_tf, tf_mode = _lookup_transform(
            tf_buffer, ODOM_FRAME, cloud.header.frame_id, cloud.header.stamp)
        points = _transform_points(xyz, odom_tf)
        base_tf, _ = _lookup_transform(
            tf_buffer, ODOM_FRAME, BASE_FRAME, cloud.header.stamp)
        base_xy = np.array([
            float(base_tf.transform.translation.x),
            float(base_tf.transform.translation.y),
        ], dtype=np.float32)
    except Exception as exc:
        return None, {
            "input_points": int(len(xyz)),
            "roi_points": 0,
            "tf_error": str(exc),
        }

    finite = np.isfinite(points).all(axis=1)
    roi = (
        finite
        & (points[:, 0] >= config.min_x)
        & (points[:, 0] <= config.max_x)
        & (points[:, 1] >= config.min_y)
        & (points[:, 1] <= config.max_y)
        & (points[:, 2] >= config.min_z)
        & (points[:, 2] <= config.max_z)
    )
    # The x crop removes the base itself; the radial test additionally removes
    # the visible bumper/caster pixels when the camera looks down at the base.
    radial_sq = np.sum((points[:, :2] - base_xy) ** 2, axis=1)
    roi &= radial_sq >= config.self_radius ** 2
    filtered = points[roi]
    diagnostics: dict[str, Any] = {
        "input_points": int(len(xyz)),
        "roi_points": int(len(filtered)),
        "tf_mode": tf_mode,
    }
    if len(filtered) < config.min_cluster_points:
        return None, diagnostics

    candidates: list[tuple[float, np.ndarray, np.ndarray, int]] = []
    for indices in _cluster_indices(filtered, config.voxel_size):
        if len(indices) < config.min_cluster_points:
            continue
        cluster = filtered[indices]
        mins = cluster.min(axis=0)
        maxs = cluster.max(axis=0)
        extent = maxs - mins
        if extent[2] < config.min_cluster_height:
            continue
        if extent[0] > config.max_dimension_xy or extent[1] > config.max_dimension_xy:
            continue
        # Point count is the primary score.  A small height bonus suppresses
        # sparse tabletop/floor remnants without hard-coding a target object.
        score = float(len(indices)) * (1.0 + min(float(extent[2]), 0.5))
        candidates.append((score, mins, maxs, len(indices)))

    diagnostics["candidate_count"] = len(candidates)
    if not candidates:
        return None, diagnostics
    _, mins, maxs, point_count = max(candidates, key=lambda item: item[0])
    extent = maxs - mins
    bottom_z = 0.0 if mins[2] <= 0.12 else max(0.0, float(mins[2] - config.padding_z))
    top_z = float(maxs[2] + config.padding_z)
    dimensions = np.array([
        max(config.min_dimension_xy, float(extent[0]) + 2.0 * config.padding_xy),
        max(config.min_dimension_xy, float(extent[1]) + 2.0 * config.padding_xy),
        max(config.min_dimension_z, top_z - bottom_z),
    ], dtype=np.float32)
    dimensions[0:2] = np.minimum(dimensions[0:2], config.max_dimension_xy)
    center = (mins + maxs) * 0.5
    detected = DetectedBox(
        center_xy=(float(center[0]), float(center[1])),
        dimensions_xyz=tuple(float(value) for value in dimensions),
        bottom_z=float(bottom_z),
        point_count=int(point_count),
        voxel_count=int(point_count),
        source_frame=cloud.header.frame_id,
        cloud_stamp=_stamp_to_float(cloud.header.stamp),
        processing_wall_sec=time.monotonic() - started,
    )
    diagnostics.update({
        "selected_point_count": detected.point_count,
        "selected_center_xy": list(detected.center_xy),
        "selected_dimensions_xyz": list(detected.dimensions_xyz),
        "selected_bottom_z": detected.bottom_z,
        "processing_wall_sec": detected.processing_wall_sec,
    })
    return detected, diagnostics


class TruthCapture:
    """Capture Sim truth for comparison; it is never used by BridgeA."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._lock = threading.Lock()
        self._history: deque[dict[str, Any]] = deque(maxlen=300)
        self._contact = False
        node.create_subscription(String, SIM_TRUTH_TOPIC, self._truth_callback, 20)
        node.create_subscription(Bool, SIM_CONTACT_TOPIC, self._contact_callback, 10)

    def _truth_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._history.append(data)

    def _contact_callback(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._contact = True

    def contact_seen(self) -> bool:
        with self._lock:
            return self._contact

    def history(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._history)

    def latest_visible(self) -> dict[str, Any] | None:
        with self._lock:
            for item in reversed(self._history):
                if item.get("visible"):
                    return dict(item)
        return None

    def wait_for(self, predicate: Any, timeout_sec: float) -> dict[str, Any] | None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                for item in reversed(self._history):
                    if predicate(item):
                        return dict(item)
            time.sleep(0.05)
        return None

    def closest(self, sim_time: float) -> dict[str, Any] | None:
        with self._lock:
            items = list(self._history)
        if not items:
            return None
        return min(items, key=lambda item: abs(float(item.get("sim_time", 0.0)) - sim_time))


class BridgeA:
    """Asynchronous PointCloud2 -> CollisionObject bridge."""

    def __init__(self, node: RobotLocalPlanner, config: BridgeConfig) -> None:
        self.node = node
        self.config = config
        self._lock = threading.Lock()
        self._latest_cloud: PointCloud2 | None = None
        self._cloud_event = threading.Event()
        self._stop_event = threading.Event()
        self._known_object = False
        self._last_detection_wall = 0.0
        self._last_publish_wall = 0.0
        self._last_cloud_stamp = -1.0
        self._latest: DetectedBox | None = None
        self._history: deque[dict[str, Any]] = deque(maxlen=300)
        self._environment_pub = node.create_publisher(
            CollisionObject, ENVIRONMENT_CONTROL_TOPIC, 10)
        node.create_subscription(
            PointCloud2,
            config.input_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="s6-bridge-a",
            daemon=True,
        )
        self._worker.start()

    def _cloud_callback(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_cloud = msg
        self._cloud_event.set()

    def wait_for_cloud(self, timeout_sec: float) -> bool:
        return self._cloud_event.wait(timeout_sec)

    def _record(self, item: dict[str, Any]) -> None:
        with self._lock:
            self._history.append(dict(item))

    def history(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._history)

    def latest(self) -> DetectedBox | None:
        with self._lock:
            return self._latest

    def _publish_add(self, detected: DetectedBox) -> None:
        message = CollisionObject()
        message.header.frame_id = ODOM_FRAME
        message.id = self.config.object_id
        message.operation = CollisionObject.ADD
        message.pose.position.x = detected.center_xy[0]
        message.pose.position.y = detected.center_xy[1]
        message.pose.position.z = detected.bottom_z + detected.dimensions_xyz[2] * 0.5
        message.pose.orientation.w = 1.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(detected.dimensions_xyz)
        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        message.primitives = [primitive]
        message.primitive_poses = [primitive_pose]
        self._environment_pub.publish(message)
        self._known_object = True

    def _publish_remove(self, reason: str) -> None:
        message = CollisionObject()
        message.header.frame_id = ODOM_FRAME
        message.id = self.config.object_id
        message.operation = CollisionObject.REMOVE
        self._environment_pub.publish(message)
        self._known_object = False
        self._record({
            "event": "bridge_remove",
            "object_id": self.config.object_id,
            "reason": reason,
            "wall_time": time.monotonic(),
        })
        _emit("bridge_remove", object_id=self.config.object_id, reason=reason)

    def _process_cloud(self, cloud: PointCloud2) -> None:
        stamp = _stamp_to_float(cloud.header.stamp)
        if stamp <= self._last_cloud_stamp:
            return
        self._last_cloud_stamp = stamp
        detected, diagnostics = detect_box(cloud, self.node._tf2_buffer, self.config)
        now = time.monotonic()
        if detected is not None:
            self._last_detection_wall = now
            with self._lock:
                self._latest = detected
            should_publish = (
                not self._known_object
                or now - self._last_publish_wall >= self.config.update_period_sec
            )
            if should_publish:
                self._publish_add(detected)
                self._last_publish_wall = now
                item = {
                    "event": "bridge_update",
                    "object_id": self.config.object_id,
                    "wall_time": now,
                    "center_xy": list(detected.center_xy),
                    "dimensions_xyz": list(detected.dimensions_xyz),
                    "bottom_z": detected.bottom_z,
                    "point_count": detected.point_count,
                    "cloud_stamp": detected.cloud_stamp,
                    "source_frame": detected.source_frame,
                    "processing_wall_sec": detected.processing_wall_sec,
                    **diagnostics,
                }
                self._record(item)
                _emit("bridge_update", **{key: value for key, value in item.items()
                                          if key != "event"})
        elif self._known_object and now - self._last_detection_wall >= self.config.stale_timeout_sec:
            self._publish_remove("pointcloud_stale")

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            self._cloud_event.wait(0.10)
            with self._lock:
                cloud = self._latest_cloud
            if cloud is not None:
                try:
                    self._process_cloud(cloud)
                except Exception as exc:
                    _emit("bridge_error", error=str(exc))
            if self._known_object and (
                time.monotonic() - self._last_detection_wall >= self.config.stale_timeout_sec
            ):
                self._publish_remove("pointcloud_stale")

    def clear(self) -> None:
        if self._known_object:
            for _ in range(3):
                self._publish_remove("shutdown")
                time.sleep(0.05)

    def stop(self) -> None:
        self._stop_event.set()
        self._cloud_event.set()
        self._worker.join(timeout=2.0)
        self.clear()


def _wait_for_bridge_detection(bridge: BridgeA, timeout_sec: float) -> DetectedBox | None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        detected = bridge.latest()
        if detected is not None:
            return detected
        time.sleep(0.05)
    return None


def _closest_truth(truth: TruthCapture, detected: DetectedBox) -> dict[str, Any] | None:
    return truth.closest(detected.cloud_stamp)


def _center_error(detected: DetectedBox, expected: dict[str, Any] | None) -> float | None:
    if expected is None or not expected.get("visible"):
        return None
    center = expected.get("center_world")
    if not isinstance(center, list) or len(center) < 2:
        return None
    return math.hypot(detected.center_xy[0] - float(center[0]),
                     detected.center_xy[1] - float(center[1]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default=POINT_CLOUD_TOPIC)
    parser.add_argument("--object-id", default=DYNAMIC_OBJECT_ID)
    parser.add_argument("--target-x", type=float, default=1.20)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    parser.add_argument("--stale-timeout-sec", type=float, default=1.0)
    parser.add_argument("--skip-base-goal", action="store_true")
    args = parser.parse_args()

    config = BridgeConfig(
        input_topic=args.input_topic,
        object_id=args.object_id,
        stale_timeout_sec=args.stale_timeout_sec,
    )
    rclpy.init()
    node = RobotLocalPlanner()
    capture = Capture(node)
    truth = TruthCapture(node)
    bridge = BridgeA(node, config)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    _emit(
        "start",
        stage="S6.2",
        bridge="BridgeA",
        point_cloud_topic=args.input_topic,
        environment_control_topic=ENVIRONMENT_CONTROL_TOPIC,
        truth_topic=SIM_TRUTH_TOPIC,
        object_id=args.object_id,
        stale_timeout_sec=args.stale_timeout_sec,
        skip_base_goal=args.skip_base_goal,
    )
    result: dict[str, Any] = {
        "stage": "S6.2",
        "bridge": "BridgeA",
        "pass": False,
    }
    try:
        reset_ok = _reset_world(node, args.reset_settle_sec)
        robot = Robot()
        whole_body = robot.get("whole_body")
        whole_body.move_to_neutral(sync=True)
        observation_pose = {
            "arm_lift_joint": 0.0,
            "arm_flex_joint": 0.0,
            "arm_roll_joint": 0.0,
            "wrist_flex_joint": -1.57,
            "wrist_roll_joint": 0.0,
            "head_pan_joint": -0.65,
            "head_tilt_joint": math.radians(-50.0),
        }
        whole_body.move_to_joint_positions(observation_pose, sync=True)
        time.sleep(0.5)
        cloud_connected = bridge.wait_for_cloud(10.0)
        env_connected = bridge._environment_pub.get_subscription_count() > 0
        truth_visible = truth.wait_for(lambda item: bool(item.get("visible")), 12.0)
        detected = _wait_for_bridge_detection(bridge, 12.0)
        initial_truth = _closest_truth(truth, detected) if detected else None
        initial_error = _center_error(detected, initial_truth) if detected else None
        _emit(
            "snapshot_check",
            cloud_connected=cloud_connected,
            environment_subscriber=env_connected,
            truth_visible=truth_visible is not None,
            detected=detected is not None,
            center_error_m=initial_error,
        )

        # Start the whole-body target while the obstacle is still present.
        # The following point-cloud and stale-object checks deliberately run
        # while this goal is in flight, so S6 tests an online environment
        # update rather than only a pre-planning snapshot.
        goal_id: str | None = None
        goal_started = time.monotonic()
        if not args.skip_base_goal:
            if _lookup_odom_base_pose(node) is not None:
                capture.clear()
                goal_id = node.move_base_any_frame(
                    args.target_x, args.target_y, 0.0, ODOM_FRAME)
                goal_started = time.monotonic()

        motion_deadline = time.monotonic() + 12.0
        initial_y = detected.center_xy[1] if detected else None
        max_y_delta = 0.0
        update_count = 0
        while time.monotonic() < motion_deadline:
            history = [item for item in bridge.history() if item.get("event") == "bridge_update"]
            update_count = len(history)
            if initial_y is not None and history:
                max_y_delta = max(
                    max_y_delta,
                    max(abs(float(item["center_xy"][1]) - initial_y) for item in history),
                )
            if max_y_delta >= 0.20 and update_count >= 2:
                break
            time.sleep(0.10)
        motion_updated = max_y_delta >= 0.20 and update_count >= 2
        _emit(
            "motion_check",
            update_count=update_count,
            max_detected_y_delta_m=max_y_delta,
            motion_updated=motion_updated,
        )

        stale_truth = truth.wait_for(lambda item: not bool(item.get("visible")), 20.0)
        stale_deadline = time.monotonic() + max(3.0, args.stale_timeout_sec + 2.0)
        stale_removed = False
        while time.monotonic() < stale_deadline:
            stale_removed = any(
                item.get("event") == "bridge_remove"
                and item.get("reason") == "pointcloud_stale"
                for item in bridge.history()
            )
            if stale_removed:
                break
            time.sleep(0.10)
        _emit(
            "stale_check",
            truth_hidden=stale_truth is not None,
            bridge_removed=stale_removed,
        )

        goal_result: dict[str, Any] = {"skipped": args.skip_base_goal}
        if goal_id is not None:
            remaining = max(1.0, args.timeout_sec - (time.monotonic() - goal_started))
            planner_status, constraint_status, statuses, status_wait = _wait_for_goal(
                capture, goal_id, remaining)
            physical = {"physical_converged": False}
            if (RobotLocalPlannerStatus.SUCCESS in statuses
                    and constraint_status in {
                        ConstraintsStatus.SATISFIED,
                        ConstraintsStatus.PREEMPTED,
                    }):
                physical = _wait_for_base_pose(
                    node, (args.target_x, args.target_y, 0.0),
                    max(1.0, remaining - status_wait),
                )
            goal_result = {
                "goal_id": goal_id,
                "planner_status": planner_status,
                "planner_statuses": statuses,
                "constraint_status": constraint_status,
                "status_wait_wall_sec": status_wait,
                **physical,
            }
        elif not args.skip_base_goal:
            goal_result = {"error": "base_pose_unavailable"}
        _emit("goal_check", **goal_result)

        goal_ok = bool(
            args.skip_base_goal
            or (
                RobotLocalPlannerStatus.SUCCESS in goal_result.get("planner_statuses", [])
                and goal_result.get("constraint_status") in {
                    ConstraintsStatus.SATISFIED,
                    ConstraintsStatus.PREEMPTED,
                }
                and goal_result.get("physical_converged", False)
            )
        )
        result.update({
            "cloud_connected": cloud_connected,
            "reset_ok": reset_ok,
            "environment_subscriber": env_connected,
            "truth_visible": truth_visible is not None,
            "initial_detected": detected is not None,
            "initial_center_error_m": initial_error,
            "motion_updated": motion_updated,
            "update_count": update_count,
            "max_detected_y_delta_m": max_y_delta,
            "truth_hidden": stale_truth is not None,
            "bridge_removed_stale_object": stale_removed,
            "goal": goal_result,
            "physical_dynamic_obstacle_contact": truth.contact_seen(),
            "pass": bool(
                cloud_connected
                and reset_ok
                and env_connected
                and truth_visible is not None
                and detected is not None
                and initial_error is not None
                and initial_error <= 0.20
                and motion_updated
                and stale_truth is not None
                and stale_removed
                and goal_ok
                and not truth.contact_seen()
            ),
        })
        _emit("summary", **result)
        return 0 if result["pass"] else 1
    finally:
        node.publish_empty_constraints()
        bridge.stop()
        time.sleep(0.3)
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
