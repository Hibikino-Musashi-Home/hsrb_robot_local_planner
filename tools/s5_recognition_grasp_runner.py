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

import numpy as np
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
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from grasp_point_detection_interfaces.srv import GraspPointService
from hma_grounding_dino2_interfaces.srv import Detection2DService
from s4_obstacle_runner import Box, EnvironmentPublisher
from s6_pcl_dynamic_obstacle_runner import (
    BridgeA,
    BridgeConfig,
    TruthCapture as DynamicTruthCapture,
    _center_error as _dynamic_center_error,
    _closest_truth as _dynamic_closest_truth,
    _wait_for_bridge_detection as _wait_for_dynamic_detection,
)
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
TABLE_DETECTION_SERVICE = "/object_detection/grounding_dino2/service"
TABLE_POINT_CLOUD_TOPIC = "/hma_pcl_reconst/depth_registered/points"
TABLE_TRUTH_TOPIC = "/rlp_validation/table_truth"
CABINET_CONTACT_TOPIC = "/rlp_validation/cabinet_contact"

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


class PointCloudCapture:
    """Keep the latest organized RGB-D cloud for table geometry estimation."""

    def __init__(self, node: RobotLocalPlanner, topic: str) -> None:
        self._lock = threading.Lock()
        self._latest: PointCloud2 | None = None
        self._event = threading.Event()
        node.create_subscription(
            PointCloud2,
            topic,
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest = msg
        self._event.set()

    def latest(self) -> PointCloud2 | None:
        with self._lock:
            return self._latest

    def wait_for_cloud(self, timeout_sec: float) -> bool:
        return self._event.wait(timeout_sec)


class TableTruthCapture:
    """Capture the independent Sim table oracle for post-hoc diagnostics."""

    def __init__(self, node: RobotLocalPlanner) -> None:
        self._lock = threading.Lock()
        self._latest: dict[str, Any] | None = None
        node.create_subscription(
            String,
            TABLE_TRUTH_TOPIC,
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self._latest = payload

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def wait_for_visible(self, timeout_sec: float) -> dict[str, Any] | None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            value = self.latest()
            if value is not None and bool(value.get("visible", False)):
                return value
            time.sleep(0.05)
        return self.latest()


class ContactCapture:
    """Capture a Sim contact oracle for one static validation obstacle."""

    def __init__(self, node: RobotLocalPlanner, topic: str) -> None:
        self._lock = threading.Lock()
        self._seen = False
        node.create_subscription(
            Bool,
            topic,
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._seen = True

    def seen(self) -> bool:
        with self._lock:
            return self._seen


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


def _stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rotation_matrix(quaternion: Any) -> np.ndarray:
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _cloud_points_in_bbox(
    cloud: PointCloud2,
    bbox: Any,
    stride: int,
) -> np.ndarray:
    """Read bbox pixels from an organized cloud without losing pixel indices.

    ``read_points`` applies NaN filtering before indexing on Humble, so
    ``skip_nans=False`` is intentional here.  GroundingDINO bbox coordinates
    are top-left ``(x, y, w, h)`` pixels.
    """
    width = int(cloud.width)
    height = int(cloud.height)
    if width <= 0 or height <= 0:
        return np.empty((0, 3), dtype=np.float64)
    x0 = max(0, min(width - 1, int(round(float(bbox.x)))))
    y0 = max(0, min(height - 1, int(round(float(bbox.y)))))
    x1 = max(x0 + 1, min(width, x0 + int(round(float(bbox.w)))))
    y1 = max(y0 + 1, min(height, y0 + int(round(float(bbox.h)))))
    step = max(1, int(stride))
    uvs = np.asarray(
        [v * width + u
         for v in range(y0, y1, step)
         for u in range(x0, x1, step)],
        dtype=np.int64,
    )
    if uvs.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    points = point_cloud2.read_points(
        cloud,
        field_names=["x", "y", "z"],
        skip_nans=False,
        uvs=uvs,
    )
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    xyz = np.asarray(xyz, dtype=np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def _transform_cloud_points(
    node: RobotLocalPlanner,
    cloud: PointCloud2,
    xyz: np.ndarray,
) -> tuple[np.ndarray, str]:
    if len(xyz) == 0:
        return xyz, "none"
    stamp = Time.from_msg(cloud.header.stamp) if _stamp_to_float(
        cloud.header.stamp) > 0.0 else Time()
    try:
        transform = node._tf2_buffer.lookup_transform(
            ODOM_FRAME,
            cloud.header.frame_id,
            stamp,
            timeout=Duration(seconds=0.15),
        ).transform
        mode = "message"
    except Exception:
        transform = node._tf2_buffer.lookup_transform(
            ODOM_FRAME,
            cloud.header.frame_id,
            Time(),
            timeout=Duration(seconds=0.15),
        ).transform
        mode = "latest"
    translation = np.asarray(
        [
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ],
        dtype=np.float64,
    )
    return xyz @ _rotation_matrix(transform.rotation).T + translation, mode


def _estimate_table_box(
    node: RobotLocalPlanner,
    cloud: PointCloud2,
    bbox: Any,
    point_stride: int,
    placement_patch_size: float = 0.20,
    object_id: str = "s6_3_detected_table",
) -> tuple[Box, dict[str, Any]]:
    """Estimate a safe tabletop patch from a detector bbox and RGB-D cloud.

    The estimator does not know the Sim table pose.  It searches for a dense,
    nearly horizontal height layer and makes a small placement patch around
    the observed tabletop center.  A partial view is still useful: placing at
    a measured patch is safer than extrapolating an unseen table edge.
    """
    raw_xyz = _cloud_points_in_bbox(cloud, bbox, point_stride)
    points, tf_mode = _transform_cloud_points(node, cloud, raw_xyz)
    diagnostics: dict[str, Any] = {
        "cloud_frame": cloud.header.frame_id,
        "cloud_stamp": _stamp_to_float(cloud.header.stamp),
        "cloud_width": int(cloud.width),
        "cloud_height": int(cloud.height),
        "input_points": int(len(raw_xyz)),
        "tf_mode": tf_mode,
    }
    if len(points) == 0:
        raise RuntimeError("机bbox内の有効なPointCloud2点がありません")

    finite = np.isfinite(points).all(axis=1)
    roi = (
        finite
        & (points[:, 0] >= 0.15)
        & (points[:, 0] <= 2.20)
        & (points[:, 1] >= -1.20)
        & (points[:, 1] <= 1.20)
        & (points[:, 2] >= 0.12)
        & (points[:, 2] <= 1.20)
    )
    base_tf = _lookup_transform(node, ODOM_FRAME, BASE_FRAME)
    radial_sq = (
        (points[:, 0] - float(base_tf.translation.x)) ** 2
        + (points[:, 1] - float(base_tf.translation.y)) ** 2
    )
    roi &= radial_sq >= 0.30 ** 2
    filtered = points[roi]
    diagnostics["roi_points"] = int(len(filtered))
    if len(filtered) < 80:
        raise RuntimeError(
            f"机bbox内の天板候補点が少なすぎます: {len(filtered)}点"
        )

    # A horizontal tabletop creates a sharp z peak.  Score a +/-15 mm slice
    # using density and horizontal footprint; vertical background edges spread
    # samples across many bins and are penalized by MAD.
    bin_edges = np.arange(0.12, 1.205, 0.01)
    counts, _ = np.histogram(filtered[:, 2], bins=bin_edges)
    candidates: list[dict[str, Any]] = []
    for index, count in enumerate(counts):
        if int(count) < 20:
            continue
        center_z = float((bin_edges[index] + bin_edges[index + 1]) * 0.5)
        slice_points = filtered[np.abs(filtered[:, 2] - center_z) <= 0.015]
        if len(slice_points) < 40:
            continue
        xy_min = np.quantile(slice_points[:, :2], 0.05, axis=0)
        xy_max = np.quantile(slice_points[:, :2], 0.95, axis=0)
        extent = xy_max - xy_min
        median_z = float(np.median(slice_points[:, 2]))
        mad_z = float(np.median(np.abs(slice_points[:, 2] - median_z)))
        footprint = max(0.0, float(extent[0] * extent[1]))
        score = (
            float(len(slice_points))
            * (1.0 + min(3.0, footprint / 0.03))
            / (1.0 + 80.0 * mad_z)
        )
        candidates.append({
            "z_m": median_z,
            "count": int(len(slice_points)),
            "extent_xy_m": [float(extent[0]), float(extent[1])],
            "mad_z_m": mad_z,
            "score": score,
        })
    diagnostics["candidate_count"] = len(candidates)
    diagnostics["candidates"] = sorted(
        candidates, key=lambda item: item["score"], reverse=True
    )[:10]
    if not candidates:
        raise RuntimeError("机bbox内に水平な天板候補が見つかりません")

    selected = max(candidates, key=lambda item: item["score"])
    top_z = float(selected["z_m"])
    top_slice = filtered[np.abs(filtered[:, 2] - top_z) <= 0.025]
    if len(top_slice) < 40:
        raise RuntimeError("選択した天板高さの点が不足しています")
    xy_min = np.quantile(top_slice[:, :2], 0.05, axis=0)
    xy_max = np.quantile(top_slice[:, :2], 0.95, axis=0)
    observed_extent_x = float(xy_max[0] - xy_min[0])
    observed_extent_y = float(xy_max[1] - xy_min[1])
    center_x = float((xy_min[0] + xy_max[0]) * 0.5)
    center_y = float((xy_min[1] + xy_max[1]) * 0.5)
    # The detector often sees only one part of a large tabletop.  Do not
    # claim the unseen table boundary as a collision object: use a measured
    # placement patch that is large enough for the YCB apple but small enough
    # to stay inside the observed plane and leave the mobile base a route.
    requested_patch = max(0.16, min(0.24, float(placement_patch_size)))
    size_x = max(0.16, min(requested_patch, observed_extent_x - 0.02))
    size_y = max(0.16, min(requested_patch, observed_extent_y - 0.02))
    thickness = 0.03
    table = Box(
        object_id,
        (center_x, center_y),
        (size_x, size_y, thickness),
        bottom_z=top_z - thickness,
    )
    diagnostics.update({
        "selected_top_z_m": top_z,
        "selected_center_xy": [center_x, center_y],
        "observed_extent_xy_m": [observed_extent_x, observed_extent_y],
        "placement_patch_policy": "measured_centered_patch",
        "selected_dimensions_xyz": [size_x, size_y, thickness],
        "selected_bottom_z": top_z - thickness,
        "selected_points": int(len(top_slice)),
    })
    return table, diagnostics


def _table_truth_check(
    table: Box,
    truth: dict[str, Any] | None,
) -> dict[str, Any]:
    """Compare detected geometry with Sim truth without feeding truth back."""
    result: dict[str, Any] = {"available": False}
    if truth is None or not bool(truth.get("visible", False)):
        return result
    center = truth.get("center_world")
    dimensions = truth.get("dimensions_xyz")
    top_z = truth.get("top_z")
    if not isinstance(center, list) or len(center) < 2:
        return result
    if not isinstance(dimensions, list) or len(dimensions) < 2:
        return result
    if top_z is None:
        return result
    detected_top = table.bottom_z + table.dimensions[2]
    dx = table.center[0] - float(center[0])
    dy = table.center[1] - float(center[1])
    truth_half_x = float(dimensions[0]) * 0.5
    truth_half_y = float(dimensions[1]) * 0.5
    detected_inside = (
        abs(dx) + table.dimensions[0] * 0.5 <= truth_half_x
        and abs(dy) + table.dimensions[1] * 0.5 <= truth_half_y
    )
    result.update({
        "available": True,
        "truth_center_xy": [float(center[0]), float(center[1])],
        "truth_dimensions_xy": [float(dimensions[0]), float(dimensions[1])],
        "truth_top_z_m": float(top_z),
        "center_error_m": math.hypot(dx, dy),
        "top_z_error_m": abs(detected_top - float(top_z)),
        "detected_patch_inside_truth_table": detected_inside,
        "pass": bool(
            detected_inside and abs(detected_top - float(top_z)) <= 0.07
        ),
    })
    return result


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


def _quaternion_conjugate(quaternion: Any) -> geometry.Quaternion:
    return geometry.Quaternion(
        -quaternion.x,
        -quaternion.y,
        -quaternion.z,
        quaternion.w,
    )


def _rotate_offset_between_orientations(
    offset_xyz: tuple[float, float, float],
    source_orientation: Any,
    target_orientation: Any,
) -> tuple[float, float, float]:
    """Rotate a rigid object-to-hand offset when the hand changes posture."""
    relative_rotation = _multiply_quaternions(
        target_orientation,
        _quaternion_conjugate(source_orientation),
    )
    return _rotate_vector(
        relative_rotation,
        geometry.Vector3(*offset_xyz),
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


def _local_xy_to_odom(
    reference_pose: tuple[float, float, float],
    local_x: float,
    local_y: float,
) -> tuple[float, float]:
    """Convert a point expressed in the current base frame to odom."""
    reference_x, reference_y, reference_yaw = reference_pose
    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)
    return (
        reference_x + cos_yaw * local_x - sin_yaw * local_y,
        reference_y + sin_yaw * local_x + cos_yaw * local_y,
    )


def _yaw_from_quaternion(rotation: Any) -> float:
    return math.atan2(
        2.0 * (
            rotation.w * rotation.z
            + rotation.x * rotation.y
        ),
        1.0 - 2.0 * (
            rotation.y * rotation.y
            + rotation.z * rotation.z
        ),
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


def _detect_table_geometry(
    node: RobotLocalPlanner,
    table_client: Any,
    cloud_capture: PointCloudCapture,
    truth_capture: TableTruthCapture,
    args: argparse.Namespace,
) -> tuple[Box, dict[str, Any]]:
    """Run GroundingDINO and derive a tabletop Box from the matching cloud."""
    if not table_client.wait_for_service(timeout_sec=15.0):
        raise RuntimeError(
            f"机検出サービスが見つかりません: {TABLE_DETECTION_SERVICE}"
        )
    if not cloud_capture.wait_for_cloud(timeout_sec=12.0):
        raise RuntimeError(
            f"机検出用PointCloud2が届きません: {TABLE_POINT_CLOUD_TOPIC}"
        )

    detection_attempts: list[list[str]] = []
    last_error = ""
    for _attempt in range(1, 6):
        request = Detection2DService.Request()
        request.confidence_th = args.table_confidence
        request.iou_th = args.table_iou
        request.use_latest_image = True
        request.max_distance = args.table_max_distance
        request.specific_id = ""
        # Keep the prompt singular.  GroundingDINO can return a full-frame
        # "table desk" box when multiple prompts are combined, which is less
        # useful for the depth crop than a single table hypothesis.
        request.class_prompts = [args.table_prompt]
        response = _wait_future(table_client.call_async(request), 30.0)
        detections = response.detections_2d
        labels = [bbox.name for bbox in detections.bbox]
        detection_attempts.append(labels)
        candidates = [
            (index, bbox)
            for index, bbox in enumerate(detections.bbox)
            if args.table_prompt.lower() in bbox.name.lower()
            or "table" in bbox.name.lower()
            or "desk" in bbox.name.lower()
        ]
        if not candidates:
            last_error = f"机bboxなし: {labels}"
            time.sleep(0.4)
            continue
        detection_index, bbox = max(candidates, key=lambda item: item[1].score)
        cloud = cloud_capture.latest()
        if cloud is None:
            last_error = "机bboxに対応するPointCloud2がありません"
            time.sleep(0.2)
            continue
        try:
            table, cloud_diagnostics = _estimate_table_box(
                node,
                cloud,
                bbox,
                args.table_point_stride,
                args.table_patch_size,
            )
        except Exception as exc:
            last_error = str(exc)
            time.sleep(0.4)
            continue

        truth = truth_capture.wait_for_visible(5.0)
        truth_check = _table_truth_check(table, truth)
        result = {
            "service": TABLE_DETECTION_SERVICE,
            "prompt": args.table_prompt,
            "name": bbox.name,
            "score": float(bbox.score),
            "bbox_top_left_xywh": [
                float(bbox.x), float(bbox.y), float(bbox.w), float(bbox.h)
            ],
            "frame_id": detections.header.frame_id,
            "all_labels": labels,
            "attempt_labels": detection_attempts,
            "cloud": cloud_diagnostics,
            "table_box": {
                "id": table.name,
                "center_xy": list(table.center),
                "dimensions_xyz": list(table.dimensions),
                "bottom_z": table.bottom_z,
                "top_z": table.bottom_z + table.dimensions[2],
            },
            "truth_check": truth_check,
            # This is intentionally diagnostic only.  Construction above
            # used bbox + cloud + TF, never the truth topic.
            "truth_used_for_construction": False,
        }
        _print_event({"event": "table_detection", **result})
        return table, result
    raise RuntimeError(
        "机検出/天板点群推定に失敗しました: "
        f"{last_error}; attempts={detection_attempts}"
    )


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
    enable_base: bool = False,
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
        enable_base=enable_base,
    )
    planner_status, constraint_status, planner_statuses, wait_sec = _wait_for_goal(
        capture, goal_id, timeout_sec
    )
    planner_failed = bool(
        any(status in FAILURE_STATUSES for status in planner_statuses)
        and RobotLocalPlannerStatus.SUCCESS not in planner_statuses
    )
    if planner_failed:
        # A deliberately unsafe collision probe is expected to fail before the
        # controller moves.  Do not spend the remaining motion timeout waiting
        # for a hand pose that must not be reached.
        physical = {
            "physical_converged": False,
            "physical_wait_skipped": True,
        }
    else:
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
            "orientation_quaternion": {
                "x": target.ori.x,
                "y": target.ori.y,
                "z": target.ori.z,
                "w": target.ori.w,
            },
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


def _move_rejected(result: dict[str, Any]) -> bool:
    """Return true when RLP rejected a target during planning/validation."""
    return bool(
        any(status in FAILURE_STATUSES for status in result.get("planner_statuses", []))
        and RobotLocalPlannerStatus.SUCCESS not in result.get("planner_statuses", [])
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


def _sim_object_samples(
    capture: Capture,
    object_path: str,
    attached: bool,
) -> list[tuple[float, tuple[float, float, float]]]:
    """Return simulator object positions for one grasp-state phase."""
    samples: list[tuple[float, tuple[float, float, float]]] = []
    for stamp, state in capture.sim_history_copy():
        if bool(state.get("attached", False)) != attached:
            continue
        for obj in state.get("objects", []):
            if obj.get("path") != object_path:
                continue
            position = obj.get("position")
            if isinstance(position, list) and len(position) == 3:
                samples.append(
                    (
                        stamp,
                        tuple(float(value) for value in position),
                    )
                )
    return samples


def _table_clearance_report(
    capture: Capture,
    table: Box,
    object_path: str,
    object_size: Any,
    attached: bool,
) -> dict[str, Any]:
    """Measure object/table AABB clearance from the simulator truth stream."""
    half_object = (
        max(0.005, float(object_size.x) * 0.5),
        max(0.005, float(object_size.y) * 0.5),
        max(0.005, float(object_size.z) * 0.5),
    )
    half_table = (
        table.dimensions[0] * 0.5,
        table.dimensions[1] * 0.5,
    )
    table_top = table.bottom_z + table.dimensions[2]
    # The YCB apple body path in SCENE=rlp uses a floor/contact reference at
    # its rigid-body origin.  Use a conservative physical height so an apple
    # below an elevated tabletop is not mistaken for a collision merely
    # because its origin z is lower than the tabletop top.
    physical_object_height = max(0.09, float(object_size.z))
    samples = _sim_object_samples(capture, object_path, attached)
    overlap_samples: list[dict[str, Any]] = []
    center_inside_samples = 0
    for stamp, position in samples:
        dx = abs(position[0] - table.center[0])
        dy = abs(position[1] - table.center[1])
        center_inside = dx <= half_table[0] and dy <= half_table[1]
        if center_inside:
            center_inside_samples += 1
        horizontal_overlap = (
            dx <= half_table[0] + half_object[0]
            and dy <= half_table[1] + half_object[1]
        )
        if not horizontal_overlap:
            continue
        object_bottom = position[2]
        object_top = position[2] + physical_object_height
        if object_top < table.bottom_z:
            clearance = table.bottom_z - object_top
        elif object_bottom > table_top:
            clearance = object_bottom - table_top
        else:
            clearance = -min(
                object_top - table.bottom_z,
                table_top - object_bottom,
            )
        estimated_shape_clearance = position[2] - half_object[2] - table_top
        overlap_samples.append(
            {
                "position": list(position),
                "vertical_clearance_m": clearance,
                # In the SCENE=rlp YCB placement, the rigid-body origin is
                # the model's floor/contact reference rather than its visual
                # center.  Keep this independent diagnostic for the physical
                # table support check.
                "origin_vertical_clearance_m": position[2] - table_top,
                "estimated_shape_vertical_clearance_m": estimated_shape_clearance,
                "center_inside_table": center_inside,
                "penetration_m": max(0.0, -clearance),
                "stamp_monotonic": stamp,
            }
        )

    latest = overlap_samples[-1] if overlap_samples else None
    min_clearance = (
        min(item["vertical_clearance_m"] for item in overlap_samples)
        if overlap_samples
        else None
    )
    min_origin_clearance = (
        min(item["origin_vertical_clearance_m"] for item in overlap_samples)
        if overlap_samples
        else None
    )
    return {
        "table_id": table.name,
        "table_top_z": table_top,
        "object_path": object_path,
        "object_half_size_xyz": list(half_object),
        "physical_object_height_m": physical_object_height,
        "physical_geometry_assumption": (
            "SCENE=rlp YCB apple body origin is treated as the lower/contact reference"
        ),
        "sample_count": len(samples),
        "horizontal_overlap_sample_count": len(overlap_samples),
        "center_inside_sample_count": center_inside_samples,
        "min_vertical_clearance_m": min_clearance,
        "max_penetration_m": (
            max(item["penetration_m"] for item in overlap_samples)
            if overlap_samples
            else None
        ),
        "min_origin_vertical_clearance_m": min_origin_clearance,
        "latest_origin_vertical_clearance_m": (
            latest["origin_vertical_clearance_m"] if latest is not None else None
        ),
        "origin_no_penetration": bool(
            overlap_samples
            and min_origin_clearance is not None
            and min_origin_clearance >= -0.005
        ),
        "latest_overlap": latest is not None,
        "latest_center_inside_table": bool(
            latest is not None and latest["center_inside_table"]
        ),
        "latest_position": latest["position"] if latest is not None else None,
        "latest_vertical_clearance_m": (
            latest["vertical_clearance_m"] if latest is not None else None
        ),
        "latest_estimated_shape_clearance_m": (
            latest["estimated_shape_vertical_clearance_m"]
            if latest is not None
            else None
        ),
        "no_penetration": bool(
            overlap_samples
            and min_clearance is not None
            and min_clearance >= -0.005
        ),
    }


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


def main(argv: list[str] | None = None) -> int:
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
    parser.add_argument(
        "--with-obstacle",
        action="store_true",
        help="also register the S4-style side box in RLP and Isaac Sim",
    )
    parser.add_argument(
        "--place-on-table",
        action="store_true",
        help="validate attached-object transfer, placement, and release on a tabletop",
    )
    parser.add_argument(
        "--detect-table",
        action="store_true",
        help="detect the placement table with GroundingDINO and PointCloud2",
    )
    parser.add_argument(
        "--cabinet-overhead",
        action="store_true",
        help="add a static overhead shelf and verify a safe under-shelf placement",
    )
    parser.add_argument(
        "--cabinet-place-under-shelf",
        action="store_true",
        help="align the measured tabletop patch and placement target under the shelf",
    )
    parser.add_argument(
        "--cabinet-side-insertion",
        action="store_true",
        help=(
            "place under the shelf by rotating the attached object to a "
            "horizontal, front-to-back insertion posture"
        ),
    )
    parser.add_argument(
        "--cabinet-side-place-lift",
        type=float,
        default=0.045,
        help=(
            "extra height retained while the sideways-attached object is "
            "inserted; it settles onto the tabletop after release"
        ),
    )
    parser.add_argument(
        "--table-only",
        action="store_true",
        help="stop after table detection/geometry validation (diagnostic mode)",
    )
    parser.add_argument(
        "--table-observation-pan",
        type=float,
        default=0.0,
        help="head pan used for the placement-table observation",
    )
    parser.add_argument(
        "--table-observation-tilt-deg",
        type=float,
        default=-50.0,
        help="head tilt in degrees used for the placement-table observation",
    )
    parser.add_argument("--table-prompt", default="table")
    parser.add_argument("--table-confidence", type=float, default=0.20)
    parser.add_argument("--table-iou", type=float, default=0.50)
    parser.add_argument("--table-max-distance", type=float, default=2.0)
    parser.add_argument(
        "--table-point-stride",
        type=int,
        default=3,
        help="pixel stride used when cropping the organized table cloud",
    )
    parser.add_argument(
        "--table-patch-size",
        type=float,
        default=0.20,
        help="side length of the measured tabletop placement patch in meters",
    )
    parser.add_argument("--table-center-x", type=float, default=0.82)
    parser.add_argument("--table-center-y", type=float, default=0.25)
    parser.add_argument("--table-bottom-z", type=float, default=0.30)
    parser.add_argument("--table-size-x", type=float, default=0.35)
    parser.add_argument("--table-size-y", type=float, default=0.30)
    parser.add_argument("--table-thickness", type=float, default=0.04)
    parser.add_argument("--cabinet-shelf-center-x", type=float, default=0.90)
    # Keep the shelf over the rear half of the tabletop.  This leaves the
    # front placement approach open while still making the overhead geometry
    # part of the attached-object planning scene.
    parser.add_argument("--cabinet-shelf-center-y", type=float, default=0.62)
    parser.add_argument("--cabinet-shelf-bottom-z", type=float, default=0.72)
    parser.add_argument("--cabinet-shelf-size-x", type=float, default=0.90)
    parser.add_argument("--cabinet-shelf-size-y", type=float, default=0.24)
    parser.add_argument("--cabinet-shelf-thickness", type=float, default=0.05)
    parser.add_argument(
        "--dynamic-bridge",
        action="store_true",
        help="run BridgeA online with the grasp/place sequence (S6.3b)",
    )
    parser.add_argument(
        "--dynamic-object-id",
        default="s6_3b_dynamic_obstacle",
        help="RLP CollisionObject id used by the S6.3b BridgeA instance",
    )
    parser.add_argument(
        "--dynamic-hide-timeout-sec",
        type=float,
        default=120.0,
        help="wait for the Sim dynamic obstacle to become hidden after release",
    )
    parser.add_argument(
        "--dynamic-stale-timeout-sec",
        type=float,
        default=1.0,
        help="BridgeA stale timeout used by the S6.3b online bridge",
    )
    parser.add_argument(
        "--place-offset-x",
        type=float,
        default=0.0,
        help="offset of the selected tabletop placement center in odom x",
    )
    parser.add_argument(
        "--place-offset-y",
        type=float,
        default=0.0,
        help="offset of the selected tabletop placement center in odom y",
    )
    parser.add_argument(
        "--place-clearance",
        type=float,
        default=0.10,
        help="clearance above the tabletop used before opening the gripper",
    )
    parser.add_argument("--place-high-offset", type=float, default=0.05)
    parser.add_argument(
        "--probe-penetration",
        type=float,
        default=0.02,
        help="deliberate object/table overlap used to verify Attached Object rejection",
    )
    parser.add_argument("--release-settle-sec", type=float, default=2.0)
    parser.add_argument("--timeout-sec", type=float, default=55.0)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.table_only:
        args.detect_table = True
        args.place_on_table = False

    rclpy.init()
    node = RobotLocalPlanner()
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    capture = Capture(node)
    frames = FramePublisher(node)
    cloud_capture = (
        PointCloudCapture(node, TABLE_POINT_CLOUD_TOPIC)
        if args.detect_table
        else None
    )
    truth_capture = TableTruthCapture(node) if args.detect_table else None
    cabinet_contact = (
        ContactCapture(node, CABINET_CONTACT_TOPIC)
        if args.cabinet_overhead
        else None
    )
    dynamic_truth = DynamicTruthCapture(node) if args.dynamic_bridge else None
    dynamic_bridge = (
        BridgeA(
            node,
            BridgeConfig(
                object_id=args.dynamic_object_id,
                # The grasp scene also contains the tabletop and apple.  The
                # negative-y crop and shape limits select only the moving box
                # without using its Sim truth stream.
                min_y=-1.00,
                max_y=-0.12,
                min_cluster_height=0.14,
                min_dimension_xy=0.16,
                min_dimension_z=0.20,
                max_dimension_xy=0.55,
                stale_timeout_sec=args.dynamic_stale_timeout_sec,
                # A 5 Hz CollisionObject stream keeps invalidating long
                # whole-body goals.  One update per second is still fast
                # relative to the moving-box validation scene while allowing
                # RLP to finish each grasp/place segment.
                update_period_sec=1.0,
            ),
        )
        if args.dynamic_bridge
        else None
    )
    environment_pub = node.create_publisher(CollisionObject, ENVIRONMENT_CONTROL_TOPIC, 10)
    obstacle_environment = (
        EnvironmentPublisher(node)
        if args.with_obstacle or args.place_on_table
        else None
    )
    attach_pub = node.create_publisher(AttachedCollisionObject, ATTACH_TOPIC, 10)
    release_pub = node.create_publisher(String, RELEASE_TOPIC, 10)
    detect_client = node.create_client(ObjectDetectionService, "/yolov8_detection/service")
    grasp_client = node.create_client(GraspPointService, "/grasp_point_detection/service")
    table_client = (
        node.create_client(Detection2DService, TABLE_DETECTION_SERVICE)
        if args.detect_table
        else None
    )
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    robot = None
    gripper = None
    whole_body = None
    attached_in_planner = False
    object_msg: CollisionObject | None = None
    obstacle_box: Box | None = None
    table_box: Box | None = None
    table_collision_msg: CollisionObject | None = None
    table_in_planner = False
    cabinet_box: Box | None = None
    cabinet_collision_msg: CollisionObject | None = None
    cabinet_in_planner = False
    environment_boxes: list[Box] = []
    static_sim_environment_ids: set[str] = set()
    environment_reference_pose: tuple[float, float, float] | None = None
    sim_object_path = ""
    dynamic_attached_wall = None
    stage_name = (
        "S6.3b"
        if args.dynamic_bridge
        else (
            "S6.3a-cabinet"
            if args.cabinet_overhead
            else ("S6.3" if args.detect_table else "S5")
        )
    )
    result: dict[str, Any] = {
        "stage": stage_name,
        "target": args.target,
        "object_id": args.object_id,
        "pass": False,
        "steps": {},
    }

    _print_event({
        "event": "start",
        "stage": stage_name,
        "target": args.target,
        "object_id": args.object_id,
        "detection_service": "/yolov8_detection/service",
        "grasp_service": "/grasp_point_detection/service",
        "tf_frames": ["s5_detected_object", "s5_grasp_target"],
        "attach_topic": ATTACH_TOPIC,
        "attached_topic": ATTACHED_TOPIC,
        "sim_grasp_topic": SIM_GRASP_TOPIC,
        "place_on_table": args.place_on_table,
        "detect_table": args.detect_table,
        "cabinet_overhead": args.cabinet_overhead,
        "cabinet_place_under_shelf": args.cabinet_place_under_shelf,
        "cabinet_side_insertion": args.cabinet_side_insertion,
        "dynamic_bridge": args.dynamic_bridge,
        "dynamic_object_id": args.dynamic_object_id if args.dynamic_bridge else None,
        "dynamic_truth_topic": (
            "/rlp_validation/dynamic_obstacle_truth"
            if args.dynamic_bridge
            else None
        ),
        "table_detection_service": (
            TABLE_DETECTION_SERVICE if args.detect_table else None
        ),
        "table_point_cloud_topic": (
            TABLE_POINT_CLOUD_TOPIC if args.detect_table else None
        ),
    })

    try:
        if not args.table_only and not detect_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("物体検出サービスが見つかりません")
        if not args.table_only and not grasp_client.wait_for_service(timeout_sec=15.0):
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

        if args.detect_table:
            if table_client is None or cloud_capture is None or truth_capture is None:
                raise RuntimeError("机検出用のROS interfaceが初期化されていません")
            table_observation_pose = dict(NEUTRAL_POSE)
            table_observation_pose["head_pan_joint"] = args.table_observation_pan
            table_observation_pose["head_tilt_joint"] = math.radians(
                args.table_observation_tilt_deg
            )
            whole_body.move_to_joint_positions(table_observation_pose, sync=True)
            result["steps"]["table_observation_pose"] = {
                "controller": "hsrb_interface",
                "targets": table_observation_pose,
                **_wait_for_joints(node, table_observation_pose, 5.0),
            }
            if not result["steps"]["table_observation_pose"].get(
                "physical_converged", False
            ):
                raise RuntimeError("机観察姿勢への移動に失敗しました")
            time.sleep(1.0)
            table_box, result["table_detection"] = _detect_table_geometry(
                node,
                table_client,
                cloud_capture,
                truth_capture,
                args,
            )
            result["table"] = result["table_detection"]["table_box"]
            if table_box is not None and args.place_on_table:
                # The procedural table is already a physical Sim collider.
                # Register only the measured tabletop patch in RLP, avoiding
                # a duplicate physical obstacle that can contact the base.
                table_collision_msg = EnvironmentPublisher._to_collision_object(
                    table_box
                )
            if args.table_only:
                truth_check = result["table_detection"].get("truth_check", {})
                result["pass"] = bool(
                    table_box is not None
                    and truth_check.get("available", False)
                    and truth_check.get("pass", False)
                )
                result["reason"] = "table_detection_geometry_only"
                return 0 if result["pass"] else 1

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

        if args.dynamic_bridge:
            if dynamic_bridge is None or dynamic_truth is None:
                raise RuntimeError("S6.3b BridgeA interfaceが初期化されていません")
            dynamic_cloud_connected = dynamic_bridge.wait_for_cloud(10.0)
            dynamic_truth_visible = dynamic_truth.wait_for(
                lambda item: bool(item.get("visible")),
                12.0,
            )
            dynamic_detected = _wait_for_dynamic_detection(dynamic_bridge, 12.0)
            dynamic_initial_truth = (
                _dynamic_closest_truth(dynamic_truth, dynamic_detected)
                if dynamic_detected is not None
                else None
            )
            dynamic_initial_error = (
                _dynamic_center_error(dynamic_detected, dynamic_initial_truth)
                if dynamic_detected is not None
                else None
            )
            result["dynamic_obstacle"] = {
                "object_id": args.dynamic_object_id,
                "point_cloud_topic": TABLE_POINT_CLOUD_TOPIC,
                "truth_topic": "/rlp_validation/dynamic_obstacle_truth",
                "cloud_connected": dynamic_cloud_connected,
                "truth_visible": dynamic_truth_visible is not None,
                "initial_detected": dynamic_detected is not None,
                "initial_center_error_m": dynamic_initial_error,
                "detected_center_xy": (
                    list(dynamic_detected.center_xy)
                    if dynamic_detected is not None
                    else None
                ),
                "truth_center_xy": (
                    list(dynamic_initial_truth.get("center_world", [])[:2])
                    if dynamic_initial_truth is not None
                    else None
                ),
            }
            _print_event({
                "event": "dynamic_snapshot_check",
                **result["dynamic_obstacle"],
            })
            if not dynamic_cloud_connected:
                raise RuntimeError("S6.3bの点群入力を受信できませんでした")
            if dynamic_truth_visible is None:
                raise RuntimeError("S6.3bの動的障害物truthがvisibleになりませんでした")
            if dynamic_detected is None:
                raise RuntimeError("S6.3bの動的障害物をBridgeAが検出できませんでした")
            if dynamic_initial_error is None or dynamic_initial_error > 0.20:
                raise RuntimeError(
                    "S6.3bのBridgeA検出中心誤差が大きすぎます: "
                    f"{dynamic_initial_error} m"
                )

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

        if obstacle_environment is not None:
            base_tf = _lookup_transform(node, ODOM_FRAME, BASE_FRAME)
            base_pose = (
                base_tf.translation.x,
                base_tf.translation.y,
                _yaw_from_quaternion(base_tf.rotation),
            )
            environment_reference_pose = base_pose
            if args.with_obstacle:
                # Keep the low box beside the arm's vertical carry path.  This
                # is the S5.2 integration slice: the same obstacle exists in
                # the RLP PlanningScene and as a PhysX collider while the
                # object changes from free to attached and back.
                obstacle_box = Box(
                    "s5_side_obstacle",
                    _local_xy_to_odom(base_pose, 0.55, 0.25),
                    (0.30, 0.40, 0.30),
                )
                environment_boxes.append(obstacle_box)
                result["obstacle"] = {
                    "id": obstacle_box.name,
                    "frame": ODOM_FRAME,
                    "center_xy": list(obstacle_box.center),
                    "dimensions_xyz": list(obstacle_box.dimensions),
                    "bottom_z": obstacle_box.bottom_z,
                }
            if args.place_on_table:
                if table_box is None:
                    # S5 compatibility path: an elevated tabletop collision
                    # slab supplied by fixed command-line geometry.
                    table_box = Box(
                        "s5_tabletop",
                        _local_xy_to_odom(
                            base_pose,
                            args.table_center_x,
                            args.table_center_y,
                        ),
                        (args.table_size_x,
                         args.table_size_y,
                         args.table_thickness),
                        bottom_z=args.table_bottom_z,
                    )
                    result["table"] = {
                        "id": table_box.name,
                        "frame": ODOM_FRAME,
                        "center_xy": list(table_box.center),
                        "dimensions_xyz": list(table_box.dimensions),
                        "bottom_z": table_box.bottom_z,
                        "top_z": table_box.bottom_z + table_box.dimensions[2],
                    }
                if table_collision_msg is None:
                    table_collision_msg = EnvironmentPublisher._to_collision_object(
                        table_box
                    )
                if args.detect_table:
                    static_sim_environment_ids.add(table_box.name)
                if args.cabinet_overhead:
                    cabinet_box = Box(
                        "s6_3_cabinet_overhead_shelf",
                        (
                            args.cabinet_shelf_center_x,
                            args.cabinet_shelf_center_y,
                        ),
                        (
                            args.cabinet_shelf_size_x,
                            args.cabinet_shelf_size_y,
                            args.cabinet_shelf_thickness,
                        ),
                        bottom_z=args.cabinet_shelf_bottom_z,
                    )
                    cabinet_collision_msg = (
                        EnvironmentPublisher._to_collision_object(cabinet_box)
                    )
                    static_sim_environment_ids.add(cabinet_box.name)
                    result["cabinet"] = {
                        "id": cabinet_box.name,
                        "frame": ODOM_FRAME,
                        "center_xy": list(cabinet_box.center),
                        "dimensions_xyz": list(cabinet_box.dimensions),
                        "bottom_z": cabinet_box.bottom_z,
                        "top_z": cabinet_box.bottom_z + cabinet_box.dimensions[2],
                    }
                    if args.cabinet_place_under_shelf:
                        # The cloud estimator gives a reliable tabletop
                        # height and observed patch size, but the camera sees
                        # only the front part of this large tabletop.  For
                        # this cabinet regression, use the explicitly
                        # configured shelf projection as the placement
                        # waypoint and move the measured patch there.  This
                        # keeps the tabletop height/size perception-derived
                        # while making the actual target lie under the shelf.
                        observed_table_box = table_box
                        result["table_detection"]["detected_table_box"] = {
                            "id": observed_table_box.name,
                            "center_xy": list(observed_table_box.center),
                            "dimensions_xyz": list(observed_table_box.dimensions),
                            "bottom_z": observed_table_box.bottom_z,
                            "top_z": observed_table_box.bottom_z
                            + observed_table_box.dimensions[2],
                        }
                        table_box = Box(
                            observed_table_box.name,
                            cabinet_box.center,
                            observed_table_box.dimensions,
                            bottom_z=observed_table_box.bottom_z,
                        )
                        table_collision_msg = (
                            EnvironmentPublisher._to_collision_object(table_box)
                        )
                        result["table_detection"]["table_box"] = {
                            "id": table_box.name,
                            "center_xy": list(table_box.center),
                            "dimensions_xyz": list(table_box.dimensions),
                            "bottom_z": table_box.bottom_z,
                            "top_z": table_box.bottom_z + table_box.dimensions[2],
                        }
                        result["table_detection"][
                            "placement_patch_policy"
                        ] = "measured_plane_shelf_projection"
                        result["table_detection"][
                            "placement_target_xy"
                        ] = list(table_box.center)
                        result["table"] = result["table_detection"]["table_box"]
                    environment_boxes.append(cabinet_box)
                environment_boxes.append(table_box)
            result["environment_boxes"] = [
                {
                    "id": box.name,
                    "center_xy": list(box.center),
                    "dimensions_xyz": list(box.dimensions),
                    "bottom_z": box.bottom_z,
                }
                for box in environment_boxes
            ]
            if not obstacle_environment.wait_for_subscriber():
                raise RuntimeError("S4/S5物理環境bridgeのsubscriberがありません")
            # Keep the detected tabletop out of the physical bridge while the
            # robot is moving to the grasp pose.  In S6.3a the procedural Sim
            # table is already a physical collider; the measured tabletop
            # patch is added to RLP only after attached-object retreat below.
            initial_environment_boxes = tuple(
                box for box in environment_boxes
                if box is not table_box
                and box.name not in static_sim_environment_ids
            )
            result["steps"]["environment_add"] = (
                obstacle_environment.publish_boxes(
                    initial_environment_boxes,
                    reference_pose=base_pose,
                )
            )
            result["steps"]["environment_obstacle_add"] = result["steps"]["environment_add"]
            result["steps"]["environment_table_add"] = (
                not args.place_on_table
                and result["steps"]["environment_add"]
            )
            if not result["steps"]["environment_add"]:
                raise RuntimeError("S5物理環境の登録を確認できませんでした")

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
            node,
            capture,
            "pregrasp",
            pregrasp_odom,
            args.timeout_sec,
            reference_frame=ODOM_FRAME,
            enable_base=True,
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
            node,
            capture,
            "grasp_pose",
            target_odom,
            args.timeout_sec,
            reference_frame=ODOM_FRAME,
            enable_base=True,
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
        sim_state = capture.latest_sim_state() or {}
        sim_object_path = str(sim_state.get("object_path", ""))
        result["sim_object_path"] = sim_object_path
        if not sim_object_path:
            raise RuntimeError("Simの把持対象object_pathを取得できませんでした")
        sim_hand_position = sim_state.get("hand_position")
        sim_object_position = next(
            (
                item.get("position")
                for item in sim_state.get("objects", [])
                if item.get("path") == sim_object_path
            ),
        )
        if (
            isinstance(sim_hand_position, list)
            and len(sim_hand_position) == 3
            and isinstance(sim_object_position, list)
            and len(sim_object_position) == 3
        ):
            result["sim_object_to_hand_offset_xyz"] = [
                float(sim_object_position[index]) - float(sim_hand_position[index])
                for index in range(3)
            ]

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
        if args.dynamic_bridge:
            dynamic_attached_wall = time.monotonic()
            result.setdefault("dynamic_obstacle", {})[
                "attached_history_start_wall"
            ] = dynamic_attached_wall

        retreat_odom = geometry.Pose(
            geometry.Vector3(
                target_odom.pos.x,
                target_odom.pos.y,
                target_odom.pos.z + 0.25,
            ),
            target_odom.ori,
        )
        result["steps"]["retreat_with_attached_object"] = _move_pose(
            node,
            capture,
            "retreat_with_attached_object",
            retreat_odom,
            args.timeout_sec,
            reference_frame=ODOM_FRAME,
            enable_base=True,
        )
        if not _move_passed(result["steps"]["retreat_with_attached_object"]):
            raise RuntimeError("Attached Objectを保持した退避動作に失敗しました")
        time.sleep(1.0)
        result["physical_carried_motion_m"] = _sim_carried_motion(capture, attached=True)

        if args.place_on_table:
            if table_box is None:
                raise RuntimeError("table_boxが作成されていません")
            if obstacle_environment is None or environment_reference_pose is None:
                raise RuntimeError("机上配置用の物理環境bridgeが初期化されていません")

            if args.detect_table:
                if table_collision_msg is None:
                    raise RuntimeError("table_collision_msgが作成されていません")

                # The procedural table is already a physical Sim collider.
                # Add only the measured tabletop patch to RLP after retreat;
                # sending it through EnvironmentPublisher as well would
                # create a second physical table and a false contact.
                result["steps"]["environment_table_add"] = _publish_environment(
                    environment_pub,
                    capture,
                    table_collision_msg,
                    present=True,
                )
                table_in_planner = result["steps"]["environment_table_add"]
            else:
                # Preserve the original S5 fixed-table compatibility path:
                # that table exists only when the physical bridge registers
                # it after the attached-object retreat.
                result["steps"]["environment_table_add"] = (
                    obstacle_environment.publish_boxes(
                        tuple(environment_boxes),
                        reference_pose=environment_reference_pose,
                    )
                )
            if not result["steps"]["environment_table_add"]:
                raise RuntimeError("机上配置前のtable登録を確認できませんでした")

            if args.cabinet_overhead:
                if cabinet_collision_msg is None or cabinet_box is None:
                    raise RuntimeError("cabinet_collision_msgが作成されていません")
                result["steps"]["environment_cabinet_add"] = _publish_environment(
                    environment_pub,
                    capture,
                    cabinet_collision_msg,
                    present=True,
                )
                cabinet_in_planner = result["steps"]["environment_cabinet_add"]
                if not result["steps"]["environment_cabinet_add"]:
                    raise RuntimeError("キャビネット上棚の登録を確認できませんでした")

            planner_object_to_hand_z = (
                grasp_odom.pose.position.z - target_odom.pos.z
            )
            raw_object_to_hand_offset = result.get(
                "sim_object_to_hand_offset_xyz"
            )
            if (
                isinstance(raw_object_to_hand_offset, list)
                and len(raw_object_to_hand_offset) == 3
                and all(value is not None for value in raw_object_to_hand_offset)
            ):
                object_to_hand_xyz = tuple(
                    float(value) for value in raw_object_to_hand_offset
                )
                object_to_hand_z = object_to_hand_xyz[2]
            else:
                object_to_hand_xyz = (0.0, 0.0, planner_object_to_hand_z)
                object_to_hand_z = planner_object_to_hand_z
            object_half_z = max(
                0.005,
                float(grasp_response.grasp.size.z) * 0.5,
            )
            object_half_x = max(
                0.005,
                float(grasp_response.grasp.size.x) * 0.5,
            )
            object_half_y = max(
                0.005,
                float(grasp_response.grasp.size.y) * 0.5,
            )
            table_top_z = table_box.bottom_z + table_box.dimensions[2]
            place_center = (
                table_box.center[0] + args.place_offset_x,
                table_box.center[1] + args.place_offset_y,
            )
            place_object_z = (
                table_top_z
                + object_half_z
                + args.place_clearance
            )
            place_hand_z = place_object_z - object_to_hand_z
            place_high_z = place_hand_z + args.place_high_offset
            place_pre_drop_z = place_hand_z + 0.10
            probe_object_bottom_z = table_top_z - args.probe_penetration
            placement_orientation = target_base.ori
            placement_mode = "top_down"
            placement_object_to_hand_xyz = object_to_hand_xyz
            placement_object_to_hand_z = object_to_hand_z
            cabinet_side_insertion = bool(
                args.cabinet_overhead
                and args.cabinet_place_under_shelf
                and args.cabinet_side_insertion
                and cabinet_box is not None
            )
            side_place_lift = (
                max(0.0, float(args.cabinet_side_place_lift))
                if cabinet_side_insertion
                else 0.0
            )
            if cabinet_side_insertion:
                placement_mode = "horizontal_side_insertion"
            probe_hand_z = (
                probe_object_bottom_z
                + object_half_z
                - placement_object_to_hand_z
            )
            place_high = geometry.Pose(
                geometry.Vector3(
                    place_center[0],
                    place_center[1],
                    place_high_z,
                ),
                target_base.ori,
            )
            place_pre_drop = geometry.Pose(
                geometry.Vector3(
                    place_center[0],
                    place_center[1],
                    place_pre_drop_z,
                ),
                placement_orientation,
            )
            probe_pose = geometry.Pose(
                geometry.Vector3(
                    place_center[0],
                    place_center[1],
                    probe_hand_z,
                ),
                placement_orientation,
            )
            place_pose = geometry.Pose(
                geometry.Vector3(
                    place_center[0],
                    place_center[1],
                    place_hand_z,
                ),
                placement_orientation,
            )
            if (
                args.cabinet_overhead
                and args.cabinet_place_under_shelf
                and cabinet_box is not None
            ):
                shelf_half_x = cabinet_box.dimensions[0] * 0.5
                shelf_half_y = cabinet_box.dimensions[1] * 0.5
                under_shelf_x = (
                    abs(place_center[0] - cabinet_box.center[0])
                    + object_half_x
                    <= shelf_half_x
                )
                under_shelf_y = (
                    abs(place_center[1] - cabinet_box.center[1])
                    + object_half_y
                    <= shelf_half_y
                )
                physical_object_height = max(
                    0.09,
                    float(grasp_response.grasp.size.z),
                )
                predicted_object_origin_z = place_object_z + side_place_lift
                predicted_object_top_z = (
                    predicted_object_origin_z + physical_object_height
                )
                shelf_vertical_clearance = (
                    cabinet_box.bottom_z - predicted_object_top_z
                )
                result["cabinet_under_shelf_check"] = {
                    "placement_center_xy_m": list(place_center),
                    "shelf_center_xy_m": list(cabinet_box.center),
                    "object_half_size_xy_m": [object_half_x, object_half_y],
                    "shelf_half_size_xy_m": [shelf_half_x, shelf_half_y],
                    "object_inside_shelf_projection": bool(
                        under_shelf_x and under_shelf_y
                    ),
                    "predicted_object_origin_z_m": predicted_object_origin_z,
                    "predicted_object_top_z_m": predicted_object_top_z,
                    "shelf_bottom_z_m": cabinet_box.bottom_z,
                    "predicted_vertical_clearance_m": shelf_vertical_clearance,
                    "placement_mode": placement_mode,
                }
                if not under_shelf_x or not under_shelf_y:
                    raise RuntimeError(
                        "配置目標が上棚の投影範囲から外れています"
                    )
                if shelf_vertical_clearance <= 0.0:
                    raise RuntimeError(
                        "配置物体の予測上端が上棚下面へ侵入します: "
                        f"{shelf_vertical_clearance:.3f} m"
                    )

            side_object_to_hand_xyz = None
            side_place_hand_z = None
            side_target_hand_xy = None
            side_front_hand_xy = None
            side_front_object_y = None
            if cabinet_side_insertion:
                # The grasp remains top-down.  Once the object is attached,
                # rotate the hand about the front of the shelf so the rigid
                # object-to-hand offset points toward +Y.  The object then
                # enters the shelf from the open/front (-Y) side while the
                # hand stays at the opening instead of lowering the object
                # vertically through the shelf.
                placement_mode = "horizontal_side_insertion"
                placement_orientation = geometry.pose(
                    ei=-math.pi / 2.0,
                ).ori
                # Keep the attached object clear of the tabletop while the
                # hand travels to the shelf front and changes orientation.
                # The same lift is used for the final sideways release pose;
                # after release the object settles onto the tabletop.
                place_high_z += side_place_lift
                side_object_to_hand_xyz = _rotate_offset_between_orientations(
                    object_to_hand_xyz,
                    target_base.ori,
                    placement_orientation,
                )
                placement_object_to_hand_xyz = side_object_to_hand_xyz
                placement_object_to_hand_z = side_object_to_hand_xyz[2]
                side_place_hand_z = (
                    place_object_z
                    - placement_object_to_hand_z
                    + side_place_lift
                )
                side_target_hand_xy = (
                    place_center[0] - side_object_to_hand_xyz[0],
                    place_center[1] - side_object_to_hand_xyz[1],
                )
                shelf_front_y = (
                    cabinet_box.center[1] - cabinet_box.dimensions[1] * 0.5
                )
                front_object_margin = max(
                    0.06,
                    object_half_y + 0.025,
                )
                side_front_object_y = shelf_front_y - front_object_margin
                side_front_hand_xy = (
                    place_center[0] - side_object_to_hand_xyz[0],
                    side_front_object_y - side_object_to_hand_xyz[1],
                )
                place_pre_drop_z = side_place_hand_z
                place_high = geometry.Pose(
                    geometry.Vector3(
                        side_front_hand_xy[0],
                        side_front_hand_xy[1],
                        place_high_z,
                    ),
                    placement_orientation,
                )
                place_pre_drop = geometry.Pose(
                    geometry.Vector3(
                        side_front_hand_xy[0],
                        side_front_hand_xy[1],
                        side_place_hand_z,
                    ),
                    placement_orientation,
                )
                place_pose = geometry.Pose(
                    geometry.Vector3(
                        side_target_hand_xy[0],
                        side_target_hand_xy[1],
                        side_place_hand_z,
                    ),
                    placement_orientation,
                )
                probe_hand_z = (
                    probe_object_bottom_z
                    + object_half_z
                    - placement_object_to_hand_z
                )
                probe_pose = geometry.Pose(
                    geometry.Vector3(
                        side_target_hand_xy[0],
                        side_target_hand_xy[1],
                        probe_hand_z,
                    ),
                    placement_orientation,
                )
            result["table_placement_geometry"] = {
                "planner_object_to_hand_z_m": planner_object_to_hand_z,
                "sim_object_to_hand_z_m": object_to_hand_z,
                "sim_object_to_hand_xyz_m": list(object_to_hand_xyz),
                "placement_object_to_hand_xyz_m": list(
                    placement_object_to_hand_xyz
                ),
                "estimated_object_half_z_m": object_half_z,
                "table_top_z_m": table_top_z,
                "placement_center_xy_m": list(place_center),
                "placement_offset_xy_m": [
                    args.place_offset_x,
                    args.place_offset_y,
                ],
                "place_object_z_m": place_object_z,
                "place_hand_z_m": place_pose.pos.z,
                "attached_place_object_z_m": (
                    place_object_z + side_place_lift
                ),
                "cabinet_side_place_lift_m": side_place_lift,
                "place_high_z_m": place_high_z,
                "place_pre_drop_z_m": place_pre_drop_z,
                "probe_hand_z_m": probe_hand_z,
                "probe_predicted_object_bottom_z_m": probe_object_bottom_z,
                "probe_predicted_penetration_m": args.probe_penetration,
                "placement_mode": placement_mode,
                "placement_orientation_quaternion": [
                    placement_orientation.x,
                    placement_orientation.y,
                    placement_orientation.z,
                    placement_orientation.w,
                ],
            }
            if cabinet_side_insertion:
                result["table_placement_geometry"].update({
                    "shelf_front_y_m": (
                        cabinet_box.center[1] - cabinet_box.dimensions[1] * 0.5
                    ),
                    "front_object_y_m": side_front_object_y,
                    "front_hand_xy_m": list(side_front_hand_xy),
                    "target_hand_xy_m": list(side_target_hand_xy),
                    "insertion_direction": "+Y",
                    "side_posture_roll_rad": -math.pi / 2.0,
                })
            carry_high = geometry.Pose(
                geometry.Vector3(
                    target_odom.pos.x,
                    target_odom.pos.y,
                    place_high_z,
                ),
                target_base.ori,
            )
            result["steps"]["carry_high_before_table"] = _move_pose(
                node,
                capture,
                "carry_high_before_table",
                carry_high,
                args.timeout_sec,
                reference_frame=ODOM_FRAME,
                enable_base=True,
            )
            if not _move_passed(result["steps"]["carry_high_before_table"]):
                raise RuntimeError("机へ水平移動する前の持ち上げに失敗しました")
            if cabinet_side_insertion:
                # Change to the side-insertion posture while the object is
                # still at the original, open location.  The following move
                # to the shelf front is then horizontal in this posture;
                # there is no top-down arm trajectory under the shelf.
                side_carry_orientation_pose = geometry.Pose(
                    geometry.Vector3(
                        target_odom.pos.x,
                        target_odom.pos.y,
                        place_high_z,
                    ),
                    placement_orientation,
                )
                result["steps"]["cabinet_side_carry_orientation"] = _move_pose(
                    node,
                    capture,
                    "cabinet_side_carry_orientation",
                    side_carry_orientation_pose,
                    args.timeout_sec,
                    reference_frame=ODOM_FRAME,
                    enable_base=True,
                )
                if not _move_passed(
                    result["steps"]["cabinet_side_carry_orientation"]
                ):
                    raise RuntimeError(
                        "棚から離れた位置での横向き搬送姿勢への変更に失敗しました"
                    )
            result["steps"]["place_high_with_attached_object"] = _move_pose(
                node,
                capture,
                "place_high_with_attached_object",
                place_high,
                args.timeout_sec,
                reference_frame=ODOM_FRAME,
                enable_base=True,
            )
            if not _move_passed(result["steps"]["place_high_with_attached_object"]):
                raise RuntimeError("Attached Objectを保持した机上高位置への移動に失敗しました")

            if args.cabinet_overhead and cabinet_box is not None:
                # First ask RLP to move the attached object into the overhead
                # shelf itself; this must be rejected before the actual
                # under-shelf placement continues.  In side-insertion mode
                # the probe uses the same horizontal posture and compensates
                # for the rotated object-to-hand offset.
                shelf_probe_object_z = cabinet_box.bottom_z + 0.02
                shelf_probe_hand_z = (
                    shelf_probe_object_z - placement_object_to_hand_z
                )
                if cabinet_side_insertion and side_target_hand_xy is not None:
                    shelf_probe_x, shelf_probe_y = side_target_hand_xy
                else:
                    shelf_probe_x, shelf_probe_y = (
                        cabinet_box.center[0],
                        cabinet_box.center[1],
                    )
                shelf_probe_pose = geometry.Pose(
                    geometry.Vector3(
                        shelf_probe_x,
                        shelf_probe_y,
                        shelf_probe_hand_z,
                    ),
                    placement_orientation,
                )
                result["cabinet_shelf_probe_geometry"] = {
                    "probe_hand_z_m": shelf_probe_hand_z,
                    "probe_object_origin_z_m": shelf_probe_object_z,
                    "predicted_shelf_penetration_m": 0.02,
                    "placement_mode": placement_mode,
                    "target_hand_xy_m": [shelf_probe_x, shelf_probe_y],
                }
                result["steps"]["cabinet_shelf_collision_probe"] = _move_pose(
                    node,
                    capture,
                    "cabinet_shelf_collision_probe",
                    shelf_probe_pose,
                    args.timeout_sec,
                    reference_frame=ODOM_FRAME,
                    enable_base=True,
                )
                result["steps"]["cabinet_shelf_probe_rejected"] = _move_rejected(
                    result["steps"]["cabinet_shelf_collision_probe"]
                )
                if not result["steps"]["cabinet_shelf_probe_rejected"]:
                    raise RuntimeError(
                        "Attached Objectをキャビネット上棚へ貫通させるprobeが拒否されませんでした"
                    )
                node.publish_empty_constraints()
                time.sleep(0.5)

            # The hand center is intentionally still above the tabletop, while
            # the attached object's bottom is inside the tabletop slab.  A
            # rejection here is the direct regression check that the planner
            # validates the attached shape, not only the robot links.
            result["steps"]["attached_collision_probe"] = _move_pose(
                node,
                capture,
                "attached_collision_probe",
                probe_pose,
                args.timeout_sec,
                reference_frame=ODOM_FRAME,
                enable_base=True,
            )
            result["steps"]["attached_collision_probe_rejected"] = _move_rejected(
                result["steps"]["attached_collision_probe"]
            )
            if not result["steps"]["attached_collision_probe_rejected"]:
                raise RuntimeError(
                    "Attached Objectを机へ貫通させるprobeがRLPで拒否されませんでした"
                )
            node.publish_empty_constraints()
            time.sleep(0.5)

            result["steps"]["place_pre_drop"] = _move_pose(
                node,
                capture,
                "place_pre_drop",
                place_pre_drop,
                args.timeout_sec,
                reference_frame=ODOM_FRAME,
                enable_base=True,
            )
            if not _move_passed(result["steps"]["place_pre_drop"]):
                if cabinet_side_insertion:
                    raise RuntimeError(
                        "棚前の水平挿入開始姿勢への移動に失敗しました"
                    )
                raise RuntimeError("机上の安全な下降前姿勢への移動に失敗しました")
            result["steps"]["place_pose_with_attached_object"] = _move_pose(
                node,
                capture,
                "place_pose_with_attached_object",
                place_pose,
                args.timeout_sec,
                reference_frame=ODOM_FRAME,
                enable_base=True,
            )
            if not _move_passed(result["steps"]["place_pose_with_attached_object"]):
                if cabinet_side_insertion:
                    raise RuntimeError(
                        "Attached Objectを保持した棚下の水平挿入に失敗しました"
                    )
                raise RuntimeError("Attached Objectを保持した机上置き姿勢への移動に失敗しました")
            time.sleep(0.8)
            result["steps"]["attached_table_clearance"] = _table_clearance_report(
                capture,
                table_box,
                sim_object_path,
                grasp_response.grasp.size,
                attached=True,
            )
            if not result["steps"]["attached_table_clearance"]["no_penetration"]:
                raise RuntimeError("把持中の物体が机上面へ侵入しました")

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
        if args.place_on_table and table_box is not None:
            time.sleep(args.release_settle_sec)
            released_clearance = _table_clearance_report(
                capture,
                table_box,
                sim_object_path,
                grasp_response.grasp.size,
                attached=False,
            )
            result["steps"]["released_table_clearance"] = released_clearance
            result["steps"]["placed_on_table"] = bool(
                released_clearance.get("latest_center_inside_table", False)
                and released_clearance.get("no_penetration", False)
                and released_clearance.get("latest_origin_vertical_clearance_m") is not None
                and released_clearance["latest_origin_vertical_clearance_m"] <= 0.12
            )
            if cabinet_in_planner and cabinet_collision_msg is not None:
                result["steps"]["environment_cabinet_remove"] = _publish_environment(
                    environment_pub,
                    capture,
                    cabinet_collision_msg,
                    present=False,
                )
                cabinet_in_planner = not result["steps"]["environment_cabinet_remove"]
            if table_in_planner and table_collision_msg is not None:
                result["steps"]["environment_table_remove"] = _publish_environment(
                    environment_pub,
                    capture,
                    table_collision_msg,
                    present=False,
                )
                table_in_planner = not result["steps"]["environment_table_remove"]
        if cabinet_contact is not None:
            result["steps"]["cabinet_physical_contact"] = cabinet_contact.seen()
        if obstacle_environment is not None:
            result["steps"]["physical_obstacle_contact"] = (
                obstacle_environment.physical_contact_seen()
            )
            result["steps"]["environment_obstacle_remove"] = (
                obstacle_environment.publish_boxes(())
            )

        if args.dynamic_bridge:
            if dynamic_bridge is None or dynamic_truth is None:
                raise RuntimeError("S6.3b BridgeA interfaceが失われました")
            history = dynamic_bridge.history()
            updates = [
                item for item in history
                if item.get("event") == "bridge_update"
            ]
            attached_updates = [
                item for item in updates
                if dynamic_attached_wall is not None
                and float(item.get("wall_time", 0.0)) >= dynamic_attached_wall
            ]
            detected_y = [
                float(item["center_xy"][1])
                for item in updates
                if isinstance(item.get("center_xy"), list)
                and len(item["center_xy"]) >= 2
            ]
            dynamic_y_delta = (
                max(detected_y) - min(detected_y)
                if len(detected_y) >= 2
                else 0.0
            )
            dynamic_truth_hidden = dynamic_truth.wait_for(
                lambda item: not bool(item.get("visible")),
                max(1.0, args.dynamic_hide_timeout_sec),
            )
            stale_deadline = time.monotonic() + max(
                3.0, args.dynamic_stale_timeout_sec + 2.0
            )
            dynamic_stale_removed = False
            while time.monotonic() < stale_deadline:
                dynamic_stale_removed = any(
                    item.get("event") == "bridge_remove"
                    and item.get("reason") == "pointcloud_stale"
                    for item in dynamic_bridge.history()
                )
                if dynamic_stale_removed:
                    break
                time.sleep(0.10)
            dynamic_physical_contact = dynamic_truth.contact_seen()
            result["dynamic_obstacle"].update({
                "updates_total": len(updates),
                "updates_while_attached": len(attached_updates),
                "motion_delta_y_m": dynamic_y_delta,
                "motion_updated": dynamic_y_delta >= 0.20 and len(updates) >= 2,
                "truth_hidden": dynamic_truth_hidden is not None,
                "bridge_removed_stale_object": dynamic_stale_removed,
                "physical_contact": dynamic_physical_contact,
            })
            _print_event({
                "event": "dynamic_stale_check",
                **result["dynamic_obstacle"],
            })

        dynamic_pass = bool(
            not args.dynamic_bridge
            or (
                result.get("dynamic_obstacle", {}).get("cloud_connected", False)
                and result.get("dynamic_obstacle", {}).get("truth_visible", False)
                and result.get("dynamic_obstacle", {}).get("initial_detected", False)
                and result.get("dynamic_obstacle", {}).get(
                    "initial_center_error_m"
                ) is not None
                and result["dynamic_obstacle"]["initial_center_error_m"] <= 0.20
                and result.get("dynamic_obstacle", {}).get("motion_updated", False)
                and result.get("dynamic_obstacle", {}).get(
                    "updates_while_attached", 0
                ) >= 1
                and result.get("dynamic_obstacle", {}).get("truth_hidden", False)
                and result.get("dynamic_obstacle", {}).get(
                    "bridge_removed_stale_object", False
                )
                and not result.get("dynamic_obstacle", {}).get(
                    "physical_contact", True
                )
            )
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
            and result["steps"].get("environment_obstacle_add", True)
            and not result["steps"].get("physical_obstacle_contact", False)
            and result["steps"].get("environment_obstacle_remove", True)
            and (
                not args.place_on_table
                or (
                    result["steps"].get("environment_table_add", False)
                    and _move_passed(
                        result["steps"].get("carry_high_before_table", {})
                    )
                    and _move_passed(
                        result["steps"].get("place_high_with_attached_object", {})
                    )
                    and (
                        not args.cabinet_side_insertion
                        or _move_passed(
                            result["steps"].get(
                                "cabinet_side_carry_orientation", {}
                            )
                        )
                    )
                    and result["steps"].get(
                        "attached_collision_probe_rejected", False
                    )
                    and _move_passed(result["steps"].get("place_pre_drop", {}))
                    and _move_passed(
                        result["steps"].get("place_pose_with_attached_object", {})
                    )
                    and result["steps"]
                    .get("attached_table_clearance", {})
                    .get("no_penetration", False)
                    and result["steps"].get("placed_on_table", False)
                    and result["steps"].get(
                        "environment_table_remove",
                        not args.detect_table,
                    )
                    and result["steps"].get(
                        "environment_cabinet_add",
                        not args.cabinet_overhead,
                    )
                    and result["steps"].get(
                        "environment_cabinet_remove",
                        not args.cabinet_overhead,
                    )
                    and not result["steps"].get("cabinet_physical_contact", False)
                    and (
                        not args.cabinet_overhead
                        or result["steps"].get("cabinet_shelf_probe_rejected", False)
                    )
                    and (
                        not args.cabinet_place_under_shelf
                        or result.get("cabinet_under_shelf_check", {}).get(
                            "object_inside_shelf_projection", False
                        )
                    )
                    and (
                        not args.cabinet_side_insertion
                        or result.get("table_placement_geometry", {}).get(
                            "placement_mode"
                        ) == "horizontal_side_insertion"
                    )
                )
            )
            and dynamic_pass
        )
        result["reason"] = (
            "recognition_tf_grasp_attach_retreat_release"
            if not args.place_on_table
            else (
                "recognition_tf_grasp_attach_cabinet_side_insert_release"
                if args.cabinet_side_insertion
                else "recognition_tf_grasp_attach_table_probe_place_release"
            )
        )
    except Exception as exc:  # keep JSONL diagnostics and clean up in finally
        result["error"] = str(exc)
        _print_event({"event": "error", "stage": stage_name, "error": str(exc)})
    finally:
        if dynamic_bridge is not None:
            try:
                dynamic_bridge.stop()
            except Exception:
                pass
        if cabinet_in_planner and cabinet_collision_msg is not None:
            try:
                _publish_environment(
                    environment_pub,
                    capture,
                    cabinet_collision_msg,
                    present=False,
                )
            except Exception:
                pass
        if table_in_planner and table_collision_msg is not None:
            try:
                _publish_environment(
                    environment_pub,
                    capture,
                    table_collision_msg,
                    present=False,
                )
            except Exception:
                pass
        if attached_in_planner:
            try:
                _publish_release(release_pub, args.object_id, repeat=4)
            except Exception:
                pass
        if gripper is not None:
            _gripper_command(gripper, 1.0)
        if obstacle_environment is not None:
            try:
                obstacle_environment.publish_boxes(())
            except Exception:
                pass
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
