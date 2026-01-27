#!/usr/bin/env python
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
from dataclasses import dataclass
import math
import threading
import time

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    Vector3,
)

from hsrb_rlp_interface_py import geometry

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tmc_planning_msgs.msg import (
    Constraints,
    ConstraintsStatus,
    LinearConstraint,
    LinearConstraintWithPose,
    RangeJointConstraint,
    RobotLocalGoal,
    RobotLocalPlannerStatus,
    TaskSpaceRegion,
    TsrLinkConstraint,
)


_END_EFFECTOR_FRAME_ID = 'hand_palm_link'


def _create_task_space_region(origin_to_tsr: Pose,
                              tsr_to_end: Pose,
                              min_bounds: tuple[float],
                              max_bounds: tuple[float],
                              rotation_first=False) -> TaskSpaceRegion:
    tsr = TaskSpaceRegion()
    tsr.origin_to_tsr = origin_to_tsr
    tsr.tsr_to_end = tsr_to_end
    tsr.min_bounds = min_bounds
    tsr.max_bounds = max_bounds
    tsr.end_frame_id = _END_EFFECTOR_FRAME_ID
    tsr.rotation_first = rotation_first
    return tsr


def _create_tsr_link_constraint(tsr: TaskSpaceRegion,
                                ref_frame_id: str = 'odom') -> TsrLinkConstraint:
    tlc = TsrLinkConstraint()
    tlc.tsr = tsr
    tlc.header.frame_id = ref_frame_id
    return tlc


def _create_constraints(hjc: list[RangeJointConstraint] = [],
                        hlc: list[TsrLinkConstraint] = [],
                        hplc: list[TsrLinkConstraint] = [],
                        imlc: LinearConstraintWithPose = LinearConstraintWithPose(),
                        grlc: LinearConstraint = LinearConstraint()) -> Constraints:
    return Constraints(hard_joint_constraints=hjc,
                       hard_link_constraints=hlc,
                       initial_motion_linear_constraint=imlc,
                       goal_relative_linear_constraint=grlc,
                       hard_path_link_constraints=hplc)


def _create_robot_local_goal(name: str,
                             constraints: Constraints,
                             normalized_velocity: float = 1.0,
                             enable_base: bool = True):
    return RobotLocalGoal(id=name,
                          constraints=constraints,
                          normalized_velocity=normalized_velocity,
                          enable_base=enable_base)


@dataclass
class PoseCommand:
    position: tuple[float]
    orientation: tuple[float]
    min_bounds: tuple[float]
    max_bounds: tuple[float]

    def to_tsr_link_constraint(self) -> TsrLinkConstraint:
        goal_pose = (geometry.Vector3(self.position[0], self.position[1], self.position[2]),
                     geometry.Quaternion(*self.orientation))

        # The hand position/orientation is represented by ref_to_tsr * transformation randomly sampled from bounds * tsr_to_end
        ref_to_tsr = geometry.tuples_to_pose(goal_pose)
        tsr_to_end = geometry.tuples_to_pose(geometry.pose())

        tsr = _create_task_space_region(ref_to_tsr, tsr_to_end, self.min_bounds, self.max_bounds)
        return _create_tsr_link_constraint(tsr)


@dataclass
class JointCommand:
    joint_names: tuple[str]
    joint_positions: tuple[float]
    odom_x: float
    odom_y: float
    odom_t: float

    def to_range_joint_constraint(self) -> RangeJointConstraint:
        rjc = RangeJointConstraint()
        rjc.header.frame_id = 'odom'
        rjc.min.joint_state.name = self.joint_names
        rjc.min.joint_state.position = self.joint_positions
        rjc.min.multi_dof_joint_state.joint_names = ['world_joint']
        q_z = math.sin(self.odom_t / 2.0)
        q_w = math.cos(self.odom_t / 2.0)
        base_goal = Transform(translation=Vector3(x=self.odom_x, y=self.odom_y, z=0.0),
                              rotation=Quaternion(x=0.0, y=0.0, z=q_z, w=q_w))
        rjc.min.multi_dof_joint_state.transforms = [base_goal]
        rjc.max = rjc.min
        return rjc


def create_linear_constraint_with_pose(end_frame_id: str,
                                       pose_stamped: PoseStamped,
                                       axis: tuple[float, float, float],
                                       distance: float) -> LinearConstraintWithPose:
    lcwp = LinearConstraintWithPose()
    lcwp.end_frame_id = end_frame_id
    lcwp.reference = pose_stamped
    lcwp.axis.x = axis[0]
    lcwp.axis.y = axis[1]
    lcwp.axis.z = axis[2]
    lcwp.distance = distance
    return lcwp


def create_linear_constraint(end_frame_id: str,
                             axis: tuple[float, float, float],
                             distance: float) -> LinearConstraint:
    lc = LinearConstraint()
    lc.end_frame_id = end_frame_id
    lc.axis.x = axis[0]
    lc.axis.y = axis[1]
    lc.axis.z = axis[2]
    lc.distance = distance
    return lc


class TfListenerWrapper:
    def __init__(self, node: rclpy.node.Node):
        self._clock = node.get_clock()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

    def _get_transform(self, target_frame: str, source_frame: str = 'odom') -> Transform:
        return self._tf_buffer.lookup_transform(source_frame,
                                                target_frame,
                                                self._clock.now(),
                                                rclpy.duration.Duration(seconds=1.0)).transform

    def _get_pose(self, target_frame: str, source_frame: str = 'odom') -> PoseStamped:
        transform = self._get_transform(target_frame, source_frame)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.pose.position.x = transform.translation.x
        pose_stamped.pose.position.y = transform.translation.y
        pose_stamped.pose.position.z = transform.translation.z
        pose_stamped.pose.orientation = transform.rotation
        return pose_stamped

    def get_hand_pose(self) -> PoseStamped:
        return self._get_pose(_END_EFFECTOR_FRAME_ID)


class PlannerStateSubscriber:
    def __init__(self, node: Node):
        self._status_msg = None
        self._status_sub = node.create_subscription(
            RobotLocalPlannerStatus, '/hsrb_robot_local_planner/planner_status',
            self._status_callback, qos_profile_sensor_data)

    def _status_callback(self, msg: RobotLocalPlannerStatus):
        self._status_msg = msg

    def is_constraint_received(self, name: str) -> bool:
        if self._status_msg is None:
            return False
        for status in self._status_msg.constraints_statuses:
            if status.id == name:
                return True
        return False

    def is_constraint_completed(self, name: str) -> bool:
        if self._status_msg is None:
            return False
        for status in self._status_msg.constraints_statuses:
            if status.id != name:
                continue
            if status.value in [ConstraintsStatus.SATISFIED, ConstraintsStatus.PREEMPTED]:
                return True
        return False


def generate_name(prefix: str, stamp: Time) -> str:
    sec, nanosec = stamp.seconds_nanoseconds()
    return f'{prefix}_{sec}.{nanosec:09}'


def send_goal_sync(goal: RobotLocalGoal,
                   constraints_pub: Publisher,
                   planner_state_sub: PlannerStateSubscriber):
    constraints_pub.publish(goal)
    while rclpy.ok():
        if planner_state_sub.is_constraint_received(goal.id):
            break
        time.sleep(0.1)
    while rclpy.ok():
        if planner_state_sub.is_constraint_completed(goal.id):
            break
        time.sleep(0.1)
    # Pause for human verification
    time.sleep(1.0)


def move_to_joint_positions(name: str,
                            joint_command: JointCommand,
                            constraints_pub: rclpy.publisher.Publisher,
                            planner_state_sub: PlannerStateSubscriber,
                            clock: rclpy.clock.Clock):
    goal_name = generate_name(name, clock.now())
    goal = _create_robot_local_goal(goal_name,
                                    _create_constraints(hjc=[joint_command.to_range_joint_constraint()]))
    send_goal_sync(goal, constraints_pub, planner_state_sub)


def move_to_end_effector_pose(name: str,
                              pose_command: list[PoseCommand],
                              constraints_pub: rclpy.publisher.Publisher,
                              planner_state_sub: PlannerStateSubscriber,
                              clock: rclpy.clock.Clock,
                              hplc: list[PoseCommand] = [],
                              imlc: LinearConstraintWithPose = LinearConstraintWithPose(),
                              grlc: LinearConstraint = LinearConstraint()):
    goal_name = generate_name(name, clock.now())
    goal = _create_robot_local_goal(goal_name,
                                    _create_constraints(hlc=[pc.to_tsr_link_constraint() for pc in pose_command],
                                                        hplc=[pc.to_tsr_link_constraint() for pc in hplc],
                                                        imlc=imlc, grlc=grlc))
    send_goal_sync(goal, constraints_pub, planner_state_sub)


def main():
    rclpy.init()
    node = rclpy.create_node('rlp_evaluation')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    tf_listener_wrapper = TfListenerWrapper(node)

    planner_state_sub = PlannerStateSubscriber(node)
    constraints_pub = node.create_publisher(RobotLocalGoal, 'hsrb_robot_local_planner/constraints', 1)
    while constraints_pub.get_subscription_count() == 0:
        time.sleep(0.1)

    try:
        _ZERO_BOUNDS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        print('Sending a joint command (Neutral)')

        neutral_pose_command = JointCommand(
            ('arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'),
            (0.0, 0.0, -1.57, -1.57, 0.0), 0.0, 0.0, 0.0)
        move_to_joint_positions('move_to_neutral_1', neutral_pose_command,
                                constraints_pub, planner_state_sub, node.get_clock())

        print('Sending a pose command  (No constraints)')

        pose_command = PoseCommand((1.0, 0.5, 1.0), (0.707, 0.0, 0.707, 0.0), _ZERO_BOUNDS, _ZERO_BOUNDS)
        move_to_end_effector_pose('move_to_hand_goal', [pose_command],
                                  constraints_pub, planner_state_sub, node.get_clock())

        print('Sending a joint command (Moving)')

        go_pose_command = JointCommand(
            ('arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint'),
            (0.0, 0.0, 0.0, -1.57, 0.0), 0.0, 0.0, 0.0)
        move_to_joint_positions('move_to_go_1', go_pose_command,
                                constraints_pub, planner_state_sub, node.get_clock())

        print('Sending a pose command  (Initial motion linear constraints)')

        current_hand_pose = tf_listener_wrapper.get_hand_pose()
        # Since it is an axis based on hand_pose, it is (0, 0, 1)
        imlc = create_linear_constraint_with_pose(_END_EFFECTOR_FRAME_ID, current_hand_pose, (0.0, 0.0, 1.0), 0.3)
        move_to_end_effector_pose('move_with_initial_motion_linear_constraint', [pose_command],
                                  constraints_pub, planner_state_sub, node.get_clock(),
                                  imlc=imlc)

        print('Sending a joint command (Neutral)')

        move_to_joint_positions('move_to_neutral_2', neutral_pose_command,
                                constraints_pub, planner_state_sub, node.get_clock())

        print('Sending a pose command  (Goal relative linear constraints)')

        # Since it is an axis based on goal orientation, it is (0, 0, -1)
        grlc = create_linear_constraint(_END_EFFECTOR_FRAME_ID, (0.0, 0.0, -1.0), 0.3)
        move_to_end_effector_pose('move_with_goal_relative_linear_constraint', [pose_command],
                                  constraints_pub, planner_state_sub, node.get_clock(),
                                  grlc=grlc)

        print('Sending a joint command (Moving)')

        move_to_joint_positions('move_to_go_2', go_pose_command,
                                constraints_pub, planner_state_sub, node.get_clock())

        print('Sending a pose command  (Path link constraints)')
        # Here, the position/orientation is the same as odom, so the coordinate system of bounds is the same as odom
        path_constraint_command = PoseCommand((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                                              (-1.0e10, -1.0e10, 1.0, -math.pi, -math.pi, -math.pi),
                                              (1.0e10, 1.0e10, 1.0e10, math.pi, math.pi, math.pi))
        move_to_end_effector_pose('move_with_path_link_constraint', [pose_command],
                                  constraints_pub, planner_state_sub, node.get_clock(),
                                  hplc=[path_constraint_command])
    finally:
        executor.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
