#!/usr/bin/env python
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
# -*- coding: utf-8 -*-

from geometry_msgs.msg import (
    Quaternion,
    Transform,
    Vector3,
)

from hsrb_rlp_interface_py.geometry import (
    pose,
    transform_to_tuples,
    tuples_to_pose,
)
from hsrb_rlp_interface_py.subscriber import JointStateSubscriber

import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros

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


END_EFFECTOR_FRAME = "hand_palm_link"
MOTION_PLANNING_JOINTS = ["wrist_flex_joint", "wrist_roll_joint",
                          "arm_roll_joint", "arm_flex_joint",
                          "arm_lift_joint", "hand_motor_joint",
                          "head_pan_joint", "head_tilt_joint"]
TF_TIMEOUT = 5.0
BASE = 'base_footprint'
ODOM = 'odom'
WAIT_FOR_COMPLETE_TIMEOUT = 20.0


def _create_robot_local_goal(constraints,
                             normalized_velocity=0.5,
                             enable_base=True):
    return RobotLocalGoal(constraints=constraints,
                          normalized_velocity=normalized_velocity,
                          enable_base=enable_base)


def _create_constraints(
    hjc=[],
    hlc=[],
    sjc=[],
    slc=[],
    grlc=LinearConstraint(),
    imlc=LinearConstraintWithPose(),
    hplc=[],
    spjc=[]
):
    return Constraints(hard_joint_constraints=hjc,
                       hard_link_constraints=hlc,
                       soft_joint_constraints=sjc,
                       soft_link_constraints=slc,
                       goal_relative_linear_constraint=grlc,
                       initial_motion_linear_constraint=imlc,
                       hard_path_link_constraints=hplc,
                       soft_path_joint_constraints=spjc)


def _lookup_odom_to_ref(tf_buffer, ref_frame_id, stamp):
    odom_to_ref_ros = tf_buffer.lookup_transform(
        ODOM,
        ref_frame_id,
        stamp,
        rclpy.duration.Duration(seconds=TF_TIMEOUT)).transform
    odom_to_ref_tuples = transform_to_tuples(odom_to_ref_ros)
    return tuples_to_pose(odom_to_ref_tuples)


class RobotLocalPlanner(Node):
    def __init__(self, loop_hz=5.0):
        super().__init__('robot_local_planner_commander')

        self._status = RobotLocalPlannerStatus()

        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer, self)

        self._end_effector_frame = END_EFFECTOR_FRAME
        self._motion_planning_joints = MOTION_PLANNING_JOINTS
        self._has_wait_complete = False
        self._rate = self.create_rate(10.0)
        self._pub = self.create_publisher(
            RobotLocalGoal, 'hsrb_robot_local_planner/constraints', 1)
        self._joint_state_sub = JointStateSubscriber(self, 'joint_states')

        self._status_sub = self.create_subscription(
            RobotLocalPlannerStatus, 'hsrb_robot_local_planner/planner_status', self._status_callback,
            qos_profile_sensor_data)

    # def __enter__(self):
    #     return self

    # def __exit__(self, exc_type, exc_val, exc_tb):
    #     self._tf2_buffer = None
    #     self._tf2_listener = None
    #     rospy.signal_shutdown('shutdown')

    def move_base_relative(self, x=0.0, y=0.0, yaw=0.0):
        return self.move_base_any_frame(x, y, yaw, BASE)

    def move_base_absolute(self, x=0.0, y=0.0, yaw=0.0):
        return self.move_base_any_frame(x, y, yaw, 'map')

    def move_base_any_frame(self, x=0.0, y=0.0, yaw=0.0, frame_id=None):
        if frame_id is None:
            self.get_logger().warn('frame_id is not set.')
            return
        joint_states = self._joint_state_sub.get_joint_state(MOTION_PLANNING_JOINTS)
        rjc = self._create_joint_constraint(joint_states, pose(x, y, 0.0, 0.0, 0.0, yaw), frame_id)
        constraint = _create_constraints(hjc=[rjc])
        goal = _create_robot_local_goal(constraint)
        return self.publish(goal)

    def move_to_joint_positions_with_base_relative(self, joint_goals={},
                                                   x=0.0, y=0.0, yaw=0.0, normalized_velocity=0.5):
        return self.move_to_joint_positions_with_base_any_frame(joint_goals, x, y, yaw, BASE, normalized_velocity)

    def move_to_joint_positions_with_base_absolute(self, joint_goals={},
                                                   x=0.0, y=0.0, yaw=0.0, normalized_velocity=0.5):
        return self.move_to_joint_positions_with_base_any_frame(joint_goals, x, y, yaw, 'map', normalized_velocity)

    def move_to_joint_positions_with_base_any_frame(self, joint_goals={},
                                                    x=0.0, y=0.0, yaw=0.0, frame_id=None, normalized_velocity=0.5):
        if frame_id is None:
            self.get_logger().warn('frame_id is not set.')
            return
        if not joint_goals:
            return
        rjc = self._create_joint_constraint(joint_goals, pose(x, y, 0.0, 0.0, 0.0, yaw), frame_id)
        constraint = _create_constraints(hjc=[rjc])
        goal = _create_robot_local_goal(constraint, normalized_velocity)
        return self.publish(goal)

    def move_to_joint_positions(self, goals={}, normalized_velocity=0.5):
        if not goals:
            return
        rjc = self._create_joint_constraint(joint_goals=goals)
        constraint = _create_constraints(hjc=[rjc])
        goal = _create_robot_local_goal(constraint, normalized_velocity, enable_base=False)
        return self.publish(goal)

    def move_to_neutral(self):
        goals = {
            'arm_lift_joint': 0.0,
            'arm_flex_joint': 0.0,
            'arm_roll_joint': 0.0,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0,
        }
        return self.move_to_joint_positions(goals)

    def move_to_go(self):
        goals = {
            'arm_flex_joint': 0.0,
            'arm_lift_joint': 0.0,
            'arm_roll_joint': -1.57,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': 0.0
        }
        return self.move_to_joint_positions(goals)

    def move_end_effector_pose(self, pose, ref_frame_id=None,
                               min_bounds=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               max_bounds=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               normalized_velocity=0.5):
        if ref_frame_id is None:
            ref_frame_id = BASE

        odom_to_ref_pose = _lookup_odom_to_ref(self._tf2_buffer, ref_frame_id, self.get_clock().now())
        ref_to_hand_pose = tuples_to_pose(pose)

        tsr = self._create_task_space_region(odom_to_ref_pose,
                                             ref_to_hand_pose,
                                             min_bounds=min_bounds,
                                             max_bounds=max_bounds)
        tlc = self._create_tsr_link_constraint(tsr)
        constraints = _create_constraints(hlc=[tlc])
        goal = _create_robot_local_goal(constraints, normalized_velocity)
        return self.publish(goal)

    def publish_empty_constraints(self):
        constraint = _create_constraints()
        goal = _create_robot_local_goal(constraint)
        # Do not return goal_id, as there is no point in waiting for goal_id
        self.publish(goal)

    def publish(self, goal):
        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        goal.id = f'hsrb_rlp_interface_py_{sec}.{nanosec:09}'
        self._pub.publish(goal)
        self._done = False

        if self.has_wait_complete:
            return self.wait_for_complete(goal.id)
        else:
            return goal.id

    def _create_tsr_link_constraint(self, tsr, ref_frame_id=ODOM):
        tlc = TsrLinkConstraint()
        tlc.tsr = tsr
        tlc.header.frame_id = ref_frame_id
        return tlc

    def _create_joint_constraint(self, joint_goals={}, base_goal=None, ref_frame_id='map'):
        rjc = RangeJointConstraint()
        rjc.header.frame_id = ref_frame_id
        for k, v in joint_goals.items():
            rjc.min.joint_state.name.append(k)
            rjc.min.joint_state.position.append(v)
            # if k in self._motion_planning_joints:
            #     rjc.min.joint_state.name.append(k)
            #     rjc.min.joint_state.position.append(v)
            # else:
            #     msg = "`joint_name` must be one of motion planning joints({0})"
            #     raise ValueError(msg.format(self._motion_planning_joints))

        trans = Vector3()
        rot = Quaternion()
        rot.w = 1.0
        if base_goal is not None:
            trans, rot = base_goal
        base_to_target_position = Transform(
            translation=Vector3(x=trans.x, y=trans.y, z=trans.z),
            rotation=Quaternion(x=rot.x, y=rot.y, z=rot.z, w=rot.w))
        rjc.min.multi_dof_joint_state.transforms.append(base_to_target_position)
        rjc.min.multi_dof_joint_state.joint_names.append('world_joint')
        rjc.max = rjc.min
        return rjc

    def _create_task_space_region(self, origin_to_tsr, tsr_to_end,
                                  min_bounds=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  max_bounds=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  rotation_first=False):
        tsr = TaskSpaceRegion()
        tsr.origin_to_tsr = origin_to_tsr
        tsr.tsr_to_end = tsr_to_end
        tsr.min_bounds = min_bounds
        tsr.max_bounds = max_bounds
        tsr.end_frame_id = self._end_effector_frame
        tsr.rotation_first = rotation_first
        return tsr

    def _status_callback(self, msg: RobotLocalPlannerStatus):
        self._status = msg

    def wait_for_complete(self, goal_id, timeout_sec=WAIT_FOR_COMPLETE_TIMEOUT):
        timeout = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
        while rclpy.ok() and self.get_clock().now() < timeout:
            if self.get_constraints_status_by_id(goal_id) in [ConstraintsStatus.SATISFIED,
                                                              ConstraintsStatus.PREEMPTED]:
                return True
            self._rate.sleep()
        return False

    @property
    def end_effector_frame(self):
        return self._end_effector_frame

    @end_effector_frame.setter
    def end_effector_frame(self, value):
        self._end_effector_frame = value

    @property
    def status(self):
        return self._status

    @property
    def has_wait_complete(self):
        return self._has_wait_complete

    @has_wait_complete.setter
    def has_wait_complete(self, value):
        self._has_wait_complete = value

    def get_constraints_statuses(self):
        statuses_dict = {}
        for status in self._status.constraints_statuses:
            statuses_dict[status.id] = status.value
        return statuses_dict

    def get_constraints_status_by_id(self, goal_id):
        return self.get_constraints_statuses().get(goal_id, None)

    def is_constraints_satisfied(self, goal_id):
        return self.get_constraints_status_by_id(goal_id) == ConstraintsStatus.SATISFIED
