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
import threading
import time

from geometry_msgs.msg import TransformStamped
from hsrb_rlp_interface_py import geometry
from hsrb_rlp_interface_py.robot_local_planner import RobotLocalPlanner
import pytest
import rclpy
from sensor_msgs.msg import JointState
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tmc_planning_msgs.msg import (
    ConstraintsStatus,
    RobotLocalGoal,
    RobotLocalPlannerStatus,
)


_COMMAND_MAP = {'arm_lift_joint': 0.1, 'arm_flex_joint': 0.2, 'arm_roll_joint': 0.3, 'wrist_flex_joint': 0.4,
                'wrist_roll_joint': 0.5, 'head_pan_joint': 0.6, 'head_tilt_joint': 0.7}


class CachingSubscriber:
    def __init__(self, node, topic_type, topic_name):
        self.reset()
        self._sub = node.create_subscription(topic_type, topic_name, self._callback, 1)

    def _callback(self, msg):
        self._msg = msg

    @property
    def data(self):
        return self._msg

    def reset(self):
        self._msg = None


class StatePublisher:
    def __init__(self, node):
        self._joint_state_pub = node.create_publisher(JointState, 'joint_states', 1)
        self._tf_broadcaster = StaticTransformBroadcaster(node)

    def publish(self):
        joint_state = JointState()
        joint_state.name = ['arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'hand_motor_joint',
                            'head_pan_joint', 'head_tilt_joint', 'wrist_flex_joint', 'wrist_roll_joint']
        joint_state.position = [0.0] * 8
        self._joint_state_pub.publish(joint_state)

        odom_to_base = TransformStamped()
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_footprint'
        odom_to_base.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(odom_to_base)


@pytest.fixture
def setup(mocker):
    rclpy.init()

    rlp = RobotLocalPlanner()

    test_node = rclpy.create_node('test_node')
    goal_cache = CachingSubscriber(test_node, RobotLocalGoal, 'hsrb_robot_local_planner/constraints')
    state_pub = StatePublisher(test_node)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(rlp)
    executor.add_node(test_node)

    # Since there is no guarantee that RobotLocalPlanner has received it, let's run it an appropriate number of times
    for _ in range(20):
        state_pub.publish()
        executor.spin_once()

    yield (rlp, goal_cache, executor, test_node)

    test_node.destroy_node
    rlp.destroy_node()
    rclpy.shutdown()


def _test_transform(actual_transform, expected_x, expected_y):
    assert actual_transform.translation.x == pytest.approx(expected_x)
    assert actual_transform.translation.y == pytest.approx(expected_y)


def _test_joint_state(actual_state, expected_dict):
    assert len(actual_state.name) == len(expected_dict)
    assert len(actual_state.position) == len(expected_dict)
    for name, position in expected_dict.items():
        index = actual_state.name.index(name)
        assert actual_state.position[index] == pytest.approx(position)


def _test_control_config(goal, base):
    assert goal.enable_base == base


def _wait_for(func, executor):
    timeout = time.time() + 5.0
    while rclpy.ok():
        executor.spin_once()
        if func():
            return True
        elif time.time() > timeout:
            return False
    return False


def _wait_for_constraints(goal_cache, executor):
    return _wait_for(lambda: goal_cache.data is not None, executor)


def test_move_base_relative(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_base_relative(x=0.1, y=0.2)
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    transform = goal.constraints.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0]

    assert goal.constraints.hard_joint_constraints[0].header.frame_id == 'base_footprint'
    _test_transform(transform, 0.1, 0.2)
    _test_control_config(goal, True)


def test_move_base_absolute(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_base_absolute(x=0.3, y=0.4)
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    transform = goal.constraints.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0]

    assert goal.constraints.hard_joint_constraints[0].header.frame_id == 'map'
    _test_transform(transform, 0.3, 0.4)
    _test_control_config(goal, True)


def test_move_base_any_frame(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_base_any_frame(x=0.5, y=0.6, frame_id='test_frame')
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    transform = goal.constraints.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0]

    assert goal.constraints.hard_joint_constraints[0].header.frame_id == 'test_frame'
    _test_transform(transform, 0.5, 0.6)
    _test_control_config(goal, True)


def test_move_to_joint_positions_with_base_relative(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_to_joint_positions_with_base_relative(_COMMAND_MAP, x=1.0, y=2.0)
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    joint_state = goal.constraints.hard_joint_constraints[0].min.joint_state
    transform = goal.constraints.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0]

    assert goal.constraints.hard_joint_constraints[0].header.frame_id == 'base_footprint'
    _test_joint_state(joint_state, _COMMAND_MAP)
    _test_transform(transform, 1.0, 2.0)
    _test_control_config(goal, True)


def test_move_to_joint_positions_with_base_absolute(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_to_joint_positions_with_base_absolute(_COMMAND_MAP, x=3.0, y=4.0)
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    joint_state = goal.constraints.hard_joint_constraints[0].min.joint_state
    transform = goal.constraints.hard_joint_constraints[0].min.multi_dof_joint_state.transforms[0]

    assert goal.constraints.hard_joint_constraints[0].header.frame_id == 'map'
    _test_joint_state(joint_state, _COMMAND_MAP)
    _test_transform(transform, 3.0, 4.0)
    _test_control_config(goal, True)


def test_move_to_joint_positions(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_to_joint_positions(_COMMAND_MAP)
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    joint_state = goal.constraints.hard_joint_constraints[0].min.joint_state

    _test_joint_state(joint_state, _COMMAND_MAP)
    _test_control_config(goal, False)


def test_move_to_neutral(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_to_neutral()
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    joint_state = goal.constraints.hard_joint_constraints[0].min.joint_state

    _test_joint_state(joint_state, {'arm_lift_joint': 0.0,
                                    'arm_flex_joint': 0.0,
                                    'arm_roll_joint': 0.0,
                                    'wrist_flex_joint': -1.57,
                                    'wrist_roll_joint': 0.0,
                                    'head_pan_joint': 0.0,
                                    'head_tilt_joint': 0.0})
    _test_control_config(goal, False)


def test_move_to_go(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_to_go()
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    joint_state = goal.constraints.hard_joint_constraints[0].min.joint_state

    _test_joint_state(joint_state, {'arm_lift_joint': 0.0,
                                    'arm_flex_joint': 0.0,
                                    'arm_roll_joint': -1.57,
                                    'wrist_flex_joint': -1.57,
                                    'wrist_roll_joint': 0.0,
                                    'head_pan_joint': 0.0,
                                    'head_tilt_joint': 0.0})
    _test_control_config(goal, False)


def test_move_end_effector_pose(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]

    assert rlp.move_end_effector_pose(geometry.pose(x=0.5))
    assert _wait_for_constraints(goal_cache, executor)

    goal = goal_cache.data
    link = goal.constraints.hard_link_constraints[0].tsr

    assert goal.constraints.hard_link_constraints[0].header.frame_id == 'odom'
    assert link.tsr_to_end.position.x == pytest.approx(0.5)
    _test_control_config(goal, True)


def test_planner_status(setup):
    rlp = setup[0]
    executor = setup[2]
    test_node = setup[3]

    status_pub = test_node.create_publisher(
        RobotLocalPlannerStatus, 'hsrb_robot_local_planner/planner_status', 1)

    planner_status = RobotLocalPlannerStatus()
    planner_status.header.stamp = test_node.get_clock().now().to_msg()
    planner_status.planner_status = RobotLocalPlannerStatus.SUCCESS
    planner_status.constraints_statuses = [
        ConstraintsStatus(id='first', value=ConstraintsStatus.SATISFIED),
        ConstraintsStatus(id='second', value=ConstraintsStatus.RUNNING)]
    status_pub.publish(planner_status)

    assert _wait_for(lambda: rlp.get_constraints_statuses(), executor)
    assert rlp.status.planner_status == RobotLocalPlannerStatus.SUCCESS
    assert rlp.get_constraints_status_by_id('first') == ConstraintsStatus.SATISFIED
    assert rlp.get_constraints_status_by_id('second') == ConstraintsStatus.RUNNING
    assert rlp.is_constraints_satisfied('first')
    assert not rlp.is_constraints_satisfied('second')


def test_wait_for_complete_combined(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]
    test_node = setup[3]

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    rlp.has_wait_complete = True

    def publish_status():
        rate = test_node.create_rate(20.0)
        while rclpy.ok():
            if goal_cache.data is not None:
                break
            rate.sleep()
        id = goal_cache.data.id

        status_pub = test_node.create_publisher(
            RobotLocalPlannerStatus, 'hsrb_robot_local_planner/planner_status', 1)
        planner_status = RobotLocalPlannerStatus()
        planner_status.header.stamp = test_node.get_clock().now().to_msg()
        planner_status.planner_status = RobotLocalPlannerStatus.CONSTRAINTS_EMPTY
        planner_status.constraints_statuses = [ConstraintsStatus(id=id, value=ConstraintsStatus.SATISFIED)]
        status_pub.publish(planner_status)

    publish_status_thread = threading.Thread(target=publish_status)
    publish_status_thread.start()

    assert rlp.move_to_neutral()

    publish_status_thread.join()

    executor.shutdown()
    spin_thread.join()


def test_wait_for_complete_separated(setup):
    rlp = setup[0]
    goal_cache = setup[1]
    executor = setup[2]
    test_node = setup[3]

    goal_id = rlp.move_to_neutral()
    assert _wait_for_constraints(goal_cache, executor)
    assert goal_cache.data.id == goal_id

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    status_pub = test_node.create_publisher(
        RobotLocalPlannerStatus, 'hsrb_robot_local_planner/planner_status', 1)

    planner_status = RobotLocalPlannerStatus()
    planner_status.header.stamp = test_node.get_clock().now().to_msg()
    planner_status.planner_status = RobotLocalPlannerStatus.SUCCESS
    planner_status.constraints_statuses = [ConstraintsStatus(id=goal_id, value=ConstraintsStatus.RUNNING)]
    status_pub.publish(planner_status)

    assert not rlp.wait_for_complete(goal_id, 0.5)

    planner_status.constraints_statuses = [ConstraintsStatus(id=goal_id, value=ConstraintsStatus.PREEMPTED)]
    status_pub.publish(planner_status)

    assert rlp.wait_for_complete(goal_id)

    executor.shutdown()
    spin_thread.join()
