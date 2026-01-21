#!/usr/bin/env python3
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
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import (
    Node,
    SetParameter,
)

from tmc_launch_ros_utils.tmc_launch_ros_utils import (
    load_collision_description,
    load_robot_description,
)


def declare_arguments():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'))

    declared_arguments.append(
        DeclareLaunchArgument('description_package',
                              default_value='hsrc_description',
                              description='Description package with robot URDF/xacro files.'))
    declared_arguments.append(
        DeclareLaunchArgument('description_file',
                              default_value='hsrc1s.urdf.xacro',
                              description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument('collision_file',
                              default_value='collision_pair_hsrc.xml',
                              description='Robot collision config xml file.'))

    declared_arguments.append(
        DeclareLaunchArgument('trajectory_origin_frame',
                              default_value='odom',
                              description='origin frame of path planning.'))
    return declared_arguments


def generate_launch_description():
    robot_description = load_robot_description()
    robot_collision_config = load_collision_description()

    joint_names = ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                   'wrist_flex_joint', 'wrist_roll_joint',
                   'hand_motor_joint',
                   'head_pan_joint', 'head_tilt_joint']
    joint_weights = [10.0, 1.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0]
    base_weights = [3.0, 3.0, 1.0]

    robot_local_planner_node_params = {
        'optimize_second': True,
        'optimize_action': 'hsrb_quick_path_optimizer/OptimizerPluginMultiThread',
        'omni_base_controller': {'joints': ['odom_x', 'odom_y', 'odom_t']},
        'link_displacement_threshold': 0.01,
        'use_current_state_for_displacement': True,
        'publish_generated_trajectories': True,
        'publish_planned_trajectory': True,
        'middle_state_base_position_range': 1.0,
        'middle_state_base_rotation_range': 1.5,
        'ik_initial_range': 1.0,
    }
    generator_params = {
        'base_movement_type': 1,
        'max_trajectory_num': 50,
        'max_simple_trajectory_num': 5,
        'generation_thread_num': 4,
        'solver_type': 'hsrb_analytic_ik/HsrcIKSolver',
        'goal_sampler_type': 'tmc_simple_path_generator/SampleGoalGenerator',
        'interpolation_type': 'tmc_simple_path_generator/PlanarTrajectoryInterpolatorPlain',
        'origin_frame': LaunchConfiguration('trajectory_origin_frame'),
        'joint_names': joint_names,
        'joint_weights': joint_weights,
        'base_names': ['world_joint'],
        'base_weights': base_weights,
        'linear_constraint_step': 0.1,
        'end_effectors': ['hand_palm_link'],
        'ik_joints': {
            'hand_palm_link': ['arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                               'wrist_flex_joint', 'wrist_roll_joint']
        }
    }
    optimizer_params = {
        'sampling_interval_sec': 0.1,
        'is_force_succeeded': False,
        'optimize_timeout': 0.05,
        'head_pan_joint': {'velocity': 2.0, 'acceleration': 1.0},
        'head_tilt_joint': {'velocity': 2.0, 'acceleration': 1.0},
        'arm_lift_joint': {'velocity': 0.15, 'acceleration': 0.15},
        'arm_flex_joint': {'velocity': 1.58, 'acceleration': 1.0},
        'arm_roll_joint': {'velocity': 2.0, 'acceleration': 1.0},
        'wrist_flex_joint': {'velocity': 2.0, 'acceleration': 1.0},
        'wrist_roll_joint': {'velocity': 2.0, 'acceleration': 1.0},
        'hand_motor_joint': {'velocity': 1.0, 'acceleration': 1.0},
        'odom_x': {'velocity': 0.2, 'acceleration': 0.1},
        'odom_y': {'velocity': 0.2, 'acceleration': 0.1},
        'odom_t': {'velocity': 0.5, 'acceleration': 0.5},
        'max_caster_velocity': 2.5,
        'max_caster_acceleration': 5.0,
        'max_wheel_velocity': 20.8,
        'max_wheel_acceleration': 41.7,
    }
    evaluator_params = {
        'score_calculations': {
            'names': ['time_base', 'soft_path_joint'],
            'time_base': {'type': 'tmc_local_path_evaluator/TimeBaseScoreCalculation'},
            'soft_path_joint': {'type': 'tmc_local_path_evaluator/SoftPathJointConstraintScoreCalculation',
                                'distance_threshold': 0.1},
        },
    }
    validator_params = {
        'print_debug_info': False,
        'save_request': False,
        'trajectory_points_collision_check_order_type': 'TrajectoryPointsCollisionCheckBothEnds',
        'time_from_start': 2.0,
        'time_from_end': 0.01,
        'interval_middle': 0.2,
        'validate_timeout': 0.15,
        'validation_thread_num': 8,
        'collision_engine': 'fcl',
    }

    robot_local_planenr = Node(package='hsrb_robot_local_planner_node',
                               executable='node_with_plugin',
                               parameters=[robot_description,
                                           robot_collision_config,
                                           robot_local_planner_node_params,
                                           generator_params,
                                           optimizer_params,
                                           evaluator_params,
                                           validator_params],
                               remappings=[('base_trajectory_controller_state', 'omni_base_controller/state'),
                                           ('odom', 'omni_base_controller/wheel_odom')])

    visualizers = [Node(package='tmc_robot_local_planner_visualization',
                        executable='link_constraints_visualization',
                        remappings=[('~/robot_local_goal', 'hsrb_robot_local_planner/constraints')]),
                   Node(package='tmc_robot_local_planner_visualization',
                        executable='planned_trajectory_visualization',
                        parameters=[robot_description],
                        remappings=[('~/planned_trajectory', 'hsrb_robot_local_planner/planned_trajectory')])]

    return LaunchDescription(declare_arguments()
                             + [SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
                                robot_local_planenr] + visualizers)
