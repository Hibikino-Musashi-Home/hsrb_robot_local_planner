#!/usr/bin/env python3
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
# All rights reserved.
# hsrb_robot_local_planner_node launch file for HSR-B

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from tmc_launch_ros_utils.tmc_launch_ros_utils import (
    load_collision_description,
    load_robot_description,
)


def declare_arguments():
    return [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'description_package',
            default_value='hsrb_description',
            description='Description package with robot URDF/xacro files.'),
        DeclareLaunchArgument(
            'description_file',
            default_value='hsrb4s.urdf.xacro',
            description='URDF/XACRO description file with the robot.'),
        DeclareLaunchArgument(
            'collision_file',
            default_value='collision_pair_hsrb.xml',
            description='Robot collision config xml file.'),
        DeclareLaunchArgument(
            'trajectory_origin_frame',
            default_value='odom',
            description='Origin frame of path planning.'),
    ]


def generate_launch_description():
    robot_description = load_robot_description()
    robot_collision_config = load_collision_description()

    joint_names = [
        'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
        'wrist_flex_joint', 'wrist_roll_joint',
        'hand_motor_joint',
        'head_pan_joint', 'head_tilt_joint',
    ]
    joint_weights = [10.0, 1.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0]
    base_weights = [3.0, 3.0, 1.0]

    robot_local_planner_node_params = {
        'optimize_second': True,
        'optimize_action': 'hsrb_quick_path_optimizer/OptimizerPluginMultiThread',
        'head_trajectory_controller': {
            'joints': ['head_pan_joint', 'head_tilt_joint'],
        },
        'arm_trajectory_controller': {
            'joints': [
                'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                'wrist_flex_joint', 'wrist_roll_joint',
            ],
        },
        'gripper_controller': {
            'joints': ['hand_motor_joint'],
        },
        'omni_base_controller': {'joints': ['odom_x', 'odom_y', 'odom_t']},
        'link_displacement_threshold': 0.01,
        # S3.5: match the HSR-B Sim controller's measured settling residual
        # (about 2 cm/rad under gravity) while remaining well below the
        # original 0.1 m/rad default.  The runner separately checks the
        # physical joint error with a tighter 0.02 tolerance.
        'joint_displacement_threshold': 0.03,
        'use_current_state_for_displacement': True,
        'publish_generated_trajectories': True,
        'publish_planned_trajectory': True,
        'middle_state_base_position_range': 1.0,
        'middle_state_base_rotation_range': 1.5,
        # S3 Sim tuning (2026-09): keep the IK search near the current posture.
        # Re-evaluate this value when S4 introduces obstacle-driven branches.
        'ik_initial_range': 0.5,
    }
    generator_params = {
        'base_movement_type': 1,
        # S4 Sim tuning (2026-09): obstacle detours need more middle-state
        # candidates than the S3 object-approach profile.  Keep the larger
        # budget explicit so the obstacle regression is reproducible after a
        # fresh launch.
        'generator_timeout': 0.20,
        'max_trajectory_num': 50,
        'max_simple_trajectory_num': 20,
        'generation_thread_num': 4,
        'solver_type': 'hsrb_analytic_ik/HsrbIKSolver',
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
            'hand_palm_link': [
                'arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                'wrist_flex_joint', 'wrist_roll_joint',
            ]
        },
    }
    optimizer_params = {
        'sampling_interval_sec': 0.1,
        'is_force_succeeded': False,
        'optimize_timeout': 0.05,
        # S3.5 HSR-B Sim hardware profile.  Keep these values explicit instead
        # of relying on hsrb_quick_path_optimizer's HSR-B defaults so a real
        # robot profile can be reviewed as one table before deployment.
        'tread': 0.266,
        'caster_offset': 0.11,
        'wheel_radius': 0.04,
        # Use flat ROS parameter names.  rclcpp/tmc_utils reads these as
        # "<joint>.velocity" and "<joint>.acceleration"; nested mappings are
        # silently omitted by the component container's parameter override
        # conversion.
        'head_pan_joint.velocity': 1.0,
        'head_pan_joint.acceleration': 1.0,
        'head_tilt_joint.velocity': 1.0,
        'head_tilt_joint.acceleration': 1.0,
        'arm_lift_joint.velocity': 0.15,
        'arm_lift_joint.acceleration': 0.15,
        'arm_flex_joint.velocity': 1.0,
        'arm_flex_joint.acceleration': 1.0,
        'arm_roll_joint.velocity': 1.0,
        'arm_roll_joint.acceleration': 1.0,
        'wrist_flex_joint.velocity': 1.0,
        'wrist_flex_joint.acceleration': 1.0,
        'wrist_roll_joint.velocity': 1.0,
        'wrist_roll_joint.acceleration': 1.0,
        'hand_motor_joint.velocity': 3.0,
        'hand_motor_joint.acceleration': 20.0,
        'odom_x.velocity': 0.2,
        'odom_x.acceleration': 0.1,
        'odom_y.velocity': 0.2,
        'odom_y.acceleration': 0.1,
        'odom_t.velocity': 0.5,
        'odom_t.acceleration': 0.5,
        'max_caster_velocity': 1.8,
        'max_caster_acceleration': 1.8,
        'max_wheel_velocity': 8.5,
        'max_wheel_acceleration': 5.0,
    }
    evaluator_params = {
        'score_calculations': {
            'names': ['time_base', 'soft_path_joint'],
            'time_base': {
                'type': 'tmc_local_path_evaluator/TimeBaseScoreCalculation',
            },
            'soft_path_joint': {
                'type': 'tmc_local_path_evaluator/SoftJointConstraintScoreCalculation',
            },
        },
    }
    validator_params = {
        'print_debug_info': False,
        'save_request': False,
        'trajectory_points_collision_check_order_type': 'TrajectoryPointsCollisionCheckBothEnds',
        'time_from_start': 2.0,
        'time_from_end': 0.01,
        'interval_middle': 0.2,
        # S4 Sim tuning (2026-09): validate the larger obstacle candidate set
        # without prematurely rejecting all candidates on the FCL deadline.
        'validate_timeout': 0.50,
        'validation_thread_num': 8,
        'collision_engine': 'fcl',
    }

    robot_local_planner = Node(
        package='hsrb_robot_local_planner_node',
        executable='node_with_plugin',
        parameters=[
            robot_description,
            robot_collision_config,
            robot_local_planner_node_params,
            generator_params,
            optimizer_params,
            evaluator_params,
            validator_params,
        ],
        remappings=[
            ('base_trajectory_controller_state', 'omni_base_controller/state'),
            ('odom', 'omni_base_controller/wheel_odom'),
        ],
    )

    return LaunchDescription(
        declare_arguments()
        + [
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            robot_local_planner,
        ]
    )
