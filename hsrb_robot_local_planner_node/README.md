hsrb_robot_local_planner_node
---------------------------

RobotLocalPlannerを用いたHSRB/Cのマニピュレーション機能を提供する

## ROSインターフェース

### 購読するトピック

- **~/constraints** [tmc_planning_msgs/msg/RobotLocalGoal]

  RobotLocalPlannerを用いた満たすべき拘束条件

- **base_trajectory_controller_state** [control_msgs/msg/JointTrajectoryControllerState]

  軌道生成の初期姿勢として利用

- **joint_states** [sensor_msgs/msg/JointState]

  軌道生成の初期姿勢として利用


### 発行するトピック

- **{head_trajectory_controllersで指定された名前}/joint_trajectory** [trajectory_msgs/msg/JointTrajectory]

- **{arm_trajectory_controllersで指定された名前}/joint_trajectory** [trajectory_msgs/msg/JointTrajectory]

- **{hand_trajectory_controllersで指定された名前}/joint_trajectory** [trajectory_msgs/msg/JointTrajectory]

- **omni_base_controller/joint_trajectory** [trajectory_msgs/msg/JointTrajectory]

- **~/is_constraints_empty** [std_msgs/msg/Bool]

  追従中のconstraintsが存在するか否か

- **~/planner_status** [tmc_planning_msgs/msg/RobotLocalPlannerStatus]

  RobotLocalPlannerの状態


### パラメータ

- head_trajectory_controllers(string[]):
  頭部の関節軌道コントローラの名前群
  デフォルトは[head_trajectory_controller]

- arm_trajectory_controllers(string[]):
  腕部の関節軌道コントローラの名前群
  デフォルトは[arm_trajectory_controller]

- hand_trajectory_controllers(string[]):
  グリッパの軌道コントローラの名前群
  デフォルトは[gripper_controller]

- acceralation_limit(double):
  台車加速度の上限[rad/s^2]，台車速度制御から接続する際に，台車軌道の最初の数点を削る処理で使用
  デフォルトは0.5

- remove_completed_constraints(bool):
  constraintsを満たした際に，追従中のconstraintsを削除するか否か
  デフォルトはtrue

- generate_action(string):
  generatorのプラグイン名
  デフォルトは"tmc_simple_path_generator/SimplePathGeneratorPlugin"

- evaluate_action(string):
  evaluatorのプラグイン名
  デフォルトは"tmc_local_path_evaluator/LocalPathEvaluatorPlugin"

- validate_action(string):
  validatorのプラグイン名
  デフォルトは"tmc_collision_detecting_validator/CollisionDetectingValidatorPlugin"

- optimize_action(string):
  optimizerのプラグイン名
  デフォルトは"hsrb_quick_path_optimizer/OptimizerPlugin"

- kinematics_type(string):
  運動学クラスIRobotKinematicsModelのプラグイン名
  デフォルトは"tmc_robot_kinematics_model::PinocchioWrapper"

- robot_description_kinematics(string):
  ロボットモデル
  FK計算で利用される
  デフォルトは空文字列

- robot_description(string):
  ロボットモデル
  robot_description_kinematicsが空の場合，こちらのロボットモデルを利用する
  デフォルトは空文字列
  頭部の関節名リスト

- joint_displacement_threshold(double):
  関節範囲拘束を満たしたとみなす距離の閾値
  デフォルトは0.1

- link_displacement_threshold(double):
  手先位置姿勢範囲拘束を満たしたとみなす距離の閾値
  デフォルトは0.005

- joint_stall_threshold(double):
  関節範囲拘束の変化が停止したとみなす距離の変化の閾値
  デフォルトは0.005

- joint_stall_threshold(double):
  手先位置姿勢範囲拘束の変化が停止したとみなす距離の変化の閾値
  デフォルトは0.001

- joint_stall_check_range(double):
  関節範囲拘束の変化の停止判定を行う距離の閾値
  デフォルトはjoint_displacement_thresholdの3倍

- link_stall_check_range(double):
  手先位置姿勢範囲拘束の変化の停止判定を行う距離の閾値
  デフォルトはlink_displacement_thresholdの3倍
