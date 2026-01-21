`hsrb_quick_path_optimizer`
===============================================================================

概要
-------------------------------------------------------------------------------
現在の動作速度，HSR-B以降の台車運動学を考慮した，時間軌道最適化を提供する.


ROS Interfaces
-------------------------------------------------------------------------------

### 提供するアクション

- ~/optimize(tmc_planning_msgs/action/OptimizeRobotTrajectoryAction)：
  時間軌道最適化.入力軌道の1点目に初期位置・速度を,2点目以降に経由する位置を与えること.
  最適化計算失敗時は,入力軌道を結果として返す.
  また,パラメータ~is_force_succeededがtrueの場合,最適化計算に失敗しても,アクションは成功となる.
  入力軌道に,最適ではないが時間情報が含まれている場合に,
  最適化計算に失敗したなら既に持っている時間情報を利用する,というケースを想定している.

### 購読するトピック

- joint_states (sensor_msgs/msg/JointState)： 関節情報,台車関節情報の取得に利用.
- odom (nav_msgs/msg/Odometry)： オドメトリ,台車姿勢yaw成分の取得に利用.

### 利用するパラメータ

- is_force_succeeded(bool, optional)：
  trueの場合,最適化計算に失敗しても,アクションを成功とする.デフォルト値はfalse.
- sampling_interval_sec(double, optional)：
  最適化した軌道の,点間の刻み幅[sec].デフォルト値は0.1[sec].
- max_caster_velocity(double, optional)：
  旋回軸の最大速度[rad/sec].デフォルト値は1.8[rad/sec].
- max_wheel_velocity(double, optional)：
  車輪の最大速度[rad/sec].デフォルト値は8.5[rad/sec].
- max_caster_acceleration(double, optional)：
  旋回軸の最大加速度[rad/sec^2].デフォルト値は1.8[rad/sec^2].
- max_wheel_acceleration(double, optional)：
  車輪の最大速度[rad/sec^2].デフォルト値は5.0[rad/sec^2].
- tread(double, optional)：
  車輪間距離[m].デフォルト値は0.266[m].
- caster_offset(double, optional)：
  旋回軸と車軸の距離[m].デフォルト値は0.11[m].
- wheel_radius(double, optional)：
  車輪半径[m].デフォルト値は0.04[m].
- ${関節名}.velocity(double)：
  関節速度上限[m/sec] or [rad/sec].
  利用する関節に加え,仮想関節odom_x,odom_y,odom_tの上限も設定すること.
  それぞれ,台車並進x方向,台車並進y方向,台車旋回で,通常odom_xとodom_yの上限は同じとなる.
- ${関節名}.acceleration(double)：
  関節加速度上限[m/sec^2] or [rad/sec^2].
  利用する関節に加え,仮想関節odom_x,odom_y,odom_tの上限も設定すること.
  ただし,仮想関節odom_x,odom_y,odom_tの上限は,台車状態から再計算され,
  再計算結果と設定した上限の大きい方が最適化計算に利用される.
- thread_pool_size(int, optional)：
  最適化計算のスレッドプールのスレッド数
  "hsrb_quick_path_optimizer/OptimizerPluginMultiThread"で利用される
  デフォルトは8

### 動的なパラメータ
- optimize_timeout(double)：
  最適化計算のタイムアウト[sec]．デフォルト値は0.1[sec]
- velocity_ratio(double)：
  関節速度上限の倍率．デフォルト値は1.0
- acceleration_ratio(double)：
  関節加速度上限の倍率．デフォルト値は1.0
- acceleration_rate_for_stop(double)：
  停止時の関節加速度上限の倍率．デフォルトは0.0で，0.0より大きい場合，停止を経由する時間最適化も実行される