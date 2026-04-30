# hsrb_robot_local_planner
ROS 2 Humble向けの HSR-B 用ローカルプランナー（Robot Local Planner）と、それをPythonからインタラクティブに操作するためのインターフェースパッケージです。

[[`Base_code`](https://github.com/hsr-project/hsrb_robot_local_planner)]
[[`Project_page`](https://toyotafrc.github.io/RobotLocalPlanner-Proj/)]

<!-- docs/robot_local_planner_system_overview.mp4) -->
<!-- <video controls>
    <source src="docs/robot_local_planner_system_overview.mp4" type="video/mp4">
</video> -->


https://github.com/user-attachments/assets/6d74dce5-e679-49f1-855e-3191f10acc53



このリポジトリは以下の2つの主要パッケージで構成されています。
- `hsrb_robot_local_planner_node`: Gazeboシミュレータ等の HSR-B と連携し、ローカルパス計画および軌道生成を行うROS 2ノード。
- `hsrb_rlp_interface_py`: Pythonからローカルプランナーにゴールを指示するためのインターフェースと、インタラクティブに操作できるシェル (`irlp`) を提供するパッケージ。

## ビルド方法

```bash
cd <your_workspace>
colcon build --packages-up-to hsrb_robot_local_planner_node hsrb_rlp_interface_py --symlink-install
source install/setup.bash
```

## 実行方法

### 1. プランナーノードの起動（シミュレータで確認）

Gazebo等のシミュレータが起動し、`/controller_manager` から `robot_description` が取得できる状態で以下のコマンドを実行します。

```bash
source /hsr_ros2_ws/install/setup.bash
ros2 launch hsrb_robot_local_planner_node hsrb_robot_local_planner.launch.py
```

このlaunchファイルは自作であり、HSR-B用のURDFや衝突判定モデルを自動的に取得・設定し、`hsrb_analytic_ik` をソルバーとしてプランナーノードを起動します。

### 2. インタラクティブシェル (`irlp`) の起動

別ターミナルを開き、プランナーを操作するためのPythonシェルを起動します。

```bash
source /hsr_ros2_ws/install/setup.bash
ros2 run hsrb_rlp_interface_py irlp
```

起動するとIPythonベースのプロンプトが表示され、`rlp` インスタンスを通じてロボットを操作できます。

#### 主要なコマンド例

```python
# 動作完了まで待機するかどうか（Trueを推奨）
rlp.has_wait_complete = True

# 定義済みのGO姿勢へ移動
rlp.move_to_go()

# ベースの相対移動 (x[m], y[m], yaw[rad])
rlp.move_base_relative(x=0.5)
rlp.move_base_relative(yaw=1.57)

# エンドエフェクタ（手先）の姿勢を指定して移動
# geometryのpose関数を使用して、x, y, z座標と roll(ei), pitch(ej), yaw(ek) を指定します
from hsrb_rlp_interface_py.geometry import pose
rlp.move_end_effector_pose(
    pose(x=0.5, y=0.0, z=0.8, ei=0.0, ej=0.0, ek=0.0)
)
```

## アーキテクチャと依存関係

- `tf2_ros`: 座標変換を利用するため、シミュレーション時はノードが `use_sim_time=True` を必要とします。本パッケージの `robot_local_planner.py` では、最新の変換を安全に取得するため `rclpy.time.Time()` を使用してTFをルックアップしています。
- **Executor**: `irlp` シェルは内部で `MultiThreadedExecutor` をバックグラウンドスレッドで実行しており、インタラクティブな入力中もROSのトピックやアクションのコールバックが正常に処理されるよう設計されています。
