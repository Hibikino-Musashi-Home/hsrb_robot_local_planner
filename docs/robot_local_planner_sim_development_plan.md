# RLP開発ロードマップとIsaac Sim検証環境

## 目的

この文書では、`hsrb_robot_local_planner`（RLP）を実機へ持っていくまでの開発順序と、各段階で使用する `carrobo-isaac` の検証環境を対応づける。

競技用の4部屋アリーナは、複数の家具・物体・移動距離・自己位置推定などが同時に入るため、RLP単体の不具合と環境由来の不具合を切り分けにくい。RLPの検証では競技用環境を直接変更せず、RLP専用の小さなシーンを段階的に拡張する。

## 責務の分け方

| 部分 | 担当 |
| --- | --- |
| RLPの生成・評価・衝突検証・最適化 | このリポジトリ |
| HSRの関節・台車・センサ・controller stateのSim再現 | `carrobo-isaac` |
| RLPへの目標送信 | `hsrb_rlp_interface_py` または検証スクリプト |
| 環境障害物のRLP向け形状化 | S4の`CollisionObject`/PhysX scene bridge |
| 把持物体のRLP向けAttached Object化 | 後半で追加するattached-object bridge |

`carrobo-isaac` 側にRLP本体を持たせる必要はない。RLPはこのリポジトリから起動し、SimとはROS 2のトピック・TF・controller interfaceで接続する。RLPをどのROS 2環境から起動するかは実行構成の問題であり、検証時には対象のRLPビルドを使っていることを確認する。
したがって、`carrobo-isaac` のDockerfileに含まれるRLPパッケージのcloneは、この検証計画のRLP実装・チューニング対象ではない。Sim側はロボット、センサ、controller、worldの提供に限定する。

## 共通ルール

### 座標系

- HSRの初期位置は `(x, y, yaw) = (0, 0, 0)` とする。
- ロボットの正面を `+X` とする。
- `odom` とSimワールドの座標を一致させる。
- 手先リンクは原則 `hand_palm_link` を使う。
- 競技用の部屋座標や物体配置を、RLPの初期検証へ持ち込まない。

### 失敗を隠さない

- `is_force_succeeded=false` を維持する。
- `~/planner_status` を必ず記録し、Generate / Evaluate / Validate / Optimize のどこで失敗したかを残す。
- `~/displacements`、生成候補軌道、採用軌道を記録または可視化する。
- 各試行の前にSimをリセットし、同じ初期姿勢・同じ物体姿勢から開始する。
- 成功率だけでなく、計画周期、最終残差、台車の意図しない移動、Sim上の接触も確認する。

### 環境を追加する条件

次の段階へ進む条件は「見た目上動いた」ではなく、現在の段階の合格条件を満たすこととする。合格前に家具、障害物、認識誤差、動的物体を追加しない。

## Simシーンの起動方法

`carrobo-isaac` では、競技用シーンを変更せずにRLP検証用シーンを選べる。シーンを切り替えるときは、いったんSimを停止してから起動する。

Isaac SimのGUIをX11で表示するため、ホスト側で最初に次を実行する。

```bash
xhost local:
```

```bash
cd /home/hma/carrobo-isaac

# S0〜S2: HSRと空の作業空間だけ。対象物・家具・人は配置しない
make up localhost SCENE=rlp_empty

# S3: 上記に正面の対象物を1個だけ追加
make down
make up localhost SCENE=rlp

# S3.5: 物体・障害物を外し、HSR-Bの台車/関節モデルを確認
make down
make up localhost SCENE=rlp_empty

# S4: RLP検証ランナーから同じ箱をRLPとPhysXへ登録
make down
make up localhost SCENE=rlp_empty
```

RLPノードは別ターミナルで、Apptainerへ入ってからこのリポジトリのbuildを使い、`use_sim_time:=true` を付けて起動する。S0〜S2、S3.5、S4では `placement.rlp_empty.yaml`、S3では `placement.rlp.yaml` が使われる。

```bash
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true
```

## 段階とSim環境の対応

| 段階 | RLP側で実装・確認する内容 | Sim環境 | 合格条件 |
| --- | --- | --- | --- |
| S0: 起動・接続 | ビルド、pluginロード、robot description、TF、controller stateの確認 | 競技物体・家具のない平面。遠い外周壁は作業範囲の目印で評価対象外 | ノードが安定起動し、必要な入力トピックが流れる。評価pluginのロード警告がない |
| S1: 腕単体 | 関節目標と手先目標。`enable_base=false`。Generate→Evaluate→Validate→Optimizeの一巡 | S0と同じ。外周壁はテスト経路外で、RLPの障害物入力は空 | 固定した腕目標を複数回実行し、毎回SUCCESS。台車が動かず、手先残差が収束 |
| S2: 自由空間whole-body | 台車のみ、腕のみ、台車＋腕の複合目標。速度・加速度制約の確認 | S0の広い平面。外周壁に近づかず、まだ障害物回避は評価しない | 台車の直進・横移動・旋回と複合動作が安定。周期内に恒常的なtimeoutがない |
| S3: Simチューニング | 候補数、IK初期値、評価重み、到達判定、時間予算を一項目ずつ調整 | 正面に対象物を1個だけ配置。例: apple/canを `x=0.65〜0.70, y=0` | 目標姿勢への接近・退避を固定シナリオで反復できる。変更前後を同じ指標で比較できる |
| S3.5: ハードウェアモデル整合 | `tread`、`caster_offset`、`wheel_radius`、台車/関節の速度・加速度上限を明示し、台車4ケース＋関節8ケースを確認 | S0〜S2と同じ空間。物体・家具・障害物なし。SimはHSR-Bモデル | RLPの実行時パラメータがプロファイルと一致し、計画軌道の制限内で全ケースがSUCCESS・収束。実機の校正値確定は含めない |
| S4: 静的環境障害物 | `CollisionObject` の登録publisher、`odom`への座標変換、Sim PhysX scene bridge、回避経路と到達不能判定 | `rlp_validation.world`＋起動前登録済みの静的Box slot。runnerがケースごとに箱を切り替える | Sim上の障害物とRLPへ送った形状が一致し、RLPの計画軌道が接触せず回避。障害物を完全に塞いだ場合は失敗を正しく返す |
| S5: 把持物体 | `attached_collision_objects`、把持中の衝突除外ペア、attach/release | S4の障害物＋正面の対象物。把持後に退避・搬送 | 把持前、把持中、解放後で形状が正しく切り替わる。指との常時接触で全滅しない |
| S6: 認識・動的環境 | PointCloud/検出結果からの形状化、更新周期、古い物体の削除 | 物体位置ずれ、複数障害物、最後に移動物体 | 座標変換、自己点群除去、形状数、更新遅延を含めて安定。静的シーンの回帰試験を壊さない |
| S7: 実機移行 | 実機の速度・加速度・台車寸法・追従誤差に合わせる | Simと同じ固定シナリオを実機で再現 | まず無障害物・低速で確認し、S3〜S5相当のシナリオを順に実施 |

## 各段階の検証シナリオ

### S0〜S1: 最小環境

最初に作る環境は次の構成に限定する。`rlp_validation.world` の外周壁は作業範囲を示すためのもので、テスト経路から十分離して配置する。障害物としての評価はS4まで行わない。

- 4〜5 m四方程度の平面床
- 外周壁以外の壁、家具、人、競技用オブジェクトなし
- HSRは原点、正面は `+X`
- `use_sim_time=true`
- RLPの環境障害物入力は空の状態

S1では、まず現在姿勢から小さな関節目標を実行し、その後に現在姿勢近傍の手先目標を実行する。手先目標がいきなり床付近になるとIK・時間最適化・接触の問題が混ざるため、最初は空中の安全な姿勢を使う。

### S2: 自由空間whole-body

次の順番で実施する。

1. `enable_base=false` の腕動作
2. 台車の `x` 移動
3. 台車の `y` 移動
4. 台車のyaw旋回
5. 台車と手先の複合目標

この段階では回避動作を評価しない。経路生成と時間軌道最適化が、障害物のない状態で安定していることを先に確認する。

S0〜S2の実行時は、`/omni_base_controller/state` の関節順を `odom_x, odom_y, odom_t` に固定する。RLPの `CalcInitOdomState()` はこの順番で配列を読むため、`odom_t, odom_x, odom_y` のままだと台車の並進と旋回が入れ替わる。`carrobo-isaac` 側では `odom_trajectory_action_server.controlled_joints` を明示してこの順番を保証する。

また、RLPの制約状態 `SATISFIED` は、反応型の再計画が目標軌道を採用した時点で先に成立する場合がある。S2の合否は `SATISFIED` だけで決めず、`/omni_base_controller/state` の `actual.positions` と最終軌道の時間を待って、実際のx/y/yawが目標へ収束したことも確認する。

### S3: 正面対象物

対象物はSim上に置くが、最初のS3ではRLPの環境障害物として送らない。S3の目的は「対象物へ近づく目標を実行できること」であり、対象物との衝突回避はS4以降の責務とする。

推奨する固定シナリオは次の3つ。

- 対象物の上方に移動
- 対象物へゆっくり接近
- 対象物から後方へ退避

対象物の見た目には既存のYCBモデルを使える。実際の把持処理や吸着は、この段階の合格条件に含めない。

#### S3チューニング記録（2026-09-08）

`make up localhost SCENE=rlp` で apple を `odom=(0.70, 0.00)` に置き、RLPの環境障害物入力は空のまま、次の固定シナリオを各試行の前にSimリセットして実行した。

```text
above:    base_footprint 基準 (0.60, 0.00, 0.40), roll=pi, velocity=0.35
approach: odom 基準        (0.60, 0.00, 0.18), roll=pi, velocity=0.20
retreat:  odom 基準        (0.60, 0.00, 0.40), roll=pi, velocity=0.35
```

`tools/s3_tuning_runner.py` は、`planner_status`、制約状態、生成候補数、計画待ち時間、`odom -> hand_palm_link` の実測残差を記録する。各ステップの合格条件は、SUCCESSを含むこと、制約が `SATISFIED`/`PREEMPTED` になること、手先位置誤差5 cm以内・姿勢誤差0.2 rad以内で0.5秒安定することとした。

| 比較 | 試行 | 成功 | 平均候補数 | 平均status待ち | 最大位置誤差 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `max_trajectory_num=5` | 5 | 5/5 | 8.4 | 4.25 s | 1.45 cm |
| `max_trajectory_num=10` | 5 | 5/5 | 13.7 | 3.77 s | 1.55 cm |
| `ik_initial_range=0.5`（max=10） | 5 | 5/5 | 13.6 | 3.98 s | 1.53 cm |
| `ik_initial_range=1.5`（max=10） | 5 | 5/5 | 13.7 | 5.20 s | 1.65 cm |
| `generator_timeout=0.01`（max=10, IK=0.5） | 5 | 5/5 | 13.9 | 4.10 s | 1.62 cm |
| `generator_timeout=0.05`（max=10, IK=0.5） | 5 | 5/5 | 13.5 | 4.03 s | 1.59 cm |
| 採用設定（max=10, IK=0.5, generator=0.03） | 10 | 10/10 | 13.4 | 4.16 s | 1.61 cm |

以上から、S3のSim用RLP設定として `max_trajectory_num=10` と `ik_initial_range=0.5` を採用し、`generator_timeout=0.03`、`max_simple_trajectory_num=5`、`generation_thread_num=4`、`validate_timeout=0.15`、`validation_thread_num=8`、`optimize_timeout=0.05` は維持する。候補数10は障害物なしのS3での値であり、S4の回避経路を追加した時点で、候補不足による失敗がないか再評価する。

なお、制約状態が完了した後も `planner_status` が `CONSTRAINTS_EMPTY(-1)` に戻ることがあるため、試験ではSUCCESSを含む履歴と実際のcontroller/TF収束を併せて判定した。これは `SATISFIED` の瞬間だけを成功と数えないためである。

### S3.5: ハードウェアモデル整合（HSR-B Sim）

S3.5は、実機の最大性能を推定する段階ではない。`carrobo-isaac` がHSR-B固定であるため、まずRLPとSimが同じ幾何・制限値を参照していること、そしてその値でコントローラが安全に追従できることを空シーンで確認する。実機の個体差、タイヤ摩耗、床材、積載による実効値はS7で別途測定する。

今回のSim用プロファイルは次のとおりとする。

| 分類 | パラメータ | S3.5値 | 備考 |
| --- | --- | ---: | --- |
| 台車寸法 | `tread` | 0.266 m | HSR-B基準 |
| 台車寸法 | `caster_offset` | 0.11 m | 操舵軸と車軸の距離 |
| 台車寸法 | `wheel_radius` | 0.04 m | HSR-B基準 |
| 台車上限 | `max_caster_velocity` / `max_wheel_velocity` | 1.8 / 8.5 rad/s | RLPの計画上限 |
| 台車上限 | `max_caster_acceleration` / `max_wheel_acceleration` | 1.8 / 5.0 rad/s² | RLPの計画上限 |
| 仮想台車関節 | `odom_x/y/t.velocity` | 0.2 / 0.2 / 0.5 | RLPの仮想関節上限 |
| 仮想台車関節 | `odom_x/y/t.acceleration` | 0.1 / 0.1 / 0.5 | RLPの仮想関節上限 |

腕・頭・ハンドの関節上限は、HSR-B標準値としてlaunchに明示した。S3.5では、各関節をGO姿勢近傍で小さく動かし、RLPが出した `JointTrajectory` の速度・加速度がプロファイル内に収まること、controller/TFの実状態が収束することを確認する。`joint_states` から得たSimの物理応答は別の診断値として記録する。現在のcarrobo-isaacは高剛性のposition-driveを使っており、RLPの加速度制限をアクチュエータ側で直接制限しないため、物理応答の加速度超過は警告として扱い、RLPの指令軌道制限違反とは分けて評価する。最大値そのものの同定は実機で行う。

実行スクリプトは [`tools/s35_hardware_model_runner.py`](../tools/s35_hardware_model_runner.py) である。S3.5では対象物を使わない。

```bash
# Apptainer側（ユーザーの共通手順を完了したシェル）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s35_hardware_model_runner.py \
  --trials 1 | tee /tmp/rlp_s35_hardware_model.jsonl
```

合格条件は、パラメータ監査が一致し、RLPの計画軌道に制限違反がなく、台車4ケース（x、y、yaw、複合）と関節8ケースがすべて planner SUCCESS・制約終端・物理状態収束になることとする。実測台車誤差は位置2 cm・yaw 0.05 rad以内、関節目標はrunnerの許容誤差内とする。Simの物理応答に加速度警告が出た場合は記録して原因を確認するが、stock position-drive由来であることを切り分けたうえで、RLPの計画軌道が制限内ならS3.5の合否とは分けて扱う。いずれかの合否条件が失敗した場合はS4へ進まず、モデル値・関節名/順序・controller stateを再確認する。

#### S3.5実行記録（2026-09-08）

`carrobo-isaac` を `SCENE=rlp_empty` で再起動し、Apptainer内で共通のROS環境を初期化した後、RLPを `use_sim_time:=true` で起動して検証した。実行コマンドは次のとおりである。

```bash
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s35_hardware_model_runner.py \
  --trials 1 --reset-settle-sec 2.0 --timeout-sec 15.0 \
  | tee /tmp/rlp_s35_full_final2.jsonl
```

| 指標 | 結果 |
| --- | ---: |
| パラメータ監査 | PASS |
| 台車ケース | 4/4 |
| 関節ケース | 8/8 |
| RLP計画軌道の制限違反 | 0 |
| Sim物理応答の制限警告 | 7/8（診断警告） |
| 試行全体 | 1/1 PASS |

最終試行の最大誤差は、台車位置 `0.01891 m`、台車yaw `0.04851 rad`、関節 `0.01107` で、S3.5の判定閾値内だった。物理応答の加速度警告はSimのposition-drive特性によるものであり、実機の加速度上限を確認した結果ではない。実機へ移る前に、実機ログ・ドライバ側制限・追従誤差を用いて同じ監査をやり直す。

### S4: 静的障害物

Isaac Simの見た目・物理衝突と、RLPの衝突判定用形状を同じ寸法・同じ `odom` 姿勢で管理する。RLP側へは点群やOctomapを直接送らず、少数のBox/Cylinderプリミティブへ変換して送る。

RLPのvalidatorが読むトピックは `collision_environment_server/transformed_environment` に固定されている。`PlanningSceneWorld.octomap` は使われない。`collision_environment_server` がこの出力を周期的に再発行するため、S4のpublisherは出力トピックへ直接送らず、入力トピック `collision_environment_server/collision_object` へ `moveit_msgs/CollisionObject` のBoxを `ADD/REMOVE` する。サーバが `odom` へ変換した出力をRLPが読む。座標は `CollisionObject.header.frame_id=odom`、`CollisionObject.pose` にBox中心と姿勢、`primitive_poses[0]` は単位姿勢で統一する。

S4の最小シナリオは次の3ケースとする。Box中心と目標は、各ケースでロボットがgo姿勢へ移った直後のロボットローカル座標で定義し、runnerが現在の `odom` 姿勢へ解決してから登録・送信する。目標はローカル座標 `(x=1.20, y=0, yaw=0)` とし、最初は腕のランダム中間姿勢との干渉を避けるため、台車高さだけを塞ぐBox（高さ `0.30 m`）で検証する。高い障害物とのwhole-body衝突は、低いBoxで台車回避が通った後に追加する。

| ケース | Box中心 `(x,y)` [m] | Box寸法 `(x,y,z)` [m] | 期待結果 |
| --- | --- | --- | --- |
| `avoidable_box` | `(0.55, 0.00)` | `(0.30, 0.40, 0.30)` | 直進を避け、左右どちらかへ迂回して到達 |
| `one_side_pass` | `(0.55, 0.25)` | `(0.30, 0.40, 0.30)` | 開いている負のy側へ迂回して到達 |
| `blocked_wall` | `(0.55, 0.00)` | `(0.30, 3.00, 0.30)` | validator由来の計画失敗を返す |

この固定publisherと基準動作を [`tools/s4_obstacle_runner.py`](../tools/s4_obstacle_runner.py) に実装した。`avoidable_box` と `one_side_pass` では、RLPが出した `world_joint` 軌道の最小Boxクリアランスと横方向の迂回量を診断し、`blocked_wall` では成功軌道を出さずにplanner failureを返すことを確認する。ケース切り替え時は入力トピックへ `REMOVE` を送り、ケース終了後に障害物がvalidatorのcacheへ残らないようにする。出力トピックへ直接送るとcollision environment serverの周期発行で上書きされるので注意する。

Sim側にも同じBoxを物理障害物として反映する。runnerはJSONスナップショットを `/rlp_validation/physical_obstacles` へ送り、`carrobo-isaac` は `/World/RLPObstacles/slot_0`〜`slot_7` の静的Cube colliderへ反映する。適用完了は `/rlp_validation/physical_obstacles_applied` のsequence ack、HSRが物理Boxへ接触した場合は `/rlp_validation/physical_obstacle_contact` で確認する。Boxはgo直後のロボットローカル座標で送り、Sim側でその時点のHSRワールド姿勢へ変換するため、RLPの`odom`形状とSimのワールド形状の基準をそろえられる。colliderの生成・削除を実行中に行うとIsaac Sim 4.5のArticulation viewが無効化されることがあるため、slotはPhysX初期化前に登録し、実行中はtransformだけを更新する。

障害物回避ではS3の候補数では乱数中間姿勢による迂回候補が不足したため、S4のSimプロファイルとして `generator_timeout=0.20`、`max_simple_trajectory_num=20`、`max_trajectory_num=50`、`validate_timeout=0.50` をlaunchへ設定した。`middle_state_base_position_range=1.0`、`generation_thread_num=4`、`validation_thread_num=8`、`optimize_timeout=0.05` は維持する。

実行手順は次のとおりである。

```bash
# ホスト側
xhost local:
cd /home/hma/carrobo-isaac
make up localhost SCENE=rlp_empty

# Apptainer側（RLP launch用ターミナル）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true

# さらに別ターミナルで、同じApptainer初期化手順を実行
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s4_obstacle_runner.py \
  --scenario all --trials 1 --reset-settle-sec 2.0 --timeout-sec 45.0 \
  | tee /tmp/rlp_s4_obstacles.jsonl
```

#### S4実行記録（2026-09-08）

`xhost local:` 実行後、`carrobo-isaac` を `SCENE=rlp_empty` で起動し、RLPと検証runnerは次のApptainer共通手順で起動した。

```bash
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
```

`tools/s4_obstacle_runner.py --scenario all --trials 1 --reset-settle-sec 2.0 --timeout-sec 45.0` を実行し、RLPへの`CollisionObject`登録、Sim側のphysical obstacle ack、HSRの物理接触フラグ、planner/controllerの収束を同時に確認した。結果は次のとおりである。

| ケース | RLP結果 | Sim物理Box | 物理接触 | 判定 |
| --- | --- | --- | --- | --- |
| `avoidable_box` | detour + SUCCESS | sequence ack | なし | PASS |
| `one_side_pass` | 負のy側へdetour + SUCCESS | sequence ack | なし | PASS |
| `blocked_wall` | validation failure（期待どおり） | sequence ack | なし | PASS |

最終結果は **3/3 PASS** であり、RLPの環境入力・計画・Sim上のPhysX障害物・物理走行を含むS4の固定シナリオを完了した。なお、`planned_min_clearance_m` はHSRを半径`0.24 m`の円で近似した診断値であり、最終判定はRLP validatorとSimの物理接触フラグを正とする。

合格条件は、環境topicにRLPのsubscriberが接続し、回避可能・片側通過の2ケースがplanner SUCCESS・制約終端・物理収束となり、計画軌道がBoxへ接触せず、到達不能ケースがplanner failureとなること、さらに同じID・寸法・`odom`姿勢のBoxがSim上にも存在し接触しないことである。

最初から薄い板や複雑なメッシュを使わない。衝突検証のサンプリング間隔や計算時間の問題と、形状の問題を分離できなくなるためである。

### S5: 把持物体

把持物体は、把持が確定したイベントで一度だけAttached Objectへ切り替える。物体を `hand_palm_link` に対する相対姿勢で表し、指先との接触ペアは必要なものだけ無効化する。解放時には空の `attached_collision_objects` を明示的に送る。

検証順は「把持前の接近 → attach → 障害物近傍を退避 → 解放 → attach解除」とする。把持物体を毎周期の認識結果として更新する処理はS6まで入れない。

## 実装成果物の予定

### このリポジトリ

- Sim用のRLP launch/パラメータ設定
- S1〜S3の固定コマンド・回帰テストスクリプト（S3: `tools/s3_tuning_runner.py`）
- S3.5のハードウェアモデル監査・基準動作スクリプト（`tools/s35_hardware_model_runner.py`）
- `planner_status` / `displacements` / 実行時間の記録
- S4の `CollisionObject` publisher・Sim PhysX scene bridge・固定Boxシナリオ・回避/到達不能回帰（`tools/s4_obstacle_runner.py`）
- S5のAttached Object publisherと衝突除外ペアの管理

### `carrobo-isaac`

- `worlds/rlp_validation.world`: 競技用とは独立した最小ワールド
- `configs/placement.rlp_empty.yaml`: S0〜S2用の空配置設定
- `configs/placement.rlp.yaml`: 正面対象物だけを置く配置設定
- RLP用world/configの切替機構
- S3.5用のHSR-B台車寸法・速度上限の環境変数（`BASE_WHEEL_SEPARATION`、`BASE_CASTER_OFFSET` など）
- S4用の静的障害物slot（`/World/RLPObstacles/slot_0`〜`slot_7`）とROS bridge

競技用 `worlds/carrobo.world` と `configs/placement*.yaml` は、RLP検証用の変更で上書きしない。

## 初回の合格判定

S1を「動いた」と判定する条件は以下とする。

- RLPが対象のビルドから起動している
- `planner_status` にGenerate/Evaluate/Validate/Optimizeの恒常的な失敗がない
- `is_force_succeeded=false`
- 同じ初期状態から10回以上連続で目標を完了する
- 台車目標を送っていない腕単体試験で台車が移動しない
- 目標完了時の手先・関節残差を記録できる
- Isaac Sim上で意図しない接触がない

この条件を満たした後にS2へ進み、S3のチューニングを開始する。

## 実行記録

2026-09-07に、`xhost local:` とApptainerの共通手順を使い、`make up localhost SCENE=rlp_empty` 上でS0〜S2のスモーク検証を実施した。

- S0: Isaac Sim、`/clock`、`/joint_states`、`/omni_base_controller/state`、RLPの4プラグイン接続を確認。
- S1: HSR標準のneutral姿勢でRLPの `planner_status=SUCCESS`、制約状態 `SATISFIED`、関節の物理収束を確認。任意の姿勢を使うと `arm_flex_link` と `base_link` の自己干渉になり得るため、初期試験はneutral/goなどの既知の安全姿勢から始める。
- S2: neutral → 台車x移動 → y移動 → yaw旋回 → 台車＋腕＋頭の複合目標を実行し、全ケースでRLPの成功、controllerの実到達、台車の目標外移動がないことを確認。

ここでのS1/S2はスモーク検証であり、同じ初期状態から10回以上連続で行う回帰試験は、S3のチューニング前に追加する。

S3.5は「HSR-B Simモデルでの整合確認」までを完了条件とし、実機用の値を確定したことを意味しない。実機へ移る前に、同じ項目を実機ログ・実測値で再取得し、S3.5のプロファイルを更新してからS7へ進む。

## 参照

- [RLPパラメータ調査・チューニングガイド](./robot_local_planner_tuning_guide.md)
- [Validatorの入力と環境障害物/Attached Object](./robot_local_planner_tuning_guide.md#入力の実体--障害物と把持物体はどこから来るか)
