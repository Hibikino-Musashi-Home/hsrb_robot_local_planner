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
| 把持物体のRLP向けAttached Object化 | S5のattached-object bridgeと把持物体の配置検証 |

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

# S5.1: 正面のappleを認識して把持し、Attached Objectのまま退避
make down
make up localhost SCENE=rlp GRASP=1

# S6.3a: 静的な机を認識し、appleを机上へ配置
make down
make up localhost SCENE=rlp_grasp_table GRASP=1

# S6.3a cabinet: 配置面の後方に上棚がある机
make down
make up localhost SCENE=rlp_grasp_cabinet GRASP=1

# S6.3b: 動的障害物を追跡しながら把持・机上配置
make down
make up localhost SCENE=rlp_dynamic_grasp GRASP=1

# S6.1/S6.2: pcl_reconstの点群から検出する動的障害物
make down
make up localhost SCENE=rlp_dynamic
```

RLPノードは別ターミナルで、Apptainerへ入ってからこのリポジトリのbuildを使い、`use_sim_time:=true` を付けて起動する。S0〜S2、S3.5、S4では `placement.rlp_empty.yaml`、S3では `placement.rlp.yaml` が使われる。

```bash
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true validation_thread_num:=8
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
| S5: 把持物体 | S5.1で認識→把持点→TF→接近・把持、`attached_collision_objects`、attach/release。S5.2でS4障害物、S5.3で机上配置を同時検証 | S5.1は`SCENE=rlp`の正面apple 1個。S5.2でS4のBox、S5.3で同一`odom`形状の机上スラブを追加 | S5.1は認識結果から得た姿勢へ到達し、Sim物理把持とAttached Objectを同時に確認。S5.2は把持中の形状切替と障害物回避、S5.3はAttached Objectのまま机へ運び、貫通を拒否し、開放後に机上へ残ることを確認 |
| S6: 認識・動的環境 | S6.1で`pcl_reconst`点群→BridgeA→`CollisionObject`、S6.2で移動物体の追従・stale削除・オンライン更新。S6.3aで静的な机と上棚、S6.3bで動的障害物を含む認識→把持→配置を検証 | S6.1/S6.2は`SCENE=rlp_dynamic`、S6.3aは`SCENE=rlp_grasp_table`/`rlp_grasp_cabinet`、S6.3bは`SCENE=rlp_dynamic_grasp` | 点群の`odom`変換、形状化、更新周期、古い物体の削除が安定し、静的机・上棚ではAttached Objectを安全に配置でき、S6.3bでは把持中の動的更新とhidden後のstale削除まで成立する。これらが安定してから複数クラスタ・遮蔽・遅延分布を追加する |
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

Sim側にも同じBoxを物理障害物として反映する。runnerはJSONスナップショットを `/rlp_validation/physical_obstacles` へ送り、`carrobo-isaac` は `/World/RLPObstacles/slot_0`〜`slot_7` の静的Cube colliderへ反映する。適用完了は `/rlp_validation/physical_obstacles_applied` のsequence ack、HSRが物理Boxへ接触した場合は `/rlp_validation/physical_obstacle_contact` で確認する。payloadには従来のロボットローカル座標に加えて`odom`上の`center_world`/`yaw_world`も含め、Simは後者を優先してRLPの`CollisionObject`と同じワールド形状を置く。これにより、把持退避の後など、台車が移動してから登録する机でもRLPとPhysXの位置がずれない。colliderの生成・削除を実行中に行うとIsaac Sim 4.5のArticulation viewが無効化されることがあるため、slotはPhysX初期化前に登録し、実行中はtransformだけを更新する。

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

#### `validation_thread_num` 検証（S4完了条件）

`validation_thread_num` は、validatorの並列評価数としてS4の固定Boxシナリオで独立Sim起動ごとに比較した。各設定は同じ`avoidable_box` 3試行で、RLPのplanner SUCCESS、制約終端、controller/TF収束、Sim物理接触なしを合否にした。`status_wait_wall_sec` はrunnerが状態終端を待つエンドツーエンド時間であり、validator単体のCPU時間ではない。

| `validation_thread_num` | 成功 | `status_wait_wall_sec` 平均 | 結論 |
| ---: | ---: | ---: | --- |
| 1 | 3/3 | 24.89 s | 機能PASS |
| 2 | 3/3 | 26.30 s | 機能PASS |
| 4 | 3/3 | 26.35 s | 機能PASS |
| 8 | 3/3 | 27.11 s | 機能PASS |
| 16 | 3/3 | 23.98 s | 機能PASS |

独立したclean runでは1、2、4、8、16のすべてが3/3 PASSとなり、今回のSim負荷では明確な最適値は決められなかった。したがって、S4までの既存設定と整合する`validation_thread_num=8`をS5の基準値として維持する。途中の同一Simでの連続実行では、前試行の状態を引き継いだ失敗やTF NaNが発生したため、thread数の性能比較には使わず、各設定をSim再起動またはclean resetして比較する。

### S5: 把持物体（S5.1 認識→TF→把持、S5.2 障害物統合、S5.3 机上配置）

S5.1では、競技アリーナや複数物体を使わず、既存の認識・把持点推定パッケージをそのまま接続する。

- `yolov8_detection`: YOLO segmentationモデル（`yolov11-seg_wrc_ycb.pt`）で対象物名、bbox、mask、depth、camera infoを取得する。
- `grasp_point_detection`: 認識結果のmask/depth/camera infoから把持姿勢、把持幅、物体サイズを推定する。
- runnerは推定姿勢をカメラフレームからTF2で`base_link`と`odom`へ変換し、`odom -> s5_detected_object` と `odom -> s5_grasp_target` を公開する。
- RLPのpregrasp・接触姿勢・把持後退避は、baseが動いても同じ場所を指し続けられるよう`odom`固定の姿勢で送る。`base_link`基準のまま送ると、`enable_base=true`のwhole-body動作で台車が動き、Sim上の物体との接触位置がずれる。
- 観察姿勢の設定とグリッパ開閉は`hsrb_interface`、認識結果からのpregrasp・把持・退避はRLPを使う。観察姿勢の成否とRLPの把持動作の成否を分けて記録する。
- 把持前は推定サイズのfree `CollisionObject`を登録し、接触姿勢へ入る前に一度削除する。グリッパを閉じてSimの物理把持を確認した後、同じ形状を`AttachedCollisionObject`として`hand_palm_link`へ追加する。touch linkは手掌と左右distal linkに限定する。
- `AttachedCollisionObject`を追加した後の搬送・配置では、物体形状を含む状態でRLPのvalidatorが机や障害物との衝突を判定する。単に「把持状態を記録する」だけにせず、Attached Objectがあるから拒否される目標を用意する。
- 退避後はrelease topicへobject IDを送り、Attached Objectを削除してfree `CollisionObject`を復元し、最後にグリッパを開く。
- `GRASP=1`ではSim側がグリッパ近傍の対象物を物理的に追従させ、`/rlp_validation/grasp_state`へattach/releaseのSim真値を公開する。このSim補助は把持状態の検証用であり、実機グリッパの摩擦・接触性能を証明するものではない。
- S5.3では、把持・退避が終わるまで机を登録せず、退避後に同じ`odom`中心・寸法の机上スラブをRLPとPhysXへ登録する。これにより、把持へ近づく途中の台車と机の接触を、Attached Objectの机上搬送試験と混同しない。
- 机上スラブは、まず物体を安全に上方搬送してから水平移動し、下降前姿勢を経て置き姿勢へ入る。置き姿勢の前には物体の底面が机へ2 cm侵入する意図的なprobeを送り、RLPが`VALIDATION_FAILURE(-4)`として拒否することを確認する。
- probe後は空の制約を再送してから安全な高さへ戻り、Attached Objectを保持したまま机上へ下降する。開放後はSimの物体位置履歴から机の水平範囲、垂直クリアランス、貫通量を計測する。

S5.1の対象は`carrobo-isaac`の`placement.rlp.yaml`にある、`odom=(0.70, 0.00)`のYCB apple 1個とする。カメラから手先を見通せるよう、基準観察姿勢はhead pan `-0.65 rad`、head tilt `-50 deg`とする。YOLOは画像更新のタイミングによって一時的に検出を返さないことがあるため、runnerは最大5回まで対象名を再取得するが、対象名の曖昧な推測で別物体を選ばない。

検証順は「Sim reset → 観察姿勢 → apple認識 → 把持点推定 → TF変換 → free object登録 → pregrasp → free object削除 → 接触姿勢 → Sim物理attach → Attached Object追加 → 退避 → release → free object復元 → Sim物理release」とする。把持物体を毎周期の認識結果として更新する処理はS6まで入れない。

実行runnerは [`tools/s5_recognition_grasp_runner.py`](../tools/s5_recognition_grasp_runner.py) である。S5.1のコマンドは次のとおりである。S5.2では最後のrunnerコマンドに`--with-obstacle`を追加する。

```bash
# ホスト側
xhost local:
cd /home/hma/carrobo-isaac
make up localhost SCENE=rlp GRASP=1

# Apptainer側（RLP launch用ターミナル）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true validation_thread_num:=8

# 別のApptainerターミナル（認識・把持点推定）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch yolov8_detection yolov8_detection_launch.py use_rviz:=false
ros2 run grasp_point_detection grasp_point_service

# さらに別のApptainerターミナル（S5 runner）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s5_recognition_grasp_runner.py \
  --target apple --object-id s5_detected_object --timeout-sec 55.0 \
  | tee /tmp/rlp_s5_recognition_grasp.jsonl

# S5.2: 上記のrunnerコマンドに --with-obstacle を追加
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s5_recognition_grasp_runner.py \
  --target apple --object-id s5_detected_object --with-obstacle \
  --timeout-sec 55.0 | tee /tmp/rlp_s5_with_obstacle.jsonl

# S5.3: Attached Objectを机へ運び、貫通拒否と開放後の机上安定を確認
# 既定値: table center=(0.82, 0.25, 0.30+0.04/2) [m], size=(0.35, 0.30, 0.04) [m]
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s5_recognition_grasp_runner.py \
  --target apple --object-id s5_detected_object --place-on-table \
  --timeout-sec 55.0 --reset-settle-sec 3.0 \
  | tee /tmp/rlp_s5_place_table.jsonl
```

#### S5.1実行記録（2026-09-08）

上記の`SCENE=rlp`、`GRASP=1`、`validation_thread_num=8`で1試行し、**1/1 PASS**となった。YOLOは2回目の問い合わせで`apple`を検出し、score `0.579`、把持点推定値はカメラフレームから変換して`odom=(0.6881, -0.0686, 0.0639) m`となった。pregrasp、把持姿勢、退避はすべてRLP planner SUCCESS・制約終端・物理収束であった。

| 確認項目 | 結果 |
| --- | --- |
| 認識 | `apple`、score `0.579` |
| TF | `head_rgbd_sensor_rgb_frame` → `base_link`/`odom`、2つの検証用frameを公開 |
| free `CollisionObject` | 把持前に追加、接触前に削除 |
| RLP pregrasp / grasp / retreat | すべてSUCCESS、位置誤差 `6.6 mm` / `6.0 mm` / `14.0 mm` |
| Sim物理把持 | `attached=true`、退避中の物体移動 `0.240 m` |
| Attached Object | `hand_palm_link`へ追加を確認 |
| 解放 | Attached Object削除、free形状復元、Sim `attached=false` |

この結果で、S5.1の「認識結果をTFで固定座標へ変換し、RLPで把持位置へ移動し、把持中だけAttached Objectとして扱い、解放後にfree形状へ戻す」経路を確認できた。

#### S5.2実行記録（2026-09-08）

S5.1と同じSimを`GRASP=1`で起動し、`--with-obstacle`を付けてS4の`one_side_pass`相当のBox（`s5_side_obstacle`、`odom`中心 `(0.55, 0.25) m`、寸法 `(0.30, 0.40, 0.30) m`）を追加した。BoxはRLPの`CollisionObject`とSimのPhysX colliderへ同じsequenceで登録し、appleのfree形状を削除してAttached Objectへ切り替えた状態で退避した。結果は**1/1 PASS**である。

| 確認項目 | 結果 |
| --- | --- |
| 障害物の同期 | RLP/Simとも登録ack、解放後に削除ack |
| RLP pregrasp / grasp / retreat | すべてSUCCESS、位置誤差 `14.1 mm` / `13.9 mm` / `15.5 mm` |
| Attached Object搬送 | `hand_palm_link`への追加後、退避中にSim物体が `0.248 m` 移動 |
| 物理接触 | 障害物との接触なし |
| 解放後 | Attached Object削除、free形状復元、Sim `attached=false` |

#### S5.3実行記録（2026-09-08）

S5.1と同じ`SCENE=rlp`、`GRASP=1`、`validation_thread_num=8`で、`--place-on-table`を付けて再実行した。机は競技家具ではなく、RLPの`CollisionObject`とSimのPhysX slotへ同じ`odom`ワールド座標で登録する検証用スラブとした。把持・退避が完了してから机を登録するため、把持へ向かう台車経路の接触を机上配置の評価に混ぜていない。

| 確認項目 | 結果 |
| --- | --- |
| 机の同期 | RLP/PhysXともsequence ack。中心 `(0.820, 0.250) m`、寸法 `(0.35, 0.30, 0.04) m`、底面 `z=0.30 m` |
| 把持・Attached登録・退避 | すべてRLP planner SUCCESS、Sim `attached=true`、退避中の物体移動 `0.240 m` |
| 机上への安全搬送 | 高位置→水平移動→下降前→置き姿勢の全stepがSUCCESS |
| Attached Objectの衝突検証 | 物体底面を机へ2 cm侵入させるprobeが`planner_status=-4 (VALIDATION_FAILURE)`で即時拒否 |
| 把持中の机とのクリアランス | 最小垂直クリアランス `0.0968 m`、貫通なし |
| 開放後の物体 | 最終位置 `(0.818, 0.336, 0.339) m`、机の水平範囲内、`placed_on_table=true` |
| 台車本体の物理接触 | なし |
| 試行全体 | **1/1 PASS** |

開放後の`max_penetration_m`は接触安定化に伴う約`1.2 mm`で、runnerの数値許容`5 mm`以内だった。物体の机上判定は、`SCENE=rlp`のYCB appleで剛体原点を接触基準として扱うSim固有の仮定を含むため、実機移行時は物体寸法・把持点・机面高さを実測値へ置き換える。重要なのは、Attached Objectを付けた状態ではprobeがRLP validatorで拒否され、実際の机上搬送では物体が机へ侵入せず、開放後も机上に残ったことである。再実行でも同じ合格条件を満たした。

これでS5の最小構成（認識→TF→把持→Attached Object→障害物統合→机上配置→解放）まで完了した。次のS6では、まず1個の点群障害物でBridgeAの最小経路を確認し、その後に移動・消失する障害物をオンライン更新する。実機へ移る前には、`GRASP=1`のSim補助に依存しない把持判定と、S3.5で未確定の実機固有パラメータを実機側で再校正する。

### S6: `pcl_reconst`を使ったBridgeAと動的障害物

S6では、認識結果をRLPへ直接渡すのではなく、BridgeAが`hma_pcl_reconst2`の`PointCloud2`を受けて、RLPが扱える`CollisionObject`へ変換する経路を検証する。RLP本体とSimは責務を分け、BridgeAはこのリポジトリの検証runnerとして実装する。

- 入力は`/hma_pcl_reconst/depth_registered/points`（frameは通常`head_rgbd_sensor_rgb_frame`）とする。Simが直接発行する`/head_rgbd_sensor/depth_registered/points`はS6のBridgeA入力に使わない。
- BridgeAは点群をメッセージ時刻のTFで`odom`へ変換し、時刻が一致しない場合だけ最新TFへフォールバックする。`odom`のROI、HSR本体の半径、床付近の点を除外し、2D voxelの連結成分から最大の立体クラスタをBoxへ形状化する。
- 形状化したBoxは`collision_environment_server/collision_object`へ`ADD`として周期的に送り、物体を検出できない状態が`stale_timeout`続いたら同じIDを`REMOVE`する。BridgeAはSim真値トピックを購読して形状を作らない。
- `SCENE=rlp_dynamic`では、Isaac Sim起動時に`/World/RLPDynamicObstacle/box`をcolliderとして先に登録し、シミュレーション中はtransformを更新してY方向へ移動させる。一定時間後は遠方へ移動して非表示にするため、点群欠落とstale削除も同じ試行で確認できる。
- `/rlp_validation/dynamic_obstacle_truth`は検出誤差を計算するためだけのSim真値であり、BridgeAの入力ではない。`/rlp_validation/dynamic_obstacle_contact`はHSRと動的Boxの接触を検出する試験用oracleである。

#### S6.1: 静止時の点群→Box化

まずベースゴールを送らず、観察姿勢で点群を受けて`CollisionObject`を1個登録する。最初の検証では、Sim真値とBridgeAの中心誤差が`0.20 m`以下、RLP環境subscriberへの接続、点群処理、`ADD`の発行を確認する。その後、Sim reset時に同じ検出が再現することを確認する。

#### S6.2: 移動・消失するBoxのオンライン更新

動的Boxが見えている状態で`odom=(1.20, 0.00)`へのベースゴールを開始し、ゴール実行中にもBridgeAの更新を継続する。次の全項目を満たした場合だけS6.2をPASSとする。

- 点群subscriberと`collision_environment_server/collision_object` subscriberが接続している。
- Sim真値がvisibleになった後、BridgeAがBoxを検出し、同時刻近傍の中心誤差が`0.20 m`以下である。
- 動的BoxのY移動に対して、BridgeAの`ADD`更新が2回以上発行され、検出中心のY差が`0.20 m`以上になる。
- Sim真値がhiddenになった後、`stale_timeout=1.0 s`以内の検出欠落を経て、BridgeAが`REMOVE`を発行する。
- ベースゴールはplanner status履歴にSUCCESSを含み、制約終端と物理的な台車収束を満たす。
- 動的BoxとHSRのSim物理接触がない。

実行するターミナルは次の4つである。すべてROSを使うターミナルはApptainer共通手順を通してから実行する。

```bash
# ホスト側
xhost local:
cd /home/hma/carrobo-isaac
make down
make up localhost SCENE=rlp_dynamic

# Apptainer 1: RLP
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true validation_thread_num:=8

# Apptainer 2: BridgeAの入力となる hma_pcl_reconst2
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  use_sim_time:=true use_compressed:=true \
  topic_rgb:=/head_rgbd_sensor/rgb/image_rect_color \
  topic_depth:=/head_rgbd_sensor/depth_registered/image_rect_raw \
  topic_camera_info:=/head_rgbd_sensor/rgb/camera_info \
  output_topic:=/hma_pcl_reconst/depth_registered/points

# Apptainer 3: S6.1 snapshot（ベースゴールなし）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s6_pcl_dynamic_obstacle_runner.py \
  --skip-base-goal --timeout-sec 30.0 --reset-settle-sec 2.0 \
  | tee /tmp/rlp_s6_bridge_a_snapshot.jsonl

# S6.2: 上記のrunnerを終了後、同じSimでベースゴール込み
python3 src/4_manipulation/hsrb_robot_local_planner/tools/s6_pcl_dynamic_obstacle_runner.py \
  --timeout-sec 45.0 --reset-settle-sec 2.0 \
  | tee /tmp/rlp_s6_bridge_a_dynamic_goal.jsonl
```

S6.1は独立Simで1/1 PASSとなった。初期中心誤差は`0.0717 m`、更新回数は15回、検出中心の最大Y差は`0.2278 m`で、stale削除も確認した。S6.2も1/1 PASSとなり、初期中心誤差`0.0766 m`、更新回数13回、最大Y差`0.2216 m`、ゴール後の台車位置誤差`0.0353 m`、yaw誤差`0.0022 rad`、動的Box接触なしであった。なお、runnerの合否は最終表示値1個だけでなく、planner status履歴のSUCCESS、制約終端、controllerの物理収束を合わせて判定している。

#### S6.3a: 静的な机の認識→把持→机上配置

動的障害物を搬送中に動かす前に、配置先の机を実際の認識経路から得て、そこへ把持物体を置けるかを分離して確認する。S6.3aでは競技用の複数部屋・家具を使わず、`carrobo-isaac`の`SCENE=rlp_grasp_table`を使う。

- Simは原点のHSR、`odom=(0.70, 0.00)`付近のYCB apple 1個、正面右側の単純な静的机だけを生成する。机の真値は中心`(0.90, 0.55) m`、天板高さ`z=0.45 m`、寸法`(1.20, 0.35, 0.02) m`とする。
- GroundingDINOの`table` bboxを机検出に使い、`/hma_pcl_reconst/depth_registered/points`をbbox内で切り出す。点群はメッセージ時刻のTFで`odom`へ変換し、平面候補の高さ・MAD・面積から天板を推定する。
- 推定した天板全体を真値で置き換えず、観測中心を中心とした`0.20 m × 0.20 m × 0.03 m`の測定配置パッチだけをRLPの`CollisionObject`として登録する。`/rlp_validation/table_truth`は誤差確認専用で、検出・配置の構築には使わない。
- Simの机は物理コライダとして1つだけ生成する。runnerはそれをPhysXの環境障害物ブリッジへ重複登録せず、退避後にRLPの計画モデルへ配置パッチを追加し、解放後に削除する。この分離により、机の二重生成による台車接触を机上配置の失敗と誤判定しない。
- appleは既存の`yolov8_detection`→`grasp_point_detection`で認識・把持点推定し、TFで`odom`へ変換する。free `CollisionObject`、Sim物理attach、`AttachedCollisionObject`、退避、机上高移動、侵入probe拒否、下降、release、机上残留を一つのrunnerで確認する。
- S6.3の最終配置移動には、RLP既存の`goal_relative_linear_constraint`を適用する。既定値はゴール姿勢の`hand_palm_link`座標系で軸`(0, 0, -1)`、距離`0.20 m`とし、通常配置では上から下、cabinet配置では横向きの前面から奥への最後の直進区間を表す。制約のwaypoint間隔は検証時に`linear_constraint_step:=0.02`とする。

実行時のROSターミナルはすべて、次のApptainer手順を最初に通す。ホスト側のGUI公開も先に行う。

```bash
# ホスト側
xhost local:
cd /home/hma/carrobo-isaac
make down
make up localhost SCENE=rlp_grasp_table GRASP=1

# 以下、各ROSターミナルで共通
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
```

共通手順の後、次を別ターミナルで起動する。

```bash
# Apptainer 1: RLP
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true validation_thread_num:=8 \
  linear_constraint_step:=0.02

# Apptainer 2: BridgeA入力用のhma_pcl_reconst2
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  use_sim_time:=true use_compressed:=true \
  topic_rgb:=/head_rgbd_sensor/rgb/image_rect_color \
  topic_depth:=/head_rgbd_sensor/depth_registered/image_rect_raw \
  topic_camera_info:=/head_rgbd_sensor/rgb/camera_info \
  output_topic:=/hma_pcl_reconst/depth_registered/points

# Apptainer 3: 机検出用GroundingDINO
ros2 launch hma_grounding_dino2 grounding_dino2_lifecycle_service.launch.py \
  use_sim_time:=true rgb_topic:=/head_rgbd_sensor/rgb/image_rect_color

# Apptainer 4: apple検出と把持点推定
ros2 launch yolov8_detection yolov8_detection_launch.py use_rviz:=false
ros2 run grasp_point_detection grasp_point_service \
  --ros-args -p min_points:=10 -p mask_erosion_px:=3

# Apptainer 5: S6.3a runner
python3 src/4_manipulation/hsrb_robot_local_planner/tools/\
s6_3_table_recognition_grasp_place_runner.py \
  --timeout-sec 55.0 --release-settle-sec 2.0 \
  | tee /tmp/rlp_s63a_static_table_grasp_place.jsonl
```

runnerには直接指定しなくても、静的机のS6.3a runnerの既定値として机観察pan`0.80 rad`、配置パッチ`0.20 m`、解放前クリアランス`0.015 m`、高位置オフセット`0 m`が入っている。cabinetとS6.3bでは配置パッチを`0.24 m`とする。机だけを切り出して確認する場合は、最後のコマンドに`--table-only`を追加する。

#### S6.3a実行記録（2026-09-09）

机だけの検出試行は**1/1 PASS**だった。GroundingDINOのbbox、PCL点群の`head_rgbd_sensor_rgb_frame`→`odom`変換、天板平面選択、配置パッチ生成を確認し、Sim真値は判定にのみ使った。

| 確認項目 | 結果 |
| --- | --- |
| 机検出 | GroundingDINO `table`、score `0.426`〜`0.486` |
| 点群天板推定 | `top_z=0.45035 m`、観測中心は試行により`(0.688, 0.484)`〜`(0.705, 0.480) m` |
| RLP配置パッチ | 観測中心の`0.24 m × 0.24 m × 0.03 m`。Sim真値は照合専用、`truth_used_for_construction=false` |
| apple認識・TF・把持 | YOLO score `0.831`、把持点を`odom`へ変換、Sim `attached=true` |
| Attached Object搬送 | 退避、机上高への水平移動、下降前姿勢、置き姿勢がすべてplanner SUCCESS・物理収束 |
| 机との衝突検証 | 物体底面を机へ`0.02 m`侵入させるprobeを`VALIDATION_FAILURE(-4)`で拒否 |
| 把持中の机上クリアランス | 最小物理クリアランス`2.21 mm`、貫通なし |
| 解放後 | `physical_sim_release=true`、`placed_on_table=true`、机上中心内、計画用机パッチ削除成功 |
| 台車の物理障害物接触 | `false` |
| 試行全体 | **1/1 PASS** |

開放後のappleは物理シミュレーション上で天板へ接触して静止するため、机上判定では剛体原点を接触基準とするYCB apple固有のモデル仮定と`5 mm`の数値許容を使う。これは搬送中の非接触条件とは別の「解放後に机上へ残る」判定である。実機移行時は、物体寸法、把持点、机面高さ、接触・滑りを実測して置き換える。

#### S6.3a cabinet: 上棚付き配置面での衝突回避

通常の机上配置が通った後、配置面の真上に段があるキャビネット状の環境で同じフローを確認する。`SCENE=rlp_grasp_cabinet`は、S6.3aの物理机へ静的な上棚を1つ追加する。上棚は中心`(0.90, 0.62) m`、下面`z=0.72 m`、寸法`(0.90, 0.24, 0.05) m`とし、配置面の投影範囲と重なる、実際に棚下へ置く条件にする。

- 上棚はSim起動時にPhysX colliderとして生成し、実行中に物理形状を追加・削除しない。
- runnerは上棚を`odom`の静的`CollisionObject`としてRLPへ登録するが、Simの物理環境bridgeへ重複登録しない。
- 机の高さ・観測パッチ寸法はGroundingDINOと`pcl_reconst`から推定し、配置用の測定パッチだけを棚の投影中心へ移す（`measured_plane_shelf_projection`）。Sim truthは照合専用で、配置目標の構築には使わない。
- appleの把持までは従来のトップダウン姿勢で行う。把持後は棚から離れた位置で手先を横向き（roll`=-π/2`）へ変更し、物体―手先オフセットも同じ回転で補正する。
- 物体を棚前面（`-Y`側）より手前へ水平搬送し、棚下面を通る区間は横向きのまま`+Y`方向へ挿入する。棚下で上から降ろす軌道は使用しない。解放前は`0.045 m`の側方配置リフトを残し、解放後に物体が机上へ着地する。
- 上棚内部へのAttached Object目標と机へのAttached Object侵入probeは、ともに`VALIDATION_FAILURE(-4)`で拒否されなければならない。実際の配置では、棚とのSim接触がなく、把持中の物体が机へ侵入せず、解放後に物体が机上へ残ることを確認する。

実行手順は、S6.3aの共通Apptainer手順を完了した後、ホスト側で次を起動する。

```bash
# ホスト側
xhost local:
cd /home/hma/carrobo-isaac
make down
make up localhost SCENE=rlp_grasp_cabinet GRASP=1

# Apptainer側はS6.3aと同じ5ターミナルを起動
python3 src/4_manipulation/hsrb_robot_local_planner/tools/\
s6_3a_cabinet_grasp_place_runner.py \
  --timeout-sec 55.0 --release-settle-sec 2.0 \
  | tee /tmp/rlp_s63a_cabinet_static.jsonl
```

runnerには`--cabinet-place-under-shelf`、`--cabinet-side-insertion`、側方配置リフト`0.045 m`が既定で入っている。確認したい場合は次のように明示できる。

```bash
python3 src/4_manipulation/hsrb_robot_local_planner/tools/\
s6_3a_cabinet_grasp_place_runner.py \
  --cabinet-place-under-shelf --cabinet-side-insertion \
  --cabinet-side-place-lift 0.045 \
  --timeout-sec 55.0 --release-settle-sec 2.0 \
  | tee /tmp/rlp_s63a_cabinet_side_insertion.jsonl
```

#### S6.3a cabinet旧仕様の実行記録（2026-09-09、受入条件から除外）

初期実装では安全な配置点を棚の前方へ逃がしていたため、`y=0.70 m`へ棚を移動した条件で**1/1 PASS**となった。しかしこれは「棚の真下へ置く」検証になっていなかったため、S6.3a cabinetの合格記録としては採用しない。棚を配置面の投影上へ戻し、水平挿入シーケンスへ置き換えた。

| 確認項目 | 結果 |
| --- | --- |
| 上棚の物理生成 | center`(0.90, 0.70) m`、bottom`z=0.62 m`、size`(0.90, 0.16, 0.05) m` |
| 判定 | 棚下への直接配置を評価していないため、旧仕様として保留 |

#### S6.3a cabinet水平挿入の実行記録（2026-09-09）

棚下面`z=0.72 m`の既定条件で、検出した机パッチを棚中心`(0.90, 0.62) m`へ合わせ、把持後に横向きへ変更して棚下へ水平挿入した。最終回帰は**1/1 PASS**である。

| 確認項目 | 結果 |
| --- | --- |
| 机認識・配置目標 | GroundingDINO + `pcl_reconst`、検出`top_z=0.45035 m`、配置中心を棚投影`(0.90, 0.62) m`へ設定 |
| apple認識・TF・把持 | YOLO→把持点推定→`odom` TF、Sim `attached=true` |
| 横向き姿勢 | roll`=-π/2`、quaternion`[-0.7071, 0, 0, 0.7071]` |
| 水平挿入 | 棚前の物体目標`y=0.44 m`から棚下中心へ`+Y`挿入。手先目標は前面側`y≈0.509 m` |
| 物体―手先オフセット | 把持時`[0.0184, 0.0040, -0.1112]`を横向き時`[0.0184, 0.1112, 0.0040] m`へ回転補正 |
| 上棚衝突probe | Attached Objectの棚貫通目標を`VALIDATION_FAILURE(-4)`で拒否 |
| 机侵入probe | Attached Objectの机貫通目標を`VALIDATION_FAILURE(-4)`で拒否 |
| 把持中の机上クリアランス | 最小`50.2 mm`、貫通なし |
| 上棚との物理接触 | `cabinet_physical_contact=false` |
| 解放後 | `physical_sim_release=true`、`placed_on_table=true`、机上中心内 |
| 試行全体 | **1/1 PASS** |

ログは`/tmp/rlp_s63a_cabinet_side_insertion_20260909_v7_bottom072.jsonl`に保存した。S6.3a cabinetの水平挿入回帰が成立したため、次はこの配置シーケンスをS6.3bのBridgeAオンライン更新と組み合わせる。

#### S6.3b: 動的障害物と把持・机上配置の統合

S6.3bでは、S6.1/S6.2のBridgeAをS6.3aの認識→TF→把持→Attached Object→机上配置→解放へ統合する。動的BoxはSim側でPhysX colliderとして起動時に登録し、`hma_pcl_reconst2`の点群からBridgeAが`odom`の`CollisionObject`をオンライン更新する。Sim真値は判定専用で、検出形状の構築には使わない。

`SCENE=rlp_dynamic_grasp`のBoxは中心`x=0.80 m`、初期`y=-0.30 m`付近からY方向へ移動し、周期`75 s`・visible`55 s`で遠方へ退避する。机とappleを同じシーンへ置くため、S6.3bのBridgeAは負のy領域と立体形状の条件で動的Boxだけを選ぶ。CollisionObjectの更新周期は`1 s`とし、点群処理のレートを保ちながらRLPの長いwhole-bodyゴールを過剰に無効化しない。

合格条件は次のすべてとする。

- `/hma_pcl_reconst/depth_registered/points`を受信し、BridgeAが動的Boxを検出する。
- Sim truthとの初期中心誤差が`0.20 m`以下である。
- 動的Boxの移動に対してBridgeAの更新が2回以上あり、検出中心のY差が`0.20 m`以上である。
- appleを認識して`odom` TFを出し、把持、Attached Object登録、退避、机上配置、解放が成功する。
- Attached Object登録後にもBridgeA更新が1回以上ある。
- Sim truthがhiddenになった後、BridgeAが`pointcloud_stale`の`REMOVE`を発行する。
- 最終配置の`place_pose_with_attached_object`が`goal_relative_linear_constraint`（距離`0.20 m`、軸`(0, 0, -1)`）付きでplanner SUCCESS・物理収束する。
- 動的Box、机、上棚とのSim物理接触がない。

実行手順は、ROSを使う各ターミナルで次の共通手順を守る。RLPのlaunchには`validation_thread_num:=8`を指定する。

```bash
# ホスト側
xhost local:
cd /home/hma/carrobo-isaac
make down
make up localhost SCENE=rlp_dynamic_grasp GRASP=1

# Apptainer側（RLP）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hsrb_robot_local_planner_node \
  hsrb_robot_local_planner.launch.py use_sim_time:=true validation_thread_num:=8 \
  linear_constraint_step:=0.02

# Apptainer側（hma_pcl_reconst2）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hma_pcl_reconst2 pcl_reconst.launch.py \
  use_sim_time:=true use_compressed:=true \
  topic_rgb:=/head_rgbd_sensor/rgb/image_rect_color \
  topic_depth:=/head_rgbd_sensor/depth_registered/image_rect_raw \
  topic_camera_info:=/head_rgbd_sensor/rgb/camera_info \
  output_topic:=/hma_pcl_reconst/depth_registered/points

# Apptainer側（GroundingDINO）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch hma_grounding_dino2 grounding_dino2_lifecycle_service.launch.py \
  use_sim_time:=true rgb_topic:=/head_rgbd_sensor/rgb/image_rect_color

# Apptainer側（YOLO + 把持点）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
ros2 launch yolov8_detection yolov8_detection_launch.py use_rviz:=false
ros2 run grasp_point_detection grasp_point_service \
  --ros-args -p min_points:=10 -p mask_erosion_px:=3

# Apptainer側（S6.3b runner）
cd ~/carrobo26_ws
bash 0_shell.sh
. /entrypoint.sh
source install/setup.bash
. 5e_isaac_mode.sh
python3 src/4_manipulation/hsrb_robot_local_planner/tools/\
s6_3b_dynamic_grasp_place_runner.py \
  --timeout-sec 70.0 --release-settle-sec 2.0 \
  --dynamic-hide-timeout-sec 120.0 \
  | tee /tmp/rlp_s63b_dynamic_grasp_place.jsonl
```

#### S6.3b実行記録（2026-09-09）

動的障害物を含む把持・配置統合は**1/1 PASS**となった。

| 確認項目 | 結果 |
| --- | --- |
| BridgeA入力 | `/hma_pcl_reconst/depth_registered/points`接続成功 |
| 動的Box初期検出 | truthとの中心誤差`0.0741 m` |
| オンライン追跡 | 更新`45回`、検出中心Y差`0.3454 m` |
| 把持中のオンライン更新 | Attached Object登録後`22回` |
| 認識→TF→把持→配置 | YOLO apple、`odom` TF、Sim attach、机上releaseまで成功 |
| hidden後のstale削除 | truth hidden、BridgeA `pointcloud_stale REMOVE`成功 |
| 物理接触 | 動的Boxとの接触なし。机上配置の貫通なし |
| 試行全体 | **1/1 PASS** |

ログは`/tmp/rlp_s63b_dynamic_grasp_place_retry2_20260909.jsonl`に保存した。S6.3bまでの最小回帰が成立したため、次は複数クラスタ、意図的な認識位置ずれ、部分遮蔽、点群レート低下、更新遅延の分布を追加する。これらとS5/S6.3aの回帰が安定するまでは実機へ移行しない。

#### S6.3b再実行記録（2026-09-09、Attached Object probe修正版）

前回の実行条件をクリーンスタートで再実行した。通常のトップダウン配置では、RLPの`Attached Object`が保持するplanner/TF基準の物体―手先Zオフセット（`-0.050 m`）で机貫通probeを作る必要がある。Simの把持oracleが返す物理物体原点オフセット（約`-0.111 m`）をそのprobeに使うと、Sim上の物理形状確認には適していても、RLPの検証目標とフレームが一致せず、誤ってprobeを通してしまう。このフレームを修正した回帰は**1/1 PASS**となった。

| 確認項目 | 結果 |
| --- | --- |
| BridgeA入力 | `/hma_pcl_reconst/depth_registered/points`接続成功 |
| 動的Box初期検出 | truthとの中心誤差`0.0741 m` |
| オンライン追跡 | 更新`43回`、検出中心Y差`0.3330 m` |
| 把持中のオンライン更新 | Attached Object登録後`21回` |
| 認識→TF→把持→配置 | YOLO apple（score`0.839`）、`odom` TF、Sim attach、机上releaseまで成功 |
| Attached Objectの机貫通probe | planner status`-4`（`VALIDATION_FAILURE`）で拒否 |
| 配置中の机上クリアランス | 最小`3.35 mm`、貫通なし。`placed_on_table=true` |
| hidden後のstale削除 | truth hidden、BridgeA `pointcloud_stale REMOVE`成功 |
| 物理接触 | 動的Box・机との接触なし |
| 試行全体 | **1/1 PASS** |

ログは`/tmp/rlp_s63b_dynamic_grasp_place_20260909_rerun2.jsonl`に保存した。なお、GroundingDINO/PCLの机検出パッチは観測中心ベースで生成するため、今回のログではSim truthとの机中心照合は合格条件に含めていない（`truth_check.pass=false`）が、実際のRLP配置では机上中心内・貫通なし・解放後残留を確認した。S6.3bの最小統合回帰は修正版でも成立したため、次は複数クラスタ、意図的な認識位置ずれ、部分遮蔽、点群レート低下、更新遅延の分布を追加する。これらとS5/S6.3aの回帰が安定するまでは実機へ移行しない。

同条件の追加再実行も**1/1 PASS**となった。初期中心誤差`0.0742 m`、BridgeA更新`46回`（Attached Object登録後`23回`）、検出中心Y差`0.3407 m`、机上配置中の最小クリアランス`4.76 mm`、Attached Objectの机貫通probeはplanner status`-4`で拒否、物理接触なしであった。ログは`/tmp/rlp_s63b_dynamic_grasp_place_20260909_rerun3.jsonl`に保存した。

#### S6.3最終直進制約の回帰（2026-09-09）

S6.3の最終配置ステップへ`goal_relative_linear_constraint`を追加し、RLPを`validation_thread_num:=8 linear_constraint_step:=0.02`で再起動して、静的机・上棚付き机・動的障害物の3条件を再実行した。runnerの既定値は`end_frame_id=hand_palm_link`、goal姿勢基準の軸`(0, 0, -1)`、距離`0.20 m`である。通常の机では上から下、cabinetではroll`=-π/2`の横向き姿勢で`+Y`へ挿入する最後の区間に対応する。

| 条件 | 結果 | 最終配置制約 | 追加確認 |
| --- | --- | --- | --- |
| S6.3 静的机 | **1/1 PASS** | `0.20 m`、`(0,0,-1)`、`hand_palm_link` | 机侵入probe`-4`、最小搬送中クリアランス`2.85 mm`、解放後机上残留、物理接触なし |
| S6.3a cabinet | **1/1 PASS** | `0.20 m`、`(0,0,-1)`、`hand_palm_link` | 棚内probe`-4`、机侵入probe`-4`、棚下面の予測余裕`106.2 mm`、搬送中最小クリアランス`51.1 mm`、物理接触なし |
| S6.3b BridgeA | **1/1 PASS** | `0.20 m`、`(0,0,-1)`、`hand_palm_link` | BridgeA更新`47回`（Attached後`22回`）、検出Y移動`0.3438 m`、stale削除、最小搬送中クリアランス`0.26 mm`、物理接触なし |

cabinetでは、棚前から最終姿勢までの距離が制約距離を下回らないよう、棚前の物体開始位置を`y=0.42 m`へ設定した。これにより、最終hand目標`y=0.5086 m`との差が`0.20 m`となり、横方向の最終直進区間を確保できた。ログはそれぞれ`/tmp/rlp_s63a_static_table_grasp_place_20260909_linear02.jsonl`、`/tmp/rlp_s63a_cabinet_linear02_20260909_rerun2.jsonl`、`/tmp/rlp_s63b_dynamic_grasp_place_20260909_linear02.jsonl`に保存した。

## 実装成果物の予定

### このリポジトリ

- Sim用のRLP launch/パラメータ設定
- S1〜S3の固定コマンド・回帰テストスクリプト（S3: `tools/s3_tuning_runner.py`）
- S3.5のハードウェアモデル監査・基準動作スクリプト（`tools/s35_hardware_model_runner.py`）
- `planner_status` / `displacements` / 実行時間の記録
- S4の `CollisionObject` publisher・Sim PhysX scene bridge・固定Boxシナリオ・回避/到達不能回帰（`tools/s4_obstacle_runner.py`）
- S5.1/S5.2/S5.3の認識→TF→把持→Attached Object・障害物/机上配置統合回帰（`tools/s5_recognition_grasp_runner.py`）
- S5のAttached Object publisher、衝突除外ペア、机上貫通probeの管理
- S6.1/S6.2の`pcl_reconst`→BridgeA→`CollisionObject`、移動追従・stale削除・オンラインゴール回帰（`tools/s6_pcl_dynamic_obstacle_runner.py`）
- S6.3aのGroundingDINO→`pcl_reconst`天板推定→apple把持→Attached Object搬送→机上配置回帰（`tools/s6_3_table_recognition_grasp_place_runner.py`）
- S6.3a cabinetの上棚付き配置面・上棚侵入probe・物理接触oracle回帰（`tools/s6_3a_cabinet_grasp_place_runner.py`）
- S6.3bのBridgeAオンライン追跡と認識→TF→把持→机上配置→hidden後stale削除回帰（`tools/s6_3b_dynamic_grasp_place_runner.py`）

### `carrobo-isaac`

- `worlds/rlp_validation.world`: 競技用とは独立した最小ワールド
- `configs/placement.rlp_empty.yaml`: S0〜S2用の空配置設定
- `configs/placement.rlp.yaml`: 正面対象物だけを置く配置設定
- RLP用world/configの切替機構
- S5.1用の`GRASP=1`物理把持状態publisher（`/rlp_validation/grasp_state`）
- S3.5用のHSR-B台車寸法・速度上限の環境変数（`BASE_WHEEL_SEPARATION`、`BASE_CASTER_OFFSET` など）
- S4用の静的障害物slot（`/World/RLPObstacles/slot_0`〜`slot_7`）とROS bridge
- S5.3用の動的な机上スラブ反映（RLP/Sim共通の`odom`ワールド座標）
- S6.3a用の静的な手続き生成机、上棚、机truth/接触oracle、`SCENE=rlp_grasp_table`/`SCENE=rlp_grasp_cabinet`
- S6.3b用の`SCENE=rlp_dynamic_grasp`、起動前登録済みの移動Box、真値・接触oracle topic
- S6用の`SCENE=rlp_dynamic`、起動前登録済みの移動Box、真値・接触oracle topic

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
