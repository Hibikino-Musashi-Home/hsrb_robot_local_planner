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
| 環境障害物のRLP向け形状化 | 後半で追加するscene bridge |
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
```

RLPノードは別ターミナルで、Apptainerへ入ってからこのリポジトリのbuildを使い、`use_sim_time:=true` を付けて起動する。S0〜S2では `placement.rlp_empty.yaml`、S3では `placement.rlp.yaml` が使われる。

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
| S4: 静的環境障害物 | `PlanningSceneWorld.collision_objects` の入力と回避経路。衝突検証の安全マージン確認 | 箱または円柱を1個。まず横に回避可能な配置 | Sim上の障害物とRLPへ送った形状が一致し、接触せず回避。障害物を完全に塞いだ場合は失敗を正しく返す |
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

### S4: 静的障害物

Isaac Simの見た目・物理衝突と、RLPの衝突判定用形状を同じ寸法・同じ `odom` 姿勢で管理する。RLP側へは点群やOctomapを直接送らず、少数のBox/Cylinderプリミティブへ変換して送る。

最低限、次の3ケースを用意する。

- 回避可能: 対象物への経路の横に箱を置く
- 経路選択: 前方を部分的に塞ぎ、左右どちらかに抜けられるようにする
- 到達不能: 前方を完全に塞ぎ、RLPが失敗を返すことを確認する

最初から薄い板や複雑なメッシュを使わない。衝突検証のサンプリング間隔や計算時間の問題と、形状の問題を分離できなくなるためである。

### S5: 把持物体

把持物体は、把持が確定したイベントで一度だけAttached Objectへ切り替える。物体を `hand_palm_link` に対する相対姿勢で表し、指先との接触ペアは必要なものだけ無効化する。解放時には空の `attached_collision_objects` を明示的に送る。

検証順は「把持前の接近 → attach → 障害物近傍を退避 → 解放 → attach解除」とする。把持物体を毎周期の認識結果として更新する処理はS6まで入れない。

## 実装成果物の予定

### このリポジトリ

- Sim用のRLP launch/パラメータ設定
- S1〜S3の固定コマンド・回帰テストスクリプト（S3: `tools/s3_tuning_runner.py`）
- `planner_status` / `displacements` / 実行時間の記録
- S4の `PlanningSceneWorld` publisherまたはbridge
- S5のAttached Object publisherと衝突除外ペアの管理

### `carrobo-isaac`

- `worlds/rlp_validation.world`: 競技用とは独立した最小ワールド
- `configs/placement.rlp_empty.yaml`: S0〜S2用の空配置設定
- `configs/placement.rlp.yaml`: 正面対象物だけを置く配置設定
- RLP用world/configの切替機構
- 後段で使う静的障害物シナリオ

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

## 参照

- [RLPパラメータ調査・チューニングガイド](./robot_local_planner_tuning_guide.md)
- [Validatorの入力と環境障害物/Attached Object](./robot_local_planner_tuning_guide.md#入力の実体--障害物と把持物体はどこから来るか)
