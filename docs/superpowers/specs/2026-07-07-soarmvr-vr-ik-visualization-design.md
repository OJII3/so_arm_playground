# SoArmVR: VR 側で IK・可視化・JointTrajectory を完結させる設計書

- 日付: 2026-07-07
- 対象: `SoArmVR/`, `ros2_ws/src/teleop_ik/`, `ros2_ws/src/lerobot_description/`
- ブランチ: (未作成, 実装着手時に切る)

## 1. 背景とゴール

現状, SO-101 の VR テレオペは次の経路で動いている:

```
[SoArmVR (Unity)]
  → TargetPoseWithInput (stick, ik_active, アンカー基準 relative pose)
[teleop_ik_node (C++ / Pinocchio)]
  → Unity→ROS 座標変換
  → セッション基準化 (active / unity_anchor)
  → joint 1..3 位置 CLIK + joint 4/5 FK 注入
  → JointTrajectory publish
[follower controller_manager]
```

- IK / 座標変換 / セッション基準化 / stick 積分がすべて ROS 2 側にある.
- AR 空間上に「現在どんな姿勢になっているか」「EE が IK で到達できたか」
  のフィードバックが無く, VR 側 (= ユーザ) は盲目で操作している.
- コントローラの入力と「バーチャル SO-101 モデル」の関係がすべて ROS 側
  のパラメータで表現されており, VR 側の使い心地を Unity だけで調整できない.
- `ResetCommand` (`/teleop/reset`) は, 設置/再配置と無関係に「A で home に戻す」
  用途で追加されたが, 本設計で再配置動線を A に集約するため削除する.

これに対し, 次を要望する:

- IK 解, 関節角, AR 上のゴースト SO-101 表示を SoArmVR 側で完結させる.
- ROS 2 側は, 受け取った JointTrajectory を `ros2_control` / `lerobot_controller`
  へ渡す薄い bridge だけにする.
- 床面と AR 平面に揃えて「バーチャル SO-101」を設置する動線 (A ボタン)
  を持つ.
- `ResetCommand` (`/teleop/reset`) と `teleop_ik_node` の IK / stick 統合 /
  座標変換は段階的に廃止する.
- 既存 `teleop_ik/gamepad_teleop_node` は触らない (ゲームパッドは別経路).

ゴール:

- SoArmVR 側に `VirtualSoArm` を新設し, 展開済み URDF (`so101.urdf`)
  から kinematic chain を作って FK/IK と AR 表示を行う.
- SoArmVR 側は最終的に `trajectory_msgs/JointTrajectory` を publish する.
- ROS 2 側に `vr_teleop_bridge.launch.py` を新設し, follower 実機/ros2_control
  + 必要ならリマップだけを立ち上げる.
- 既存の `vr_teleop.launch.py` は当面残し, 新経路に切替え可能とする.
- VR 入力から `/teleop/reset` への publish を外す. `teleop_ik::msg::ResetCommand`
  と C++ 側 reset 経路の物理削除は別 spec とする (Step 5, §6).
- 現行 README 類の A ボタン説明 (`Reset`) は A = 床設置 に書き換える.

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| URDF | 展開済み `so101.urdf` を `SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf` に TextAsset として同梱. xacro 展開は SoArmVR 側で行わない. |
| ロボットモデル | URDF から link/joint 階層を `UrdfModel` として読み込み, Unity GameObject ツリーへ transform を反映. mesh は URDF visual mesh を読まずに, 各 link を joint origin 間を結ぶ capsule/cube で可視化し, link 名ごとに固定色を割り当てる. URDF mesh 反映は将来段階. |
| 設置 | A ボタン押下中に右コントローラー ray を AR 平面に raycast し, release 時に確定. 押下時間による分岐はしない. base position は hit pose.position, base up は `Vector3.up`, base forward は右コントローラー forward を床平面へ投影した方向とする. 投影が小さい (`sqrMagnitude < 1e-4`) 場合は `XR Origin` の forward へフォールバック. |
| 基準フレーム | 設置後の `base` Transform からの相対で EE 目標と joint 4 pitch を扱う. 既存 `TeleoperationAnchor` (ヨー基準) は今回廃止. |
| 座標変換点 | クラッチ delta は world 空間で取り, `baseTransform.InverseTransformVector` で `deltaBaseUnity` に変換し, さらに固定の `RosToUnity.InverseTransformVector` で `deltaBaseRos` に変換してから `eeTargetLocalRos` に加算する. joint 4 pitch 符号は URDF joint 4 axis (so101.urdf 上 Z 軸回転, 正方向は `ee` 側を上に倒す向き) に合わせて `j4 = -pitch` とする. この符号は初期実装で実機/RViz と一致しない場合は調整する. |
| 操作 | グリップ押下中だけクラッチ式に EE 位置を追従. 押し始めの controller pose と「grip 押下開始時の EE pose」を基準化し, 差分を EE 目標に反映. EE rotation は今回 IK 対象外とし, 保持する. |
| joint 4 | 右コントローラーの forward 方向を `base` 基準の pitch に変換して目標値とする. yaw は j4 には入れない. |
| joint 5 | 右スティック左右で速度積分 (deadzone 0.1 で再マップした x 値を `dt` 積分) し, 関節角度として注入. |
| グリッパ | 右トリガー (0..1) を joint 6 angle にマップ. |
| A ボタン | 設置専用. 押下中ずっとプレビュー, release で確定. 押下時間による分岐はしない. |
| Reset | 本 spec では VR 入力からの `/teleop/reset` publish を外し無効化する. `teleop_ik::msg::ResetCommand` および C++ 側 reset 経路の物理削除は別 spec (Step 5, §6). |
| IK | 数値 IK を C# で実装. 対象は joint 1..3. ターゲットは EE 位置. seed は前回の成功解. 収束失敗時は最終成功解を維持し, ゴーストの材質を「IK 失敗」表現 (半透明 red) にして publish をスキップする. IK 成功時は半透明 cyan. |
| 座標系 | URDF 内部計算は ROS 座標系 (右手系, Z-up) で保持する. Unity 表示は「URDF 座標系の親 Transform」で全 GameObject を包んで `ros → unity` の固定変換を適用する. ROS publish 値は ROS 座標系のまま (URDF と同梱のため ROS 側と一致) 送出する. |
| ROS 2 出力 | `trajectory_msgs/JointTrajectory` 2 本: `/follower/arm_controller/joint_trajectory`, `/follower/gripper_controller/joint_trajectory`. 値は URDF 同梱の ROS 座標系. |
| msg 生成 | `trajectory_msgs/msg/JointTrajectory`, `trajectory_msgs/msg/JointTrajectoryPoint`, `builtin_interfaces/msg/Duration` の C# msg を `rosettadds-genmsg` で生成し `Assets/_SoArmVR/Scripts/GeneratedMsgs` 配下に追加する. |
| QoS | arm/gripper trajectory は BestEffort / Volatile を避け, Reliable / Volatile とする (現 `teleop_ik_node` と同じ reliable publisher). |
| publish 周期 | `Update()` のたびに publish. 失敗フレームは publish しない. |
| 既存 launch | `vr_teleop.launch.py` は当面温存. 新経路は `vr_teleop_bridge.launch.py` として独立. `vr_teleop_bridge.launch.py` は follower 実機/ros2_control だけを起動し `teleop_ik_node` は起動しない. `use_mock:=true` のときは `teleop_ik_node` を起動しない mock 専用 launch (`ros2_ws/src/teleop_ik/launch/vr_teleop_bridge_rviz.launch.py` 等) を取り込む. 既存 `vr_teleop_rviz.launch.py` は `teleop_ik_node` を include しているため, 新経路からは include しない. |

## 3. アーキテクチャ

### 3.1 全体フロー

```
┌──────────────────────────────────────────┐
│ SoArmVR (Unity, AndroidXR / OpenXR)      │
│                                          │
│  XR Origin ── Right Controller           │
│      │                                   │
│      ├─ A hold → ARPlane raycast (床)    │
│      │       └─ release → base を確定     │
│      │                                   │
│      ├─ grip hold → EE クラッチ追従       │
│      │                                   │
│      └─ update ─► VirtualSoArm           │
│                    ├ UrdfModel (joint tree)
│                    ├ FK でゴースト更新    │
│                    ├ IK ソルバ (j1..3)    │
│                    ├ j4 ← controller pitch
│                    ├ j5 ← stick 積分
│                    └ j6 ← trigger        │
│                                          │
│      └─ update ─► VrTrajectorySink        │
│                    └ ROSettaDDS publish  │
│                      /follower/arm_controller/joint_trajectory
│                      /follower/gripper_controller/joint_trajectory
└──────────────────────┬───────────────────┘
                       │ DDS (LAN)
                       ▼
┌──────────────────────────────────────────┐
│ ros2_ws (Jazzy)                          │
│                                          │
│  (remap のみ)                            │
│     /follower/arm_controller/joint_trajectory
│     /follower/gripper_controller/joint_trajectory
│                  │                       │
│                  ▼                       │
│        ros2_control (実機 or RViz mock)  │
└──────────────────────────────────────────┘
```

Unity 側で IK を解き, 結果の関節角を JointTrajectory で送る. ROS 2 側は
「Unity→ROS 座標変換」「IK」「stick 積分」を持たない.

### 3.2 モジュール分割 (SoArmVR 側)

- `UrdfModel` (新規): URDF XML をパースし, link/joint 階層, 静止 origin,
  joint axis, lower/upper limit を `UrdfLink[]` / `UrdfJoint[]` に保持.
  内部表現は ROS 座標系 (右手系, Z-up) のまま. Unity 表示用の `RosToUnity`
  親 Transform とは別レイヤーで扱う.
- `UrdfKinematics` (新規): `UrdfModel` と関節角から
  `Matrix4x4 GetJointTransform(string linkName, double[] q)` を計算.
  ROS 座標系で保持し, 表示時に `RosToUnity` 親で包む.
- `UrdfIKSolver` (新規): joint 1..3 を変数として EE 位置ターゲットを解く
  damped least squares. q_seed を受けて反復, URDF の lower/upper で clamp.
  damping 初期値 `0.01` (既存 ROS 側 `1e-6` とは異なる; 数値安定性優先).
- `ArmPlacementController` (新規): A ボタンの hold/release, AR 平面
  raycast, base Transform 確定, 配置済みフラグ管理. `ARRaycastManager`,
  `ARPlaneManager` への参照を保持.
- `VirtualSoArm` (新規): 配置済み base に対する関節目標と EE 目標を保持.
  `Update()` で `UrdfKinematics` を回し, `RosToUnity` 配下の子 GameObject の
  Transform を更新. `UrdfIKSolver` を呼んで joint 1..3 を求め, 失敗を
  `bool ikSolved` で公開. 子 GameObject は `base`, `shoulder`, `upper_arm`,
  `lower_arm`, `wrist`, `gripper`, `jaw` の 7 個.
- `VrTrajectorySink` (新規, `RosTeleoperationSink` の置換): 関節角を
  `trajectory_msgs/JointTrajectory` メッセージに詰めて ROSettaDDS で
  publish. `ResetCommand` publisher は削除.
- `ArmTeleoperationController` (新規): 入力 (grip, trigger, stick, j4 pitch)
  を `VirtualSoArm` の目標へ反映. 既存 `TeleoperationSession` を置換.

### 3.3 モジュール分割 (ROS 2 側)

- `ros2_ws/src/teleop_ik/launch/vr_teleop_bridge.launch.py` (新規):
  `use_mock:=false` のとき `so101_follower_controller.launch.py` だけを取り込む.
  `use_mock:=true` のときは `vr_teleop_bridge_rviz.launch.py` を取り込む
  (この launch は `vr_teleop_rviz.launch.py` から `teleop_ik_node` 部分を除
  き, mock hardware + controller + RViz だけを残した新 launch).
  どちらも `teleop_ik_node` は起動しない. launch 引数 `arm_topic`,
  `gripper_topic` で ROS topic 名を remap できる.
- `ros2_ws/src/teleop_ik/msg/ResetCommand.msg`: 本 spec では削除しない
  (Step 5). VR 側からの publish を切るのみ.
- `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`: 削除しない.
  gamepad 経路と既存 C++ テストが依存.
- `teleop_ik_node`: `vr_teleop_bridge.launch.py` からは起動しない.
  既存 launch テストは温存.

### 3.4 既存コードへの影響

| 既存 | 扱い |
| --- | --- |
| `Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs` | 廃止. A ボタン設置に置換. |
| `Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs` | 廃止. `ArmTeleoperationController` に置換. |
| `Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs` | `VrTrajectorySink` に置換. ResetCommand publisher を削除. |
| `Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs` | 廃止. 旧 Sink 抽象は SoArmVR 内に閉じないため. |
| `Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions` | Reset を削除. A は「PlaceArm」にリネーム. grip/trigger/stick は継続. IkActive は削除 (常時 IK). |
| `Assets/_SoArmVR/Prefabs/Anchor.prefab` | 廃止. |
| `Assets/_SoArmVR/Prefabs/Teleoperation.prefab` | 新規スクリプトに付け替え. AR Session / XR Origin / `ARRaycastManager` / `ARPlaneManager` を保持. 追加: `ArmPlacementController`, `ArmTeleoperationController`, `VirtualSoArm`, `VrTrajectorySink`. |
| `Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab` (新規) | `RosToUnity` 親 + `base`, `shoulder`, `upper_arm`, `lower_arm`, `wrist`, `gripper`, `jaw` の GameObject ツリー. 各 link は URDF joint origin 間を結ぶ capsule/cube. link 名ごとに固定色を割当. URDF visual mesh は読まない. |
| `ros2_ws/src/teleop_ik/...` | 本 spec では縮退しない. `vr_teleop_bridge.launch.py` 追加のみ. |

## 4. 詳細設計

### 4.1 設置 (A ボタン)

- `ArmPlacementController.Update()`:
  - A 押下中: 毎フレーム `ARRaycastManager.Raycast` で右コントローラー forward
    を AR 平面 (Plane alignment = Horizontal up) に対して raycast.
  - hit があれば `previewBase` を hit pose に合わせて可視化.
  - A 離した瞬間に, hit pose を `VirtualSoArm.Place(basePose)` に渡す.
    `basePose.position` = hit pose.position,
    `basePose.up` = `Vector3.up` (床平面の normal),
    `basePose.forward` = `Vector3.ProjectOnPlane(rightController.forward, Vector3.up)`.
    投影が `sqrMagnitude < 1e-4` のときは `XR Origin` の forward へフォールバック.
  - 既に配置済みの場合, 「再配置する」動線も許可 (1 度目と同じ挙動,
    IK 状態はリセット).
- AR 平面が取得できないときは何もしない (プレビュー/設置もスキップ).
  視認できる HUD で「床平面を検出中」と出す.

### 4.2 クラッチ式 EE 追従 (右グリップ)

- `ArmTeleoperationController`:
  - 配置済み (`VirtualSoArm.IsPlaced == true`) でないときは grip を無視.
  - grip 押下開始時:
    - `controllerStartPose` = 現在の右コントローラー pose (world)
    - `eeStartLocal` = 「grip 押下開始時の」EE pose (URDF base 基準)
    - `clutchActive = true`
  - grip 押下中:
    - `deltaWorld = controller.position - controllerStartPosition`
    - `deltaBaseUnity = baseTransform.InverseTransformVector(deltaWorld)`
    - `deltaBaseRos = rosToUnity.InverseTransformVector(deltaBaseUnity)`
      (`rosToUnity` は `RosToUnity` 親 Transform. これにより Unity
      base-local の delta を URDF 内部の ROS base-local ベクトルへ変換)
    - `eeTargetLocalRos = eeStartLocalRos + deltaBaseRos * positionScale`
    - EE rotation は今回 IK 対象外のため更新しない (保持).
  - grip 離した: `clutchActive = false`. 最終 EE pose は固定.

### 4.3 joint 4 (controller pitch → base 基準 pitch)

- `controllerForward = rightController.rotation * Vector3.forward`
- `dy = Vector3.Dot(controllerForward, baseTransform.up)` (up との射影)
- `pitch = Mathf.Asin(Mathf.Clamp(dy, -1f, 1f))`
- `j4 = -pitch` とし, URDF の lower/upper で clamp.
  URDF joint 4 の axis は Z 軸回転で正方向が「EE を上に倒す」向きのため,
  コントローラ forward の上向き pitch を負符号で URDF 側へ渡す.
  初期実装で実機 / RViz mock と符号が逆転する場合は本仕様を修正する.
- base に対する yaw は使用しない (要望: pitch のみ).

### 4.4 joint 5 (stick 積分)

- `x = stick.x`
- `deadzone = 0.1`. `abs(x) <= deadzone` のとき 0.
- それ以外は `x' = sign(x) * (abs(x) - deadzone) / (1 - deadzone)`.
- `j5 += x' * j5RadPerSec * dt` (`j5RadPerSec` の default は
  既存 ROS 側 `stick_velocity_scale=1.5` rad/sec と一致).
- lower/upper で clamp.

### 4.5 グリッパ (joint 6)

- `j6 = gripperLowerRad + value * (gripperUpperRad - gripperLowerRad)`
- `gripperLowerRad = -0.174533`, `gripperUpperRad = 1.74533`
  (現 `teleop_ik_node` の値と一致).

### 4.6 IK (joint 1..3, EE 位置)

- `UrdfIKSolver.Solve(eeTargetLocal, qSeed, out qOut)`:
  - 反復: `dq = J^T (J J^T + λ I)^{-1} (eeTarget - eeCurrent)`
  - J は 3x3 (URDF の joint 1..3 に関する位置 Jacobian を差分で近似).
  - `λ = 0.01` (damping). 初期実装は固定 (既存 ROS 側 `1e-6` とは別
    値; 数値安定性優先).
  - `maxIter = 100`, `tol = 1e-4` (現 ROS 側と一致).
  - 各反復で `UrdfKinematics` を呼び, lower/upper で clamp.
  - 収束したら `qOut` を返し `true`. 失敗なら `false`.
- seed: 前回成功解 `qSolution_`. 未配置時/初回は URDF 読み込み時の
  neutral configuration を使う.
- 呼び出し側 (`VirtualSoArm.Update()`) は成功時のみ `qSolution_` を
  今回の `qSolved1_3` で更新する. 失敗時は `qSolution_` を更新しない.

### 4.7 FK とゴースト更新

- `VirtualSoArm.Update()` の順序は次に固定する:
  1. 入力 (`ArmTeleoperationController`) から `eeTargetLocalRos`,
     `j4`, `j5`, `j6` を取得.
  2. `UrdfIKSolver.Solve(eeTargetLocalRos, qSolution_, out qSolved1_3)` を
     呼び, 失敗時は `qSolved1_3` を更新せず IK 失敗フラグを立てる.
  3. `j4`, `j5`, `j6` を `qSolved1_3` に注入して `qCurrent_` を確定.
  4. `UrdfKinematics.GetJointTransform(linkName, qCurrent_)` を全 link で
     計算し, `RosToUnity` 配下の子 GameObject の
     localPosition/localRotation を更新.
  5. IK 成功時は全 link の material を半透明 cyan,
     失敗時は半透明 red に切替える.
  6. `VrTrajectorySink` に対し `qCurrent_` と `IsIkSolved` を渡して publish.

### 4.8 ROS 2 出力 (VrTrajectorySink)

- topic:
  - `/follower/arm_controller/joint_trajectory` (joint names: `1`, `2`, `3`, `4`, `5`)
  - `/follower/gripper_controller/joint_trajectory` (joint names: `6`)
- QoS: Reliable / Volatile. 実装は ROSettaDDS の `ReliabilityQos.Reliable,
  DurabilityQos.Volatile` を指定する (既存 `RosTeleoperationSink` の
  ResetCommand publisher と同じ指定).
- payload: `JointTrajectoryPoint` 1 点, `positions` のみ設定
  (`velocities` / `accelerations` / `effort` は空配列), `time_from_start`
  = `trajectory_dt` (default 0.1 s). 単点軌道. `header.stamp` は Unity
  の `DateTimeOffset.UtcNow` を `builtin_interfaces/Time` に変換して詰める.
  時刻は wall-clock UTC であり `/use_sim_time` とは同期しない. controller
  (`joint_trajectory_controller`) は stamp を必須視しない想定のため, 同期
  しない前提でよい.
- 入力: `VirtualSoArm.GetLastSolvedJoints()`, `IsPlaced`, `IsIkSolved` (last frame).
  - 未配置 or IK 失敗 → publish しない.
- publish 周期: 初期実装は Unity `Update()` 同期. 将来 `minPublishIntervalSec`
  などで throttling する余地は spec 上残す (本 spec では実装しない).
- C# msg 生成: `trajectory_msgs/msg/JointTrajectory`,
  `trajectory_msgs/msg/JointTrajectoryPoint`, `builtin_interfaces/msg/Duration`
  を `rosettadds-genmsg` で生成し `Assets/_SoArmVR/Scripts/GeneratedMsgs/`
  配下に追加する.
- `ResetCommand` publisher は VR 経路から外す. `teleop_ik::msg::ResetCommand`
  型/msg 自体の削除は Step 5 (別 spec).

### 4.9 入力アクション

| アクション | 旧 | 新 |
| --- | --- | --- |
| Teleoperate (`gripPressed`) | grip 押下でテレオペ開始 + target/anchor 更新 | 配置済み時, grip 押下でクラッチ開始 |
| Gripper (`trigger`) | 0..1 で ROS publish | 0..1 → joint 6 角 (Unity 側) |
| Stick (`primary2DAxis`) | stick 値を ROS publish | x → joint 5 積分, y は当面未使用 |
| Reset (`primaryButton`) | ResetCommand publish | 削除 |
| PlaceArm (`primaryButton`, 新) | なし | A hold/release で base 設置 |
| IkActive (`activate`) | IK/手首モード切替 | 削除 (常時 IK モード) |

## 5. 検証

- ビルド: Unity Editor で compile error 0 件を確認.
  検証は `uloop build SoArmVR` を使う. 利用できない環境では
  Unity Editor の compile error 0 件を人の目で確認する.
- 動作: `vr_teleop_bridge.launch.py use_mock:=true` (mock 経由) と
  組み合わせ, Unity 内ゴーストが A 設置後の配置で表示されること. RViz
  mock の SO-101 は Unity から publish された joint 1..6 に追従すること
  (AR 床設置 pose は TF には反映しない. base は固定で joint のみ反映).
- 動作: グリップで EE を動かし, 実機/モックで joint 1..3 が
  追従すること. joint 4/5 は VR 側操作で動くこと.
- 既存 ROS 側テスト (`teleop_ik` の gtest / launch テスト) は
  本 spec 時点では縮退しない. VR 経路のテストは別 spec で追加する.

## 6. 段階移行

- ステップ 1: SoArmVR 側に `UrdfModel` + `UrdfKinematics` + `VirtualSoArm`
  プレファブ + `ArmPlacementController` を入れ, 旧 `TeleoperationSession`
  を一時残置しつつ A 設置とゴースト表示をデバッグ UI で確認.
  ROS 出力は既存 `RosTeleoperationSink` のまま (Step 1 中は Reset publish も
  維持). (visualization-only フェーズ)

- ステップ 2: 入力アクションを A=PlaceArm / grip=clutch / trigger→j6 /
  stick.x→j5 に切替. Reset action と `RosTeleoperationSink` の
  ResetCommand publisher はこのステップで削除する. 旧
  `TeleoperationSession` と `RosTeleoperationSink` の
  `TargetPoseWithInput` publish 経路のみを `useLegacyVrSink=true` で
  残置し, 新入力と旧 sink の両方が Step 3 完了まで動く形にする.
  `useLegacyVrSink=false` のときは旧 `TeleoperationSession` を
  ディスパッチしない (新入力は無視). フラグのデフォルトは `true`
  (Step 4 で legacy 経路を削除するまで legacy 側を既定にする). (legacy 縮退フェーズ A)

- ステップ 3: `UrdfIKSolver` + `VrTrajectorySink` + 必要 msg 生成
  (`trajectory_msgs/msg/JointTrajectory`, `JointTrajectoryPoint`,
  `builtin_interfaces/msg/Duration`) を追加. `vr_teleop_bridge.launch.py`
  + `vr_teleop_bridge_rviz.launch.py` を新設し, SoArmVR から
  `/follower/arm_controller/joint_trajectory` を直接 publish.
  旧 `RosTeleoperationSink` は Step 3 中も残し, 動作切替は ROS launch
  起動側で選択する形にする (新 launch = 旧 sink を使わない).

- ステップ 4: 旧 `RosTeleoperationSink` を削除し, 新経路で VR テレオペを
  一本化. `vr_teleop.launch.py` の `teleop_ik_node` include を切離し,
  新 launch への切替を促す. README/launch の利用ガイドを新経路に揃える.

- ステップ 5: `teleop_ik::msg::ResetCommand` / `/teleop/reset` を削除し,
  VR 経路から `teleop_ik_node` への依存を完全に外す. `teleop_ik_node` 自体は
  gamepad 経路で残し, その gamepad 経路を `JointTrajectory` 直出しに
  移行する別 spec で `teleop_ik_node` を削除する.

本 spec はステップ 1〜4 を範囲とする. ステップ 5 は別 spec.

> 旧 VR 経路 (`RosTeleoperationSink` + `teleop_ik_node`) と新 VR 経路
> (`VrTrajectorySink` + `vr_teleop_bridge.launch.py`) は Step 3 中に
> 並存する. どちらが動くかは起動する launch 側で決まる. Step 4 で旧
> 経路を削除し一本化する.

## 7. 関連

- 2026-06-21-soarmvr-anchor-projection-stick-wrist-design
- 2026-06-23-teleop-ik-cpp-rewrite-design
- 2026-06-23-teleop-ik-joint5-fk-design
- 2026-07-01-vr-reset-command-design (本 spec で無効化)
- 2026-07-07-vr-teleop-rviz-fake-design (mock hardware / RViz 表示)
