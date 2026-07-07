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
- `ResetCommand`/`/teleop/reset` を削除する.
- 現行 README 類の A ボタン説明 (`Reset`) は A = 床設置 に書き換える.

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| URDF | 展開済み `so101.urdf` を `SoArmVR/Assets/_SoArmVR/Urdf/so101.urdf` に TextAsset として同梱. xacro 展開は SoArmVR 側で行わない. |
| ロボットモデル | URDF から link/joint 階層を `UrdfModel` として読み込み, Unity GameObject ツリーへ transform を反映. mesh は当面プリミティブで OK (URDF mesh は将来段階). |
| 設置 | A ボタン長押し中に右コントローラー ray を AR 平面に raycast し, 離した瞬間の hit pose を `base` のワールド Transform として確定. |
| 基準フレーム | 設置後の `base` Transform からの相対で EE 目標と joint 4 pitch を扱う. 既存 `TeleoperationAnchor` (ヨー基準) は今回廃止. |
| 操作 | グリップ押下中だけクラッチ式に EE 位置を追従. 押し始めの controller pose と設置後最初の EE pose を基準化し, 差分を EE 目標に反映. |
| joint 4 | 右コントローラーの forward 方向を `base` 基準の pitch に変換して目標値とする. |
| joint 5 | 右スティック左右で速度積分 (deadzone 付き) し, 関節角度として注入. |
| グリッパ | 右トリガー (0..1) を joint 6 angle にマップ. |
| A ボタン | 設置専用. 短押し/長押し区別なし. 押下中ずっとプレビュー, 離した時点で確定. |
| Reset | 完全削除. `/teleop/reset` 経路と `teleop_ik::msg::ResetCommand` を削除. |
| IK | 数値 IK を C# で実装. 対象は joint 1..3. ターゲットは EE 位置. seed は前回の成功解. 収束失敗時は最終成功解を維持し, ゴーストを「IK 失敗」表現 (色変更) にする. |
| ROS 2 出力 | `trajectory_msgs/JointTrajectory` 2 本: `/follower/arm_controller/joint_trajectory`, `/follower/gripper_controller/joint_trajectory`. Unity 側座標は ROS 側へ変換しない (URDF 同梱のため ROS 側と一致). |
| QoS | arm/gripper trajectory は BestEffort / Volatile を避け, Reliable / Volatile とする (現 `teleop_ik_node` と同じ reliable publisher). |
| publish 周期 | `Update()` のたびに publish. 失敗フレームは publish しない. |
| 既存 launch | `vr_teleop.launch.py` は当面温存. 新経路は `vr_teleop_bridge.launch.py` として独立. README には両方を記載し, 用途を明示. |

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
  Unity の Y-up, left-handed を前提として axis/rotation を直接読み込む.
- `UrdfKinematics` (新規): `UrdfModel` と関節角から
  `Vector3 GetJointOrigin(int linkIndex)` と
  `Matrix4x4 GetJointTransform(int linkIndex, double[] q)` を計算.
  joint axis と rotation を順に適用する forward kinematics.
- `UrdfIKSolver` (新規): joint 1..3 を変数として EE 位置ターゲットを解く
  damped least squares. q_seed を受けて反復, URDF の lower/upper で clamp.
- `ArmPlacementController` (新規): A ボタンの hold/release, AR 平面
  raycast, base Transform 確定, 配置済みフラグ管理.
- `VirtualSoArm` (新規): 配置済み base に対する関節目標と EE 目標を保持.
  `Update()` で `UrdfKinematics` を回し, 子 GameObject の Transform を更新.
  `UrdfIKSolver` を呼んで joint 1..3 を求め, 失敗を `bool ikSolved` で公開.
- `VrTrajectorySink` (新規, `RosTeleoperationSink` の置換): 関節角を
  JointTrajectory メッセージに詰めて ROSettaDDS で publish.
- `ArmTeleoperationController` (新規): 入力 (grip, trigger, stick, j4 pitch)
  を `VirtualSoArm` の目標へ反映. 既存 `TeleoperationSession` を置換.

### 3.3 モジュール分割 (ROS 2 側)

- `ros2_ws/src/teleop_ik/launch/vr_teleop_bridge.launch.py` (新規):
  `so101_follower_controller.launch.py` だけを取り込み,
  `teleop_ik_node` は起動しない.
- `ros2_ws/src/teleop_ik/msg/ResetCommand.msg` (削除)
- `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg` (Unity 側で送らなくなる.
  rosidl からは削除してよいが, 既存 C++ テストが依存している可能性に
  注意. 段階的に外す).
- `ros2_ws/src/teleop_ik/src/ik_node.cpp` の `on_reset` 関連 (削除)
- `teleop_ik_node` 自体: 段階的に縮退. `vr_teleop_bridge.launch.py` からは
  起動しない. テストは当面残置.

### 3.4 既存コードへの影響

| 既存 | 扱い |
| --- | --- |
| `Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs` | 廃止. A ボタン設置に置換. |
| `Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs` | 廃止. `ArmTeleoperationController` に置換. |
| `Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs` | `VrTrajectorySink` に置換. ResetCommand publisher を削除. |
| `Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs` | 廃止. 旧 Sink 抽象は SoArmVR 内に閉じないため. |
| `Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions` | Reset を削除. A は「PlaceArm」にリネーム. grip/trigger/stick は継続. |
| `Assets/_SoArmVR/Prefabs/Anchor.prefab` | 廃止. |
| `Assets/_SoArmVR/Prefabs/Teleoperation.prefab` | 新規スクリプトに付け替え. AR Session / XR Origin は継続. |
| `Assets/_SoArmVR/Prefabs/VirtualSoArm.prefab` (新規) | ベース + 5 リンク + グリッパの GameObject ツリー. mesh は当面 Primitive. |
| `ros2_ws/src/teleop_ik/...` | 段階的に縮退 (本 spec の範囲外, 別途 spec で). |

## 4. 詳細設計

### 4.1 設置 (A ボタン)

- `ArmPlacementController.Update()`:
  - A 押下中: 毎フレーム `ARRaycastManager.Raycast` で右コントローラー forward
    を AR 平面 (Plane alignment = Horizontal up) に対して raycast.
  - hit があれば `previewBase` を hit pose に合わせて可視化.
  - A 離した瞬間に, hit pose を `VirtualSoArm.Place(basePose)` に渡す.
  - 既に配置済みの場合, 「再配置する」動線も許可 (1 度目と同じ挙動,
    IK 状態はリセット).
- AR 平面が取得できないときは何もしない (プレビュー/設置もスキップ).
  視認できる HUD で「床平面を検出中」と出す.

### 4.2 クラッチ式 EE 追従 (右グリップ)

- `ArmTeleoperationController`:
  - 配置済み (`VirtualSoArm.IsPlaced == true`) でないときは grip を無視.
  - grip 押下開始時:
    - `controllerStartPose` = 現在の右コントローラー pose
    - `eeStartPose` = 現在の EE pose (URDF FK から)
    - `clutchActive = true`
  - grip 押下中:
    - `deltaPos = controller.position - controllerStartPose`
    - `deltaRot = controller.rotation * Inverse(controllerStartPose.rotation)`
    - `eeTargetPose.position = eeStartPose.position + deltaPos * positionScale`
    - `eeTargetPose.rotation = eeStartPose.rotation * deltaRot` (任意)
  - grip 離した: `clutchActive = false`. 最終 EE pose は固定.

### 4.3 joint 4 (controller pitch → base 基準 pitch)

- 配置後の `base` forward (base.rotation * Vector3.forward) を基準.
- `controllerForward = controller.rotation * Vector3.forward`
- `baseForwardProjected = Vector3.ProjectOnPlane(baseForward, Vector3.up)` (正規化)
- `controllerForwardProjected = Vector3.ProjectOnPlane(controllerForward, Vector3.up)` (正規化)
- 両者の角度差 `theta` (水平面上での yaw) と, `controllerForward` の
  上下成分 `dy` (= `Vector3.Dot(controllerForward, Vector3.up)`) から
  pitch を計算:
  - `pitch = Mathf.Asin(Mathf.Clamp(dy, -1f, 1f))`
  - `j4 = pitch` とし, lower/upper で clamp.
- ヨー (`theta`) は `j4` には入れない (要望: pitch のみ).

### 4.4 joint 5 (stick 積分)

- `j5 += stick.x * j5RadPerSec * dt`
- lower/upper で clamp.
- deadzone 半径は `0.1` とし, それ以下は 0 として扱う.

### 4.5 グリッパ (joint 6)

- `j6 = gripperLowerRad + value * (gripperUpperRad - gripperLowerRad)`
- `gripperLowerRad = -0.174533`, `gripperUpperRad = 1.74533`
  (現 `teleop_ik_node` の値と一致).

### 4.6 IK (joint 1..3, EE 位置)

- `UrdfIKSolver.Solve(eeTargetLocal, qSeed, out qOut)`:
  - 反復: `q = q + J^T (J J^T + λ I)^{-1} (eeTarget - eeCurrent)`
  - J は 3x3 (URDF の joint 1..3 に関する位置 Jacobian を差分で近似).
  - `λ = 0.01` (damping). 初期実装は固定.
  - `maxIter = 100`, `tol = 1e-4` (現 ROS 側と一致).
  - 各反復で `UrdfKinematics` を呼び, lower/upper で clamp.
  - 収束したら `qOut` を返し `true`. 失敗なら `false`.
- seed: 前回成功解 `qSolution_`. 未配置時/初回は URDF 読み込み時の
  neutral configuration を使う.

### 4.7 FK とゴースト更新

- `VirtualSoArm.Update()`:
  1. joint 1..3 = `UrdfIKSolver` 出力 (成功時)
  2. joint 4 = `j4` (controller pitch)
  3. joint 5 = `j5` (stick 積分)
  4. joint 6 = `j6` (trigger)
  5. `UrdfKinematics.GetJointTransform(i, q)` を全 link で計算し,
     対応する GameObject の localPosition/localRotation を更新.
  6. IK 失敗時はゴーストの材質を「赤」または半透明へ切替え.
     publish は行わない (前フレームの joint 角で発行した最後の
     JointTrajectory を 1 周期間スキップする).

### 4.8 ROS 2 出力 (VrTrajectorySink)

- topic:
  - `/follower/arm_controller/joint_trajectory` (joint names: `1`, `2`, `3`, `4`, `5`)
  - `/follower/gripper_controller/joint_trajectory` (joint names: `6`)
- QoS: Reliable / Volatile.
- payload: `JointTrajectoryPoint` 1 点, `time_from_start` = `trajectory_dt`
  (default 0.1 s). 単点軌道.
- 入力: `VirtualSoArm.GetLastSolvedJoints()`, `IsPlaced`, `IsIkSolved` (last frame).
  - 未配置 or IK 失敗 → publish しない.
- `VrTrajectorySink` は `RosTeleoperationSink` の実装を
  `JointTrajectory` ベースに書換えたもの. `ResetCommand` publisher は
  この段階で削除する (型/msg も削除予定).

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

- ビルド: Unity 側で C# エラー / 警告がないこと.
- 静的: `SoArmVR/Assets/_SoArmVR/Scripts/**/*.cs` に対する
  `csc -warnaserror` 相当のチェック (CI は無いのでスクリプト or
  `uloop build` で担保).
- 動作: RViz mock launch (`vr_teleop_rviz.launch.py`) と組み合わせ,
  A で床に置いたバーチャル SO-101 の姿勢が TF / RViz に反映されること.
- 動作: グリップで EE を動かし, 実機/モックで joint 1..3 が
  追従すること. joint 4/5 は VR 側操作で動くこと.
- 既存 ROS 側テスト (`teleop_ik` の gtest / launch テスト) は
  本 spec 時点では縮退しない. VR 経路のテストは別 spec で追加する.

## 6. 段階移行

- ステップ 1: SoArmVR 側に `VirtualSoArm` + `UrdfModel` + `UrdfKinematics` を
  入れ, 配置とゴースト表示だけ有効化. ROS 出力は既存 `RosTeleoperationSink`
  のまま. (visualization-only フェーズ)
- ステップ 2: SoArmVR 側で IK を解き, `VrTrajectorySink` を追加.
  `vr_teleop_bridge.launch.py` を新設し, SoArmVR の出力を
  `controller_manager` へ直結.
- ステップ 3: 旧 `RosTeleoperationSink` / `teleop_ik_node` の IK 経路を
  切る. `vr_teleop.launch.py` は `vr_teleop_bridge.launch.py` への
  リダイレクトに縮退.
- ステップ 4: `teleop_ik::msg::ResetCommand` / `/teleop/reset` を削除.
  `teleop_ik_node` 自体を削除 (ゲームパッドは別ノード).

本 spec はステップ 1〜3 を範囲とする. ステップ 4 は別 spec.

## 7. 関連

- 2026-06-21-soarmvr-anchor-projection-stick-wrist-design
- 2026-06-23-teleop-ik-cpp-rewrite-design
- 2026-06-23-teleop-ik-joint5-fk-design
- 2026-07-01-vr-reset-command-design (本 spec で無効化)
- 2026-07-07-vr-teleop-rviz-fake-design (mock hardware / RViz 表示)
