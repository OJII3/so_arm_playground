# SoArmVR: アンカー投影 & スティック駆動手首 設計書

- 日付: 2026-06-21
- 対象: `SoArmVR/`(Unity), `ros2_ws/src/teleop_ik/`(ROS 2)
- ブランチ: `feat/unity-sticks-pub`

## 1. 背景とゴール

VR テレオペレーション(SoArmVR → SO-101 follower)では、Unity 側が
コントローラ姿勢(位置 + 全軸回転)をアンカー基準で publish し、ROS 2 側 (`teleop_ik`)が
IK を解いていた。手首(joint 4 = pitch、joint 5 = roll)は publish された quaternion から
ローカル pitch/roll を抽出して与えていた。

この方式には次の問題がある。

1. アンカー設置時のコントローラの傾き(pitch/roll)がそのまま session 開始角に反映され、
   ユーザの手首が中立でもロボット手首が傾く。
2. セッション中の手首制御が「コントローラの傾き」に縛られ、細かい pitch/roll を
   独立に合わせる手段がない。
3. アンカーがコントローラの全軸に追従するため、ワールド鉛直軸と一致せず、
   ロボットの作業空間が傾いて見える。

本変更では以下を実現する。

- アンカーの up 軸を常にワールド up 軸に一致させ、ヨーのみ保持する。
- コントローラのスティック入力(右親指スティック)を publish し、IK 側で
  速度として積分して手首目標に置き換える。
- 入力契約(`Pose` + `Vector2 stick`)を単一の新規 msg に統一し、
  既存の `/teleop/target_pose` (PoseStamped) は廃止する。

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| アンカーの自由度 | ヨーのみ保持(up = world up, forward = コントローラ forward の水平面投影) |
| アンカーの位置 | コントローラ位置をそのまま(world position) |
| スティック取得元 | 右コントローラの `primary2DAxis` |
| 軸マッピング | stick.x → joint 5(roll), stick.y → joint 4(pitch) |
| 新規 msg | `teleop_ik/msg/TargetPoseWithInput.msg`(Header + Pose + Vector2 stick) |
| 新規トピック | `/teleop/target`(BestEffort/Volatile) |
| 既存 `/teleop/target_pose` | 廃止 |
| スティック積分 | ROS 側 `ik_node` で行う(dt は `header.stamp` から算出) |
| gamepad_node | 新型 publish に移行、stick = (0, 0) |
| gamepad の手首 | ニュートラル固定(駆動しない) |

## 3. アーキテクチャ

### 3.1 データフロー

```
[Quest 3]
   ├─ TeleoperationSession  ── Update ──> TeleoperationSample{pos, rot, stick, gripper}
   │                                          │
   │                                          ▼
   │                            TeleoperationAnchor.ToAnchorSpace()
   │                            (anchor.up = world.up, yaw only)
   │                                          │
   │                                          ▼
   └─ RosTeleoperationSink.Push(sample)
                │ publish /teleop/target (BestEffort)
                ▼
[teleop_ik_node]
   ├─ _on_target_pose(msg: TargetPoseWithInput)
   │     ├─ pose.position を Unity→ROS 変換
   │     ├─ stick を deadzone 後に delta_t で積分し _integrated_stick を更新
   │     └─ joint 4/5 目標 = wrist_init_pos + (stick.x→j5, stick.y→j4)
   ├─ joint 1〜3 のみ position IK(CLIK)
   └─ JointTrajectory publish
```

### 3.2 コンポーネント責務

| コンポーネント | 責務 |
| --- | --- |
| `TeleoperationAnchor.Place` | ヨーのみ抽出した回転 + 入力位置でアンカーを設置 |
| `TeleoperationSession` | Input Actions を読み、`TeleoperationSample` を sink に送る |
| `RosTeleoperationSink` | DDS publisher を持ち、TargetPoseWithInput を publish |
| `ik_node` | Unity→ROS 座標変換、stick 積分、position IK、JointTrajectory publish |
| `gamepad_node` | ゲームパッドの積分済み位置 + stick=0 で TargetPoseWithInput を publish |
| `teleop_ik/msg/TargetPoseWithInput` | 新規 ROS 2 msg |

## 4. 詳細

### 4.1 アンカー投影(Unity)

`TeleoperationAnchor.Place(Vector3 worldPos, Quaternion worldRot)`:

```csharp
Vector3 fwd = Vector3.ProjectOnPlane(worldRot * Vector3.forward, Vector3.up);
Vector3 up = Vector3.up;
if (fwd.sqrMagnitude < 1e-6f) {
    fwd = Vector3.ProjectOnPlane(Vector3.forward, Vector3.up);
}
transform.SetPositionAndRotation(worldPos, Quaternion.LookRotation(fwd.normalized, up));
```

- `up` は常に `Vector3.up`(=ワールド up、Unity 左手系 Y-up)。
- `forward` はコントローラ forward を水平面に投影し、必要なら `Vector3.forward` にフォールバック。
- `ToAnchorSpace()` は変更なし。ローカル回転の pitch/roll は session 開始以降の変化分のみ。

### 4.2 Input Actions 追加(Unity)

`SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions`:

- アクション `Stick` を追加(`type: Value`, `expectedControlType: Vector2`)
- バインディング: `<XRController>{RightHand}/primary2DAxis`
- 既存 `Teleoperate` / `Gripper` は不変

`TeleoperationSession` の追加:

- `[SerializeField] InputActionProperty _stickAction;`
- `OnEnable` / `OnDisable` で `_stickAction.action?.Enable()/Disable()`
- `PushSample()` で `var stick = _stickAction.action != null ? _stickAction.action.ReadValue<Vector2>() : Vector2.zero;` を sample に詰める

`TeleoperationSample` への追加:

```csharp
public Vector2 stick;
```

### 4.3 新規 msg(ROS 2)

`ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`:

```
std_msgs/Header header
Pose pose
geometry_msgs/Vector2 stick
```

- 配置: `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`(パッケージ内に新規ディレクトリ)
- `package.xml` の編集:
  - `<buildtool_depend>rosidl_default_generators</buildtool_depend>` を追加
  - `<exec_depend>rosidl_default_runtime</exec_depend>` を追加
  - `<member_of_group>rosidl_interface_packages</member_of_group>` を追加
- `teleop_ik` は ament_python のため、`CMakeLists.txt` を新規追加して
  `find_package(rosidl_default_generators REQUIRED)` と
  `rosidl_generate_interfaces(${PROJECT_NAME} msg/TargetPoseWithInput.msg ...)` を呼ぶ。
  既存の ament_python 部分はそのまま並置可能。

### 4.4 Unity 側 Sink

`RosTeleoperationSink`:

- `_targetPub: Publisher<TargetPoseWithInput>` を `CreatePublisher<TargetPoseWithInput>(
  "/teleop/target", serializer, ReliabilityQos.BestEffort, DurabilityQos.Volatile, DdsTypeName)` で生成
- 旧 `_posePub`(PoseStamped 用の 5 引数版 BestEffort 呼び出し)は削除
- `_gripperPub` / `_activePub` は据え置き
- `Push(sample)`:
  - Header(`stamp`, `"teleop"`)を組み立て
  - `Pose(Position(sample.position), Quaternion(sample.rotation))` を組み立て
  - `Vector2((double)sample.stick.x, (double)sample.stick.y)` を組み立て
    (ROS 側 `geometry_msgs/Vector2` は `float64` のため Unity の `float` からキャスト)
  - `TargetPoseWithInput(header, pose, stick)` を `PublishAsync`

Unity 側 msg の再生成:

- `nix develop .#soarmvr` で `dotnet run --project <ROSettaDDS>/tools/rosettadds-genmsg` を再実行
- 出力先: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs`(2026-06-13 設計書 §3.2 と同じ)
- Unity asmdef の `references` に `ROSettaDDS` が入っていることは既存で確認済み

### 4.5 IK ノード

`teleop_ik/ik_node.py`:

- パラメータ追加(全て `teleop_ik_params.yaml` に追記):
  - `stick_velocity_scale`(double, 既定 `1.5`)
  - `stick_deadzone`(double, 既定 `0.1`)
  - `stick_max_delta_per_msg`(double, 既定 `0.2`)
  - `stick_fallback_dt`(double, 既定 `0.0111` = 1/90Hz)
- `subscribe`:
  - 旧 `PoseStamped` 購詮を削除
  - `TargetPoseWithInput` 購詮を追加(QoS は BestEffort/Volatile、旧 `target_pose` と同じ)
- 状態変数追加:
  - `_integrated_stick: tuple[float, float] = (0.0, 0.0)`
  - `_last_msg_stamp: rclpy.time.Time | None = None`
- `_start_session`:
  - 既存通り `_wrist_init_pos` を joint states からキャプチャ
  - `_integrated_stick = (0.0, 0.0)` にリセット
  - `_last_msg_stamp = None`
- `_stop_session`:
  - 既存通り。`_integrated_stick` のリセットは `_start_session` 側で実施
- `_on_target_pose` (新 `_on_target_with_input`):
  - `_active` 等の前提チェック(既存通り)
  - `delta_t = _compute_dt(msg.header.stamp)`:
    - 初回 (`_last_msg_stamp is None`) / 負値 / 0.5s 超 → `stick_fallback_dt`
  - `vx_raw, vy_raw = msg.stick.x, msg.stick.y`
  - deadzone 適用:  `|v| < deadzone` なら 0、 そうでなければ
    `v = sign(v) * (|v| - deadzone) / (1.0 - deadzone)`(リマップ)
  - 各軸で `|v| > stick_max_delta_per_msg / (scale * delta_t)` なら
    `v` を `sign(v) * stick_max_delta_per_msg / (scale * delta_t)` にクランプ
    (1 メッセージあたりの変位が `stick_max_delta_per_msg` を超えない安全弁)
  - `delta_vx, delta_vy = vx * scale * delta_t, vy * scale * delta_t`
  - `_integrated_stick = (_integrated_stick[0] + delta_vx, _integrated_stick[1] + delta_vy)`
  - 関節目標:
    - `q_seed[j4] = clamp(wrist_init_pos[0] + _integrated_stick[1], lower, upper)`(stick.y → joint 4/pitch)
    - `q_seed[j5] = clamp(wrist_init_pos[1] + _integrated_stick[0], lower, upper)`(stick.x → joint 5/roll)
  - position IK は joint 1〜3 のみ(従来通り)
- 削除:
  - `unity_quaternion_to_pitch_roll` の import と呼び出し
  - `coordinate_utils.py` の `unity_quaternion_to_pitch_roll` 関数(後方互換なし)

### 4.6 gamepad_node の移行

`teleop_ik/gamepad_node.py`:

- 購詮: `Joy`(既存)
- publish:
  - `TargetPoseWithInput` を `/teleop/target` に
  - `pose.position` = 積分済みの (x, y, z)
  - `pose.orientation` = identity(現状維持)
  - `stick` = `Vector2(x=0.0, y=0.0)`(`geometry_msgs/Vector2` インスタンス、ROS 側は常にゼロで受信)
  - `header.frame_id = "world"`
  - `header.stamp` = `self.get_clock().now().to_msg()`(既存通り)
- 旧 `PoseStamped` publish コードは削除
- パラメータ: `publish_rate` / `linear_speed` / `vertical_speed` / `deadzone` /
  軸ボタン対応は不変

### 4.7 coordinate_utils の整理

`ros2_ws/src/teleop_ik/teleop_ik/coordinate_utils.py`:

- `unity_position_to_ros`: 維持
- `unity_quaternion_to_ros`: 維持(将来用に残す)
- `unity_quaternion_to_pitch_roll`: 削除

## 5. Launch / Interop

- `teleop_ik/launch/teleop_ik.launch.py` / `vr_teleop.launch.py` / `gamepad_teleop.launch.py` の引数や include 構成に変更なし。
- 購読トピックが `/teleop/target_pose` → `/teleop/target` に変わるだけ。
- 既存 `test_vr_teleop_launch.py` は launch 引数のみ検証しているため影響なし。

## 6. テスト

### 6.1 追加・更新する pytest

`ros2_ws/src/teleop_ik/test/test_ik_node.py`:

- 既存テストの更新:
  - `test_target_pose_uses_previous_successful_solution_as_next_seed` → `test_target_uses_previous_successful_solution_as_next_seed`
    - seed の wrist 期待値を integrated_stick 由来に書き換え(stick.y → j4, stick.x → j5)
  - `test_solve_ik_keeps_wrist_joint_targets_fixed` は不変(seed に j4/j5 を直接書く)
- 新規テスト:
  - `test_wrist_integrates_stick_per_message`: 連続 2 メッセージで integrated_stick が増分し、j4/j5 の seed が wrist_init + 増分に等しい
  - `test_wrist_resets_on_session_start`: `_start_session` 後に `_integrated_stick == (0, 0)`
  - `test_stick_deadzone_zeros_small_inputs`: 0.05 入力が deadzone(0.1)未満で変位ゼロ
  - `test_stick_huge_dt_clamps_to_fallback`: 1.0s 間隔の stamp で `stick_fallback_dt` が使われる
  - `test_stick_max_delta_per_msg_clamp`: 大きな delta_t でも 1 メッセージあたり `stick_max_delta_per_msg` を超えない

`ros2_ws/src/teleop_ik/test/test_coordinate_utils.py`:

- `unity_quaternion_to_pitch_roll` 関連のテストを削除

新規 `ros2_ws/src/teleop_ik/test/test_target_msg.py`:

- `teleop_ik.msg.TargetPoseWithInput` を import してインスタンス化・フィールドアクセスできることを確認
- 既存 CMake ビルド成果物前提のため、colcon build 後の環境のみで pass する旨を docstring に明記

### 6.2 検証チェックリスト(PR 出す前)

- `nix develop .#ros` で `colcon build --packages-select teleop_ik` 成功
- `colcon test --packages-select teleop_ik` 全パス
- `nix develop .#soarmvr` で `rosettadds-genmsg` 再生成成功
- `uloop` で Unity コンソールにエラー無し
- 可能なら実機 LAN 環境で `ros2 topic list` / `ros2 topic echo /teleop/target` の疎通確認

## 7. スコープ外

- ROSettaDDS 本体への変更・PR
- Unity Test Framework の EditMode/PlayMode 追加
- スティックの第3軸(将来用)
- gamepad_node での手首駆動
- スティックに対する明示的な「リセット」ボタン(セッション ON/OFF で代用)
- 既存 `unity_quaternion_to_pitch_roll` 関数の後方互換維持(完全削除)
- ヘッドレス CI での IK 結合テスト

## 8. 段階コミット(参考)

リポジトリ運用ルール(AGENTS.md)に従い、コミットは小さく切る想定:

1. `teleop_ik` に新規 msg と `package.xml` / `CMakeLists.txt` 追加
2. `ik_node` のスティック駆動化 + テスト更新
3. `gamepad_node` の新型 publish 化
4. `coordinate_utils` から `unity_quaternion_to_pitch_roll` 削除
5. `TeleoperationAnchor` のヨー抽出化
6. `SoArmTeleoperation.inputactions` に `Stick` 追加
7. `TeleoperationSample` に `stick` 追加
8. `RosTeleoperationSink` の新 msg publish 化
9. Unity 側で genmsg 再生成・Scene/Prefab の動作確認

各ステップの直後に該当ユニットテスト / Unity コンパイルを走らせる.
