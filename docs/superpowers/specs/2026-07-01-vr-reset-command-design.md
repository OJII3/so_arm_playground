# teleop_ik: VR リセットコマンド (IK バイパス) 設計書

- 日付: 2026-07-01
- 対象: `ros2_ws/src/teleop_ik/`, `SoArmVR/`
- ブランチ: (未作成, 実装着手時に切る)

## 1. 背景とゴール

`SoArmVR` (Unity) のテレオペレーション中, ユーザが
SO-101 follower の姿勢を任意の初期姿勢 (home) に戻したい
ケースがある. 現状のシステムでは `apply_home_on_activate`
+ `home_j*_rad` という「起動時にのみ home を適用する」経路
しか存在せず, 一度テレオペを始めた後に姿勢をリセットする
手段が無い.

これに対し, 次を要望する:

- VR コントローラの A ボタン (`<XRController>{RightHand}/primaryButton`)
  を押すと, アーム 5 関節 + グリッパ 1 関節 (合計 6 軸) を
  設定済みの home 姿勢へ戻す.
- 戻す際は IK を通さず, 関節空間の setpoint として直接
  `JointTrajectory` を publish する.
- ホーム姿勢のソースは `teleop_ik` ノードのパラメータとし,
  VR 側からは「リセット要求」だけを publish する.
- グリッパも同時にリセットする (アームだけ戻すオプションは
  今回提供しない).
- ゲームパッド (PS3) 側は無改修. VR からのリセットのみ.

ゴール:

- 新規 ROS メッセージ `teleop_ik/msg/ResetCommand.msg` を定義.
- `teleop_ik_node` が `/teleop/reset` を subscribe して
  IK バイパスで `JointTrajectory` を publish.
- `SoArmVR` 側にリセット送信経路を追加.
- 既存の IK 経路・テレオペ経路・パラメータ・トピック QoS は
  影響を受けない.

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| メッセージ | `teleop_ik/msg/ResetCommand.msg` を新規定義 |
| トピック | `/teleop/reset` (RELIABLE / VOLATILE) |
| ホーム姿勢ソース | `teleop_ik` ノードの param (`home_j*_rad`) |
| 対象関節 | joint 1, 2, 3, 4, 5, 6 (アーム 5 + グリッパ 1) |
| IK 使用 | しない. 関節空間の setpoint を JointTrajectory に詰めて publish. |
| 動作中セッション | リセット受信時に `active_ = false` にしてから publish. 次の `TargetPoseWithInput` は破棄される (`on_target_with_input` 冒頭の `if (!active_) return false;` で弾かれる). ユーザが grip を離し再度押すと新しいセッションが始まる. |
| ゲームパッド側 | 触らない. VR のみ. |
| launch / config (follower 側) | 触らない |

## 3. アーキテクチャ

### 3.1 全体フロー

```
┌──────────────────┐                                  ┌──────────────────┐
│  SoArmVR (Unity) │                                  │  teleop_ik_node  │
│                  │                                  │  (C++)           │
│ Reset InputAction│  /teleop/reset                  │  ┌────────────┐  │
│ (A button) ──────┼─► ResetCommand msg ──────────────┼─►│ sub_reset_ │  │
│                  │                                  │  └─────┬──────┘  │
│ RosTeleoperation │                                  │        │         │
│ Sink.cs (新規pub)│                                  │  on_reset_msg    │
│                  │                                  │  ├ active_=false │
│                  │                                  │  ├ param 補完    │
│                  │                                  │  └ JointTrajectory 2 個 publish │
│                  │                                  │    (arm, gripper) │
└──────────────────┘                                  │        │         │
                                                     │        ▼         │
                                                     │  /follower/arm_controller/...    │
                                                     │  /follower/gripper_controller/... │
                                                     └──────────────────┘
```

IK 経路 (VR pose → CLIK → JointTrajectory) とリセット経路
(param home → JointTrajectory) は完全に独立.
`on_reset_msg` は `solve_ik` も `forwardKinematics` も
呼ばない.

### 3.2 ResetCommand メッセージ

`ros2_ws/src/teleop_ik/msg/ResetCommand.msg`:

```msg
# teleop_ik/msg/ResetCommand.msg
# Signal to publish a JointTrajectory to move the arm + gripper to a
# pre-configured home pose, bypassing IK.

# Standard ROS header
std_msgs/Header header

# Home joint angles in radians.
# Element order matches joint_names ["1", "2", "3", "4", "5", "6"].
# NaN value at index i = use node parameter `home_j{i+1}_rad` for that joint.
# This allows partial overrides (e.g., override arm but keep gripper default).
float32[6] home_joints

# Trajectory time_from_start in seconds.
# <= 0.0 = use node parameter `reset_duration_sec`.
float32 duration_sec
```

センチネルに NaN を採用するのは `feetech_ros2_driver` で
`home_rads_` の初期値として既に
`std::numeric_limits<double>::quiet_NaN()` を使っており,
コードベースの流儀と揃うため.

> **Note**: `float32[6]` は固定長なので, 生成される C++
> 型は `std::array<float, 6>` となり `.size() == 6` 固定.
> 受信側はインデックスアクセスのみでよく, 長さチェックは不要.
> NaN 判定は `std::isnan(msg.home_joints[i])` で行う.

## 4. 変更点

### 4.1 `ros2_ws/src/teleop_ik/msg/ResetCommand.msg` (新規)

上記 §3.2 の内容で作成.

### 4.2 `ros2_ws/src/teleop_ik/CMakeLists.txt`

`rosidl_generate_interfaces` に `msg/ResetCommand.msg` を追加:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  msg/ResetCommand.msg
  DEPENDENCIES geometry_msgs std_msgs
)
```

`std_msgs` は `ResetCommand.msg` で `std_msgs/Header` を使うため
既存依存で OK.

### 4.3 `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`

- 公開メソッド追加: `void on_reset_msg(const teleop_ik::msg::ResetCommand::SharedPtr & msg);`
- private ヘルパ追加: `void on_reset(const teleop_ik::msg::ResetCommand & msg);`
  (テストから直接呼べるよう, `on_active` 等と同じ粒度で切る)
- subscription メンバ追加:
  `rclcpp::Subscription<teleop_ik::msg::ResetCommand>::SharedPtr sub_reset_;`

> ホーム値 (home_j*_rad) と `reset_duration_sec` は
> `on_reset` 内で都度 `get_parameter` 経由で読む
> (既存 `on_target_msg` の `trajectory_time_from_start` 等と
> 同じ流儀). メンバキャッシュは追加しない.

### 4.4 `ros2_ws/src/teleop_ik/src/ik_node.cpp`

`init_ros_node` 内で:

```cpp
this->declare_parameter<double>("home_j1_rad", 0.0);
this->declare_parameter<double>("home_j2_rad", 0.0);
this->declare_parameter<double>("home_j3_rad", 0.0);
this->declare_parameter<double>("home_j4_rad", 0.0);
this->declare_parameter<double>("home_j5_rad", 0.0);
this->declare_parameter<double>("home_j6_rad", 0.0);
this->declare_parameter<double>("reset_duration_sec", 2.0);
```

> これらのパラメータ値は `on_reset` 内で
> `get_parameter` 経由で都度読み直す
> (launch 引数での上書きや dynamic reconfigure に備える).
> メンバキャッシュはしない (§4.3 の方針).

`sub_reset_` を作成:

```cpp
sub_reset_ = this->create_subscription<teleop_ik::msg::ResetCommand>(
    "/teleop/reset", 10,
    std::bind(&TeleopIKNode::on_reset_msg, this, std::placeholders::_1));
```

`on_reset` の実装:

```cpp
void TeleopIKNode::on_reset(const teleop_ik::msg::ResetCommand & msg)
{
  // 1. ホーム関節角度を解決 (NaN は param フォールバック)
  std::array<double, 6> home{};
  const std::array<const char *, 6> param_names = {
    "home_j1_rad", "home_j2_rad", "home_j3_rad",
    "home_j4_rad", "home_j5_rad", "home_j6_rad",
  };
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isnan(msg.home_joints[i])) {
      home[i] = msg.home_joints[i];
    } else {
      home[i] = this->get_parameter(param_names[i]).as_double();
    }
  }

  // 2. duration を解決 (<=0 / NaN は param フォールバック)
  double duration = msg.duration_sec;
  if (!(duration > 0.0)) {  // NaN も false なのでここに落ちる
    duration = this->get_parameter("reset_duration_sec").as_double();
  }

  // 3. セッションを解除 (次の target msg は破棄される)
  active_ = false;
  unity_anchor_set_ = false;
  integrated_stick_.setZero();

  // 4. arm (joint 1-5) と gripper (joint 6) の軌道を publish
  Eigen::VectorXd q_arm = Eigen::VectorXd::Zero(model_.nq);
  for (size_t i = 0; i < 5; ++i) {
    if (arm_joint_ids_[i] == static_cast<pinocchio::JointIndex>(-1)) continue;
    const auto idx_q = model_.joints[arm_joint_ids_[i]].idx_q();
    q_arm[idx_q] = home[i];
  }
  pub_arm_->publish(make_arm_trajectory(q_arm, duration));
  pub_gripper_->publish(make_gripper_trajectory(home[5], duration));
}
```

> **Note**: `q_current_` / `q_solution_` は触らない. リセットは
> 「目標角度を JointTrajectory に書いて終わり」で,
> IK のシードにも反映しない. 次のテレオペが始まったタイミングで
> `on_active(true)` 経路が新しい home として
> `q_current_` (= 現関節角度) を読み直す.

`on_reset_msg` は `on_reset(*msg)` を呼ぶだけ.

### 4.5 `ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml`

```yaml
teleop_ik_node:
  ros__parameters:
    # ... existing params ...
    home_j1_rad: 0.0
    home_j2_rad: 0.0
    home_j3_rad: 0.0
    home_j4_rad: 0.0
    home_j5_rad: 0.0
    home_j6_rad: 0.0
    reset_duration_sec: 2.0
```

### 4.6 Unity 側: `SoArmVR/Assets/Msgs/teleop_ik/msg/ResetCommand.msg` (新規)

`ros2_ws/src/teleop_ik/msg/ResetCommand.msg` と同内容.
C# バインディング再生成は `nix develop .#soarmvr` 環境で
`rosettadds-genmsg` を実行 (既存
[`2026-06-13-soarmvr-rosettadds-migration-design.md`](2026-06-13-soarmvr-rosettadds-migration-design.md)
§3.2 と同じ手順).

### 4.7 Unity 側: `SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions`

`Reset` アクション (Button) を追加し,
`<XRController>{RightHand}/primaryButton` にバインド.

### 4.8 Unity 側: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs`

`PublishReset()` を追加:

```csharp
public interface ITeleoperationSink
{
    void OnSessionBegin();
    void Push(in TeleoperationSample sample);
    void OnSessionEnd();
    void PublishReset();
}
```

### 4.9 Unity 側: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/DebugTeleoperationSink.cs`

`PublishReset()` の空実装を追加 (interface 契約維持).

### 4.10 Unity 側: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`

- `Publisher<ResetCommandMessage> _resetPub;` を追加
- `InitParticipant()` 内に

  ```csharp
  _resetPub = _participant.CreatePublisher<ResetCommandMessage>(
      "/teleop/reset",
      ResetCommandMessageSerializer.Instance,
      ReliabilityQos.Reliable,
      DurabilityQos.Volatile,
      ResetCommandMessage.DdsTypeName);
  ```

  を追加. `/teleop/active` や `/teleop/gripper` と同じ
  Reliable/Volatile 設定 (リセットは 1 ショットだが,
  取りこぼし防止のため Reliable にする).
- `OnDestroy()` で `_resetPub?.Dispose();` 追加
- `PublishReset()` 実装:

  ```csharp
  public void PublishReset()
  {
      _ = PublishResetAsync();
  }

  async System.Threading.Tasks.Task PublishResetAsync()
  {
      if (_resetPub == null) return;
      var now = System.DateTimeOffset.UtcNow;
      var stamp = new RosTime((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000));
      var msg = new ResetCommandMessage(
          new Header(stamp, "teleop_reset"),
          new float[] { float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN },
          0.0f
      );
      try { await _resetPub.PublishAsync(msg); }
      catch (System.ObjectDisposedException) { }
  }
  ```

### 4.11 Unity 側: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs`

- `[SerializeField] InputActionProperty _resetAction;` 追加
- `OnEnable / OnDisable` で enable / disable
- `Update()` 内で rising edge 検出:

  ```csharp
  bool reset_pressed = _resetAction.action != null && _resetAction.action.IsPressed();
  if (reset_pressed && !_prev_reset_pressed_)
      _sink?.PublishReset();
  _prev_reset_pressed_ = reset_pressed;
  ```

- `bool _prev_reset_pressed_ = false;` メンバ追加

## 5. テスト戦略

### 5.1 単体テスト (C++ gtest)

新規 `test/test_reset_msg.cpp`:

| ケース | 検証内容 |
| --- | --- |
| `ResetCommandMsg.DefaultValues` | `home_joints` は 6 要素のゼロ配列, `duration_sec == 0.0` |
| `ResetCommandMsg.SetFields` | フィールド設定のラウンドトリップ |
| `ResetCommandMsg.NaNSentinel` | `NaN` を代入して読み戻せる |

`test/test_ik_node_helpers.cpp` への追加:

> **Note**: 既存 `make_for_test` は publisher を作成しない
> (`init_ros_node` をスキップするため). テストで publish を
> 検証するには publisher が必要なので, テストでは
> `parameter_overrides` を渡して **通常のコンストラクタ**
> を使う. `test_qos.cpp` の `ik_node_` 構築方法を踏襲.

新規 fixture `ResetFixture` (TeleopIKHelpersTest を継承):

```cpp
struct ResetFixture : public TeleopIKHelpersTest
{
  void SetUp() override
  {
    // 通常のコンストラクタ + parameter_overrides を使い,
    // publisher / subscriber / declare_parameter まで走らせる.
    const char * path = std::getenv("TELEOP_IK_TEST_URDF_PATH");
    ASSERT_NE(path, nullptr);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        rclcpp::Parameter("urdf_path", std::string(path)),
    });
    for (size_t i = 0; i < 6; ++i) {
      const std::string name = "home_j" + std::to_string(i + 1) + "_rad";
      opts.parameter_overrides.push_back(
          rclcpp::Parameter(name, 0.1 * static_cast<double>(i + 1)));
    }
    opts.parameter_overrides.push_back(
        rclcpp::Parameter("reset_duration_sec", 1.5));
    node_ = std::make_shared<teleop_ik::TeleopIKNode>(opts);
  }
  std::shared_ptr<teleop_ik::TeleopIKNode> node_;
};
```

| ケース | 検証内容 |
| --- | --- |
| `OnResetUsesParamDefaultsForAllNaN` | `home_joints` 全 NaN, duration 0 → JointTrajectory の中身が param 設定値 `0.1, 0.2, 0.3, 0.4, 0.5, 0.6` の並び (gripper は `home[5]=0.6`). `active_` は `false`. publish 検証は下記 `OnResetPublishesArmAndGripper` で行う. |
| `OnResetPartialOverride` | `home_joints[0]=0.7` (実数), 残り NaN → arm 5 関節 + gripper の JointTrajectory が `[0.7, 0.2, 0.3, 0.4, 0.5, 0.6]` 相当. publish 検証も同じく下記. |
| `OnResetUsesProvidedDuration` | `duration_sec=0.5` → JointTrajectory の `time_from_start` が 0.5 sec |
| `OnResetDurationFallsBackOnZeroOrNaN` | `duration_sec=0.0` / NaN → param 値の 1.5 sec |
| `OnResetClearsActiveSession` | リセット前に `active_=true, unity_anchor_set_=true, integrated_stick_=non-zero` → on_reset 後 `active_==false`, `unity_anchor_set_==false`, `integrated_stick_==zero` |
| `OnResetPublishesArmAndGripper` | 別 `rclcpp::Node` (`probe_node`) を作って `/follower/arm_controller/joint_trajectory` と `/follower/gripper_controller/joint_trajectory` に RELIABLE subscribe. `node_->spin_some()` を 1 回回して publish を処理させ, 両トピックから 1 メッセージずつ届くことを assert. arm 側は `joint_names={"1".."5"}`, gripper 側は `joint_names={"6"}` で各 `positions` の長さが一致. |
| `OnResetDoesNotInvokeSolver` | リセット前後で `q_solution_` のノルムが一致 (IK 経路を一切走らないことの確認) |

`test_ik_node_helpers.cpp` の既存テストは影響なし.

### 5.2 launch テスト

`test/test_vr_teleop_launch.py` は launch 引数のみ検証しているため
影響なし.

### 5.3 Unity テスト

現状 C# テストフレームワークは入っていない
([`docs/rosettadds-feature-gaps.md`](../../rosettadds-feature-gaps.md) 参照).
`PublishReset()` の msg 構築ロジックは小さく,
目視レビュー + Unity ビルド成功で代替する.

### 5.4 手動検証 (ドキュメント化のみ)

- `nix develop .#ros` で `ros2 topic list` に `/teleop/reset` が
  出ること
- VR 起動 → A ボタン押下 → `ros2 topic echo /teleop/reset` で
  1 メッセージ, `ros2 topic echo /follower/arm_controller/joint_trajectory`
  に JointTrajectory 1 個が届くこと

## 6. マイルストーン

| # | 内容 | 検証 |
| --- | --- | --- |
| 1 | `msg/ResetCommand.msg` 新規 + `CMakeLists.txt` 追加 | `colcon build` 成功 |
| 2 | `teleop_ik_node` に param 追加 + `sub_reset_` + `on_reset_msg` 実装 | ビルド成功 |
| 3 | gtest 追加 (`test_reset_msg.cpp` + `test_ik_node_helpers.cpp`) | `colcon test` 全パス |
| 4 | `teleop_ik_params.yaml` に `home_j*_rad` + `reset_duration_sec` 追加 | YAML パース成功 |
| 5 | Unity 側: `.msg` ミラー + `rosettadds-genmsg` 再生成 | C# ファイル生成, ビルド成功 |
| 6 | Unity 側: inputactions に Reset 追加, `RosTeleoperationSink` に publisher, `ITeleoperationSink` 拡張, `TeleoperationSession` で rising edge 検出 | Unity ビルド成功 |
| 7 | `ros2_ws/README.md` / `SoArmVR/README.md` にリセット機能の説明を追記 (任意) | ドキュメント確認 |

## 7. リスクと対策

| リスク | 対策 |
| --- | --- |
| `home_j*_rad` の param 値が関節リミット外で実機が脱力姿勢に突っ込む | launch 側で `urdf` の joint リミットに合わせた値を設定する責任. IK ノード側では特に clamp しない (現状の `apply_home_on_activate` も同方針). 必要なら §8 範囲外として後続タスクで `clamp_joints` を通す. |
| `ResetCommand` を VR 側が連打したときの挙動 | `JointTrajectoryController` が新ゴールを前ゴールに preempt するので安全. アプリ側でクールダウンは設けない. |
| `float32` 配列に `NaN` を入れて送ったとき, DDS シリアライズで失われる | ROS 2 の IDL は IEEE 754 を素直にシリアライズする. ROSettaDDS も同様. ただし C# 側の `float.NaN` が DDS 経由で 0 に化けるケースが理論上あり得る. 失敗時は §8 範囲外として `float32[6]` を `bool use_default_home + float32[6]` に変えることも検討. |
| 既存の `TestIkNodeSubscribers` 等, subscriber 数が増えることによる既存テストの失敗 | テストが `get_node_names` 等で subscriber 検証していないか確認. 影響があれば更新. |
| C# バインディング再生成が CI に組み込まれていない | 既存も手動運用 ([`2026-06-13-soarmvr-rosettadds-migration-design.md`](2026-06-13-soarmvr-rosettadds-migration-design.md) §7). 今回も同じ. |

## 8. 範囲外

- ゲームパッド (PS3) 側のリセット対応
- アームだけリセット (グリッパ維持) オプション
- リセット中の進捗・完了通知 (`/teleop/reset_status` 等)
- home_j*_rad の値を joint limit にクランプするロジック
- dynamic reconfigure での home 値変更
- C# 側テストフレームワークの導入
- 既存の IK 経路の動作変更

## 9. 関連ドキュメント

- [`ros2_ws/src/teleop_ik/src/ik_node.cpp`](../../ros2_ws/src/teleop_ik/src/ik_node.cpp)
- [`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`](../../ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp)
- [`ros2_ws/src/teleop_ik/CMakeLists.txt`](../../ros2_ws/src/teleop_ik/CMakeLists.txt)
- [`ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml`](../../ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml)
- [`SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`](../../SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs)
- [`SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs`](../../SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs)
- [`SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions`](../../SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions)
- [`docs/superpowers/specs/2026-06-13-soarmvr-rosettadds-migration-design.md`](2026-06-13-soarmvr-rosettadds-migration-design.md) … C# msg 生成手順
- [`docs/superpowers/specs/2026-06-23-teleop-ik-cpp-rewrite-design.md`](2026-06-23-teleop-ik-cpp-rewrite-design.md) … 直近の C++ 化 spec
- [`docs/rosettadds-feature-gaps.md`](../../rosettadds-feature-gaps.md) … C# テスト事情
