# teleop_ik: 手首制御の分割 (IK: joint 1-4, FK: joint 5) 設計書

- 日付: 2026-06-23
- 対象: `ros2_ws/src/teleop_ik/`
- ブランチ: (未作成, 実装着手時に切る)

## 1. 背景とゴール

`teleop_ik_node` は VR / gamepad からの target pose を
`pinocchio` の CLIK (Closed-Loop Inverse Kinematics) で
関節軌道に変換し, SO-101 follower へ `JointTrajectory` を
publish するノード. 現状の関節分担は次のとおり.

| 関節 | 役割 | 制御源 |
| --- | --- | --- |
| joint 1, 2, 3 | IK (3D 位置ターゲット) | pose の delta |
| joint 4 | FK (q_seed で固定) | stick_y 速度積分 |
| joint 5 | FK (q_seed で固定) | stick_x 速度積分 |

ソルバは `position_joint_ids_ = [1,2,3]` の 3DOF だけで
ヤコビアン 3x3 を構成し, 手首 2 関節はソルバ外で
`q_seed` に積分値を上書きして「IK ループ中は触らない」
形で実現している.

これに対し, 次の挙動を要望する:

- joint 1, 2, 3, 4 を IK ソルバの対象に含める (4DOF,
  3D 位置ターゲットなので 1 自由度冗長).
- joint 5 のみを従来どおり FK (stick_x の速度積分) で
  直接制御する.
- stick_y 由来の joint 4 制御は「q_seed の bias として
  残す」. つまり IK の開始値には反映するが, ソルバが
  位置制約を満たすために動かしてしまうのは許容する.

ゴール:

- 上記の制御分担へ変更し, 既存のトピック・パラメータ・
  joint 名は変えない.
- 既存の trajectory publish の joint 並び (`"1"〜"5"`)
  はそのまま.
- テストを gtest で更新・追加し, ビルドと全テストが
  通る.

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| IK 対象関節 | joint 1, 2, 3, 4 (`position_joint_ids_` のサイズを 4 へ) |
| FK 対象関節 | joint 5 のみ (`wrist_joint_ids_` のサイズを 1 へ, 中身は `arm_joint_ids_[4]`) |
| stick_y → joint 4 | q_seed への上書きとして継続. IK ソルバの bias 扱い. |
| stick_x → joint 5 | FK として継続. ソルバは触らない. `q_solution_[idx_q_5]` に別途書き戻す. |
| 既存の冗長 DOF 解消 | 何もしない. CLIK のまま, 4DOF vs 3D 制約の冗長性は q_seed と数値誤差に任せる. |
| パラメータ | 追加・変更なし (`stick_*` 系は現行どおり) |
| トピック / メッセージ | 変更なし |
| launch / YAML | 変更なし |

## 3. アーキテクチャ

### 3.1 変更後のソルバ構成

```
position_joint_ids_ = [joint 1, joint 2, joint 3, joint 4]   // IK ソルバが触れる関節
wrist_joint_ids_    = [joint 5]                               // FK (stick 由来)
```

ヤコビアンは `J ∈ R^{3×4}` (3 制約 × 4 関節).
`damped least squares` で `dq_pos ∈ R^4` を求め,
`pinocchio::integrate` で 1 ステップ反映する流れは
現行と同じ.

### 3.2 q_seed の組み立て

`on_target_with_input` 内で従来どおり
`q_seed = q_solution_` から始める. その後:

- `q_seed[idx_q_4] = wrist_init_pos_.y() + integrated_stick_.y()`
  ← **継続**. セッション開始時に保存した joint 4 の
  初期値にスティック積分量を足したものを IK の初期値
  として与える.
- `q_seed[idx_q_5]` ← **何もしない**. すなわち
  `q_solution_` の前回値 (初回は `q_current_`) をそのまま
  使う. これは stick 由来の値ではない.

### 3.3 IK 結果への joint 5 注入

`solve_ik` は joint 5 を変更しない (position_joint_ids_
に含まれないため). 戻り値の `q_solution_` を受け取った
側で次の代入を行う.

```cpp
q_solution_[idx_q_5] =
    wrist_init_pos_.x() + integrated_stick_.x();
```

これで `make_arm_trajectory` (既存) が全 5 関節を
`q_solution_` から読むため, joint 5 は FK 値,
joint 1-4 は IK 値としてそのまま publish される.

### 3.4 セッション開始時の wrist 初期化

`on_active(true)` の経路で `q_current_` から
`wrist_init_pos_` を取り出す処理は, 軸マッピングを
維持したまま継続する:

- `wrist_init_pos_.y() = q_current_[idx_q_4]`  (stick_y 軸, joint 4 の初期値)
- `wrist_init_pos_.x() = q_current_[idx_q_5]`  (stick_x 軸, joint 5 の初期値)

joint 4 の値は IK の seed bias に使われ続けるので,
この取得は引き続き必要.

## 4. 変更点

### 4.1 `include/teleop_ik/ik_node.hpp`

- `std::array<pinocchio::JointIndex, 3> position_joint_ids_{};`
  → `std::array<pinocchio::JointIndex, 4> position_joint_ids_{};`
- `std::array<pinocchio::JointIndex, 2> wrist_joint_ids_{};`
  → `std::array<pinocchio::JointIndex, 1> wrist_joint_ids_{};`

### 4.2 `src/ik_node.cpp`

`init_ros_node` 内:

```cpp
for (size_t i = 0; i < 4; ++i) {
  position_joint_ids_[i] = arm_joint_ids_[i];   // 1, 2, 3, 4
}
wrist_joint_ids_[0] = arm_joint_ids_[4];         // 5 のみ
```

`make_for_test` も同様に更新.

`on_active(true)` の wrist 初期化処理は, 配列の
意味付けを整理したうえで継続する.

- `wrist_joint_ids_[0]` は新方式では **joint 5** を指す
  (従来は joint 4 を指していた `wrist_joint_ids_[0]`
  と joint 5 を指していた `wrist_joint_ids_[1]` のうち,
  joint 5 側のみを添字 0 に集約する).
- joint 4 は `wrist_joint_ids_` には含めず, 
  `arm_joint_ids_[3]` から直接 `idx_q_4` を取り出して
  `wrist_init_pos_.y()` に保存する.
- joint 5 は `wrist_joint_ids_[0]` (= `arm_joint_ids_[4]`)
  から `idx_q_5` を取り出して `wrist_init_pos_.x()` に
  保存する.

`on_target_with_input` 内の q_seed 組み立て:

- `q_seed[idx_q_4] = wrist_init_pos_.y() + integrated_stick_.y();`
  を継続.
- `q_seed[idx_q_5]` への上書きブロックは削除.

`on_target_with_input` 内の IK 結果処理:

```cpp
if (auto result = solve_ik(target_pos, q_seed, ...); result.has_value()) {
  q_solution_ = *result;
  // joint 5 はソルバ外で FK 値を流し込む
  if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_5 = model_.joints[wrist_joint_ids_[0]].idx_q();
    q_solution_[idx_q_5] =
        wrist_init_pos_.x() + integrated_stick_.x();
  }
  return true;
}
```

> `wrist_joint_ids_[0]` は新方式では joint 5 を指す.
> 変数名のリネーム (`fk_joint_ids_` など) も検討したが,
> スコープを抑えるため, 配列サイズだけ縮めて添字 0 を
> joint 5 に割り当てる最小変更にとどめる. コメントで
> 意図を明示する.

### 4.3 `test/test_ik_node_helpers.cpp`

- `SolveIkKeepsWristJointTargetsFixed` を改訂:
  - テスト名を `SolveIkKeepsJoint5Fixed` に変更.
  - joint 5 のみが保持されることを検証 (joint 4 は
    IK に組み入れたので seed 値から動くことを許容).
- `SolveIkAdjustsJoint4` を新規追加:
  - `position_joint_ids_` に joint 4 が含まれることを
    検証 (例: `node_->position_joint_ids_.size() == 4`).
  - 任意で, 異なる joint 4 シードから始めてもソルバが
    位置ターゲットに到達できることを検証.
- `WristResetsOnSessionStart`:
  - `wrist_init_pos_.x()` (joint 5) および
    `wrist_init_pos_.y()` (joint 4) の両方が,
    `q_current_` から正しくセットされることを検証する.
  - 取得経路の変更 (joint 4 は `arm_joint_ids_[3]`,
    joint 5 は `wrist_joint_ids_[0]`) についても
    テストで確認できるようにする.

### 4.4 CMakeLists.txt / launch / config

変更なし.

## 5. テスト戦略

| テスト | 検証内容 |
| --- | --- |
| `SolveIkReturnsNulloptForUnreachableTarget` | 変更なし. 4DOF でも非可達ターゲットには nullopt. |
| `SolveIkReachesCurrentPosition` | 変更なし. 現在姿勢に収束. |
| `SolveIkConvergesForReachablePositionTarget` | 変更なし. |
| `SolveIkKeepsJoint5Fixed` (改訂) | seed の joint 5 値が結果でも保持される. |
| `SolveIkAdjustsJoint4` (新規) | `position_joint_ids_` に joint 4 が含まれ, IK 結果が変わる. |
| `OnTargetWithInputIntegratesStickPerMessage` | 変更なし. stick 積分は継続. |
| `FirstMessageSetsAnchorWithoutIntegration` | 変更なし. |
| `TargetUsesPreviousSuccessfulSolutionAsNextSeed` | 変更なし. |
| `WristResetsOnSessionStart` (改訂) | `wrist_init_pos_` の x, y 両方が `q_current_` から正しくセットされることを検証. |
| `StickDeadzoneBlocksSmallInputs` | 変更なし. |
| `StickMaxDeltaPerMsgClampsIntegration` | 変更なし. |

## 6. マイルストーン

| # | 内容 | 検証 |
| --- | --- | --- |
| 1 | ヘッダの `position_joint_ids_` / `wrist_joint_ids_` サイズ変更, `init_ros_node` / `make_for_test` の joint id 設定更新 | `colcon build` 成功 |
| 2 | `on_target_with_input` の q_seed 上書きを joint 4 のみにし, IK 後に joint 5 を `q_solution_` に注入 | ビルド成功 |
| 3 | テスト改訂・追加 (`SolveIkKeepsJoint5Fixed`, `SolveIkAdjustsJoint4`, `WristResetsOnSessionStart`) | `colcon test` 全パス |
| 4 | 既存 launch 経由で起動し, joint 並び・パラメータ・トピックが従来どおり公開されることを確認 | `ros2 node list` 等で目視 |

## 7. リスクと対策

| リスク | 対策 |
| --- | --- |
| 4DOF vs 3D 制約で冗長 DOF が意図しない方向に動き, 実機姿勢が破綻する | `position_joint_ids_` の seed に stick_y bias を残しているので, スティックを触らない状態ではほぼ現行の姿勢に収束する想定. マイルストーン 4 で挙動を観察. |
| joint 5 を `q_solution_` に直接書き戻すことで, `q_seed = q_solution_` の経路で値が変わる | 想定内. `q_seed[idx_q_5]` はあえて上書きしない設計. 前回 IK 後の joint 5 値 (= 直近の stick 積分値) が次回 seed として使われる. |
| `wrist_joint_ids_[0]` の意味が「joint 4」から「joint 5」へ変わる | コメントで明示し, テスト名 (`SolveIkKeepsJoint5Fixed`) でも判別可能にする. |
| 既存の `SolveIkKeepsWristJointTargetsFixed` テストが壊れる | テスト自体を改訂し, joint 5 のみ保持の検証に置き換える. |

## 8. 範囲外

- IK アルゴリズムの変更 (CLIK のままだが, 4DOF 化に伴う
  ソルバの数値的安定性改善はしない).
- stick → 関節のマッピングの追加変更 (stick_y → joint 5
  など). 今回の変更とは別件.
- パラメータ追加.
- launch / config / メッセージ定義の変更.

## 9. 関連ドキュメント

- [`ros2_ws/src/teleop_ik/src/ik_node.cpp`](../../ros2_ws/src/teleop_ik/src/ik_node.cpp)
- [`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`](../../ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp)
- [`docs/superpowers/specs/2026-06-23-teleop-ik-cpp-rewrite-design.md`](2026-06-23-teleop-ik-cpp-rewrite-design.md) … 直近の C++ 化 spec
- [`docs/superpowers/specs/2026-06-21-soarmvr-anchor-projection-stick-wrist-design.md`](2026-06-21-soarmvr-anchor-projection-stick-wrist-design.md) … anchor + stick 手首 spec
