# SoArmVR

SO-101 の VR/AR teleoperation プロジェクト (Unity)。ROS 2 とは
[ROSettaDDS](https://github.com/OJII3/ROSettaDDS) (`com.ojii3.rosettadds`) で
pure C# RTPS/DDS により直接 pub/sub する。

## ROS 2 連携

- 参照パッケージ: `com.ojii3.rosettadds` (UPM git 依存, `Packages/manifest.json`)
- `std_msgs` / `builtin_interfaces` / `geometry_msgs` などの標準 msg は ROSettaDDS に同梱。
  当プロジェクトでは独自 msg を持たない。
- LAN 上の ROS 2 とは `DomainParticipantOptions` 既定 (NIC 自動列挙) で直接 pub/sub する。

### publish するトピック

| トピック | 型 | QoS |
| --- | --- | --- |
| `/teleop/target_pose` | `geometry_msgs/PoseStamped` | BestEffort |
| `/teleop/gripper` | `std_msgs/Float64` | Reliable |
| `/teleop/active` | `std_msgs/Bool` | Reliable |

> [!IMPORTANT]
> `/teleop/target_pose` は BestEffort で publish する。ROS 2 側の subscriber が
> デフォルト (Reliable) のままだと DDS の QoS 非互換でマッチせず**メッセージが届かない**。
> 購読側も `reliability=BEST_EFFORT` (例: `rclpy` の `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)`) に設定すること。

## 入力アクション

- **Reset**: 右コントローラ A ボタン. SO-101 を home 姿勢に戻す (IK バイパス).

## 独自 msg を追加する場合

標準 msg は ROSettaDDS 同梱なので通常は生成不要。ROS 2 標準に無い独自 msg が必要に
なった場合のみ、`rosettadds-genmsg` で C# 型を生成する。dotnet SDK は専用 devShell
(`.#soarmvr`) に含まれる。リポジトリルートで実行する。

```sh
# 生成ツールを含む ROSettaDDS を取得 (未取得の場合)
git clone --depth 1 https://github.com/OJII3/ROSettaDDS.git /tmp/rosettadds-genmsg-src

# 例: Assets/Msgs/<package>/msg/*.msg を生成して Assets 配下へ出力する
nix develop .#soarmvr --command dotnet run \
  --project /tmp/rosettadds-genmsg-src/tools/rosettadds-genmsg -- \
  --input  SoArmVR/Assets/Msgs \
  --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs
```

生成型は ROSettaDDS 同梱の `ROSettaDDS.Cdr` / `ROSettaDDS.Msgs.*` を参照する。
