# SoArmVR

SO-101 の VR/AR teleoperation プロジェクト (Unity)。ROS 2 とは
[ROSettaDDS](https://github.com/OJII3/ROSettaDDS) (`com.ojii3.rosettadds`) で
pure C# RTPS/DDS により直接 pub/sub する。

## ROS 2 連携

- 参照パッケージ: `com.ojii3.rosettadds` (UPM git 依存, `Packages/manifest.json`)
- 標準 msg (`std_msgs` / `builtin_interfaces`) は ROSettaDDS に同梱。
- `geometry_msgs` は ROSettaDDS 未同梱のため、当プロジェクトで生成・保守する
  (`Assets/Msgs/geometry_msgs/`, 生成物は `Assets/_SoArmVR/Scripts/GeneratedMsgs/`)。
- LAN 上の ROS 2 と通信するため、`RosTeleoperationSink` は `LocalNetwork.ResolvePrimaryIPv4()`
  で実 NIC の IPv4 を検出し `DomainParticipantOptions` に渡す
  (ROSettaDDS が NIC 自動列挙に未対応なための暫定対応。`docs/rosettadds-feature-gaps.md` 参照)。

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

## msg の再生成

`.msg` を変更したら `rosettadds-genmsg` で再生成する。dotnet SDK は専用 devShell
(`.#soarmvr`) に含まれる。リポジトリルートで実行する。

```sh
# 生成ツールを含む ROSettaDDS を取得 (未取得の場合)
git clone --depth 1 https://github.com/OJII3/ROSettaDDS.git /tmp/rosettadds-genmsg-src

# 再生成 (入力: Assets/Msgs, 出力: Assets/_SoArmVR/Scripts/GeneratedMsgs)
nix develop .#soarmvr --command dotnet run \
  --project /tmp/rosettadds-genmsg-src/tools/rosettadds-genmsg -- \
  --input  SoArmVR/Assets/Msgs \
  --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs
```

生成型は `ROSettaDDS.Msgs.Geometry` 名前空間で出力され、ROSettaDDS 同梱の
`ROSettaDDS.Cdr` / `ROSettaDDS.Msgs.Std` を参照する。
