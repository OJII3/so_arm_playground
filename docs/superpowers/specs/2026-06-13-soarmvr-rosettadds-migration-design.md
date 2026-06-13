# SoArmVR: rclsharp → ROSettaDDS 移行 設計書

- 日付: 2026-06-13
- 対象: `SoArmVR/`（Unity プロジェクト）
- ブランチ: `feat/rosettadds-migration`

## 1. 背景とゴール

SoArmVR は ROS 2 と pure C# で通信するため `rclsharp`（`com.ojii3.rclsharp`）を利用していた。
`rclsharp` は **ROSettaDDS**（`com.ojii3.rosettadds`, `github.com/OJII3/ROSettaDDS`）にリニューアルされた。
本作業で SoArmVR を ROSettaDDS へ移行し、メッセージを新しい生成機能で再生成し、interop 設定を整え、
ROSettaDDS 側に残る機能不足を洗い出す。

### 確定方針

| 項目 | 決定 |
| --- | --- |
| 作業範囲 | `so_arm_playground` 内のみ。ROSettaDDS 本体の修正・PR は出さず、機能不足は**報告のみ** |
| geometry_msgs | ROSettaDDS 未同梱のため **SoArmVR でローカル再生成を継続** |
| interop 設定 | 整える（NIC 自動検出・DdsTypeName 明示・トピック別 QoS） |
| NIC 指定 | **自動検出のみ**（最小）。ただし本来 ROSettaDDS が担うべき責務として報告 |
| QoS | `/teleop/target_pose` は BestEffort、`/teleop/gripper`・`/teleop/active` は Reliable |
| dotnet SDK | `SoArmVR/nix/` に定義し、flake.nix から専用 devShell `.#soarmvr` として公開 |

### API 調査結果（重要）

旧 `rclsharp` と `ROSettaDDS` の公開 API はほぼ同一。`CreatePublisher` / `CreateSubscription` /
`DomainParticipantOptions` のシグネチャは一致しており、移行は実質**リネーム中心**。
唯一 SoArmVR がローカルで補っているのは `geometry_msgs`（ROSettaDDS は `std_msgs` /
`builtin_interfaces` のみ同梱）。

## 2. パッケージ移行

### 2.1 UPM 依存

`SoArmVR/Packages/manifest.json`:
- 削除: `"com.ojii3.rclsharp": "https://github.com/OJII3/rclsharp.git?path=/src/rclsharp"`
- 追加: `"com.ojii3.rosettadds": "https://github.com/OJII3/ROSettaDDS.git?path=/src/rosettadds"`

`SoArmVR/Packages/packages-lock.json`:
- 対応するエントリを差し替える。最終的に Unity がパッケージを解決し直して lock を確定させる
  （hash は Unity が再計算）。

### 2.2 asmdef

`SoArmVR/Assets/_SoArmVR/Scripts/_SoArmVR.asmdef`:
- `references` の `"Rclsharp"` → `"ROSettaDDS"`。

### 2.3 名前空間

SoArmVR 内の全 `.cs` で `Rclsharp.*` → `ROSettaDDS.*`:
- `RosTeleoperationSink.cs` の `using Rclsharp.Dds;` / `Rclsharp.Msgs.Std` /
  `Rclsharp.Msgs.BuiltinInterfaces` / `Rclsharp.Msgs.Geometry` と、エイリアス
  （`RosQuaternion = Rclsharp.Msgs.Geometry.Quaternion` 等）の右辺。

## 3. メッセージ再生成（geometry_msgs）

ROSettaDDS は `geometry_msgs` を同梱しないため、SoArmVR でローカル生成を継続する。

### 3.1 入力 `.msg`

`SoArmVR/Assets/Msgs/geometry_msgs/msg/` に標準 ROS 定義を新規作成しコミットする:
- `Point.msg`, `Vector3.msg`（`float64 x/y/z`）
- `Quaternion.msg`（`float64 x/y/z/w`、各 default 値も標準どおり）
- `Pose.msg`（`Point position` / `Quaternion orientation`、同一パッケージ相対参照）
- `PoseStamped.msg`（`std_msgs/Header header` / `Pose pose`）
  - ネストの `std_msgs/Header` は ROSettaDDS 本体同梱型を参照する `.cs` が生成される（再生成不要）。

### 3.2 生成

```sh
nix develop .#soarmvr   # dotnet-sdk_8 を含む（§6）
dotnet run --project <ROSettaDDS>/tools/rosettadds-genmsg -- \
  --input  SoArmVR/Assets/Msgs \
  --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs
```

- 生成型は `ROSettaDDS.Msgs.Geometry` 名前空間、`auto-generated` コメントは `rosettadds-genmsg` 由来になる。
- 既存の旧 `Rclsharp.Msgs.Geometry` 生成物（`GeneratedMsgs/Geometry/*.cs`）は削除し、再生成物で置換。
- `<ROSettaDDS>` は利用者環境のローカルクローンパス。再生成手順を `SoArmVR/README`（無ければ新規）に記載。

### 3.3 CDR 互換性

生成される `Serialize` / `Deserialize` / `GetSerializedSize` は旧手書き/生成物と CDR バイト列が一致する
（ROSettaDDS 側で bit-exact 回帰テストにより担保済み）。差分は名前空間と生成元コメントのみのはず。

## 4. RosTeleoperationSink の interop 対応

### 4.1 NIC 自動検出（ROSettaDDS ギャップの暫定カバー）

`DomainParticipantOptions` は既定で loopback の locator を広告するため、LAN 上の ROS 2 から到達できない。
本来 DDS/rmw 層（ROSettaDDS）が全 NIC を自動列挙すべき責務（§7-1）だが、暫定的に SoArmVR 側で補う。

- `NetworkInterface.GetAllNetworkInterfaces()` から「`OperationalStatus.Up` / 非 loopback / IPv4 /
  有効なユニキャストアドレスを持つ」最初の NIC を選ぶヘルパー（例: `LocalNetwork.ResolvePrimaryIPv4()`）を追加。
- 取得した IP を `DomainParticipantOptions { DomainId = 0, EntityName = "soarmvr",
  LocalUnicastAddress = ip, MulticastInterface = ip }` に渡す。
- 見つからない場合は既定（loopback）にフォールバックし、`Debug.LogWarning` で通知。
- コメントに「ROSettaDDS のインターフェース自動列挙が入れば不要になる暫定対応」と明記。

### 4.2 DdsTypeName 明示

`CreatePublisher` 呼び出しに生成型の `DdsTypeName` 定数を渡す（ROS 2 との型名マッチを明示）。

### 4.3 トピック別 QoS

| トピック | QoS | 呼び出し |
| --- | --- | --- |
| `/teleop/target_pose` | BestEffort | `CreatePublisher(topic, PoseStampedSerializer.Instance, ReliabilityQos.BestEffort, DurabilityQos.Volatile, PoseStamped.DdsTypeName)` |
| `/teleop/gripper` | Reliable | `CreatePublisher(topic, Float64MessageSerializer.Instance, Float64Message.DdsTypeName)` |
| `/teleop/active` | Reliable | `CreatePublisher(topic, BoolMessageSerializer.Instance, BoolMessage.DdsTypeName)` |

`using ROSettaDDS.Dds.QoS;` を追加。BestEffort を使う `target_pose` は `durability` 必須のため 5 引数版を用いる。
購読側（ROS 2 ノード）も `target_pose` を sensor-data 相当の BestEffort で受ける前提とする。

## 5. 削除する「補い」と維持するもの

- **削除**: 旧 `Rclsharp.Msgs.Geometry` のローカル生成物（名前空間刷新で置換）。
- **維持**: `async void` + `ObjectDisposedException` ガード、手動 `Time` スタンプ生成。
  これらは rclsharp の機能不足の回避ではなく Unity 側の妥当な実装。不要な変更は行わない。
- 精査結果、「機能不足でカバーしていた部分」の実体は geometry_msgs のローカル生成のみだった
  （NIC 自動検出は今回 interop 対応として新規に追加するカバー）。

## 6. nix への dotnet 追加

- `SoArmVR/nix/dotnet-shell.nix`: `mkShell { packages = [ dotnet-sdk_8 ]; }` 相当を定義（`callPackage` 可能な形）。
- `flake.nix`: `.#ros` が `ros2_ws/nix/shell.nix` を import するパターンに倣い、
  `devShells.<system>.soarmvr` として `SoArmVR/nix/dotnet-shell.nix` を import。
- default シェル（`commonPackages`）には dotnet を**入れない**（モノレポのルート汚染回避）。
- 用途は msg 再生成。`nix develop .#soarmvr` で利用。

## 7. ROSettaDDS 機能不足の洗い出し（報告のみ）

本リポジトリ内に記録する（修正・PR は出さない）。

1. **ローカルインターフェースの自動列挙が無い（機能不足）**
   `ParticipantTransportSet.Create` は `LocalUnicastAddress ?? IPAddress.Loopback` で、実 IP を渡さない限り
   loopback locator を広告する。標準的な ROS 2（Fast DDS / Cyclone DDS）は DDS/rmw 層が全 NIC を自動列挙して
   ユニキャスト locator を広告し、ユーザーは自 IP を指定しない。**インターフェース列挙はライブラリが負担すべき
   責務**であり、現状は各利用側が NIC 検出を再実装する必要がある（SoArmVR でも §4.1 で暫定実装）。

2. **geometry_msgs を同梱していない**
   標準 msg は `std_msgs` / `builtin_interfaces` のみ。`geometry_msgs` 等の一般的な ROS 型は各利用側で
   `.msg` 用意＋生成が必要。よく使う型の同梱、または「標準 msg パック」提供の余地。

3. **README / docs のサンプルが実 API と不一致（ドキュメント不正確）**
   `README` の QoS publisher 例 `CreatePublisher(topic, ser, ReliabilityQos.BestEffort, DdsTypeName)`（4 引数）は
   実シグネチャ（`durability` 必須の 5 引数版、または typeName のみの 3 引数版）と一致せずコンパイルできない。
   **実装は正しく、ドキュメントの誤り**。docs 全般が現行 API に追従できていない可能性があり、要見直し。

## 8. 検証

- `nix develop .#soarmvr` で genmsg が動作し、`GeneratedMsgs/Geometry/*.cs` が `ROSettaDDS.Msgs.Geometry`
  名前空間で生成されること。
- Unity（`uloop` 経由）でコンパイルエラー / コンソールエラーが無いこと。
- 生成 `.cs` のフィールド構成・`GetSerializedSize` が旧実装と一致すること（目視 diff）。
- 可能なら `nix develop .#ros` 側で `ros2 topic list` / `ros2 topic echo /teleop/target_pose` により
  型名・QoS マッチと疎通を確認（実機 LAN 環境が必要。最低限コンパイル＋型名一致まで保証）。

## 9. スコープ外

- ROSettaDDS 本体への変更（geometry_msgs 同梱・NIC 自動列挙・docs 修正）。報告のみ。
- Inspector 設定化・DomainId 可変化（自動検出のみ＝最小実装）。
- 後方互換のためのフォールバック実装（loopback フォールバック＋警告ログを除く）。
