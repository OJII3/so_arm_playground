# SoArmVR rclsharp → ROSettaDDS 移行 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SoArmVR の ROS 2 通信を `rclsharp` から `ROSettaDDS` へ移行し、geometry_msgs を新生成ツールで再生成し、ROS 2 実機 interop 設定を整え、ROSettaDDS の機能不足を報告する。

**Architecture:** 移行は実質リネーム（パッケージ・名前空間・asmdef）。geometry_msgs は ROSettaDDS 未同梱のため `rosettadds-genmsg` CLI で SoArmVR ローカル生成を継続。RosTeleoperationSink に NIC 自動検出・DdsTypeName 明示・トピック別 QoS を追加。dotnet SDK は `SoArmVR/nix/` 定義の専用 devShell `.#soarmvr` で供給。ROSettaDDS 本体は触らず、機能不足は `docs/` に報告。

**Tech Stack:** Unity 6000.3 (.NET Standard 2.1), C#, ROSettaDDS (`com.ojii3.rosettadds`), Nix flake, dotnet-sdk_8, uloop CLI。

**設計書:** `docs/superpowers/specs/2026-06-13-soarmvr-rosettadds-migration-design.md`

**前提:** ブランチ `feat/rosettadds-migration` で作業中。

---

## File Structure

- `SoArmVR/nix/dotnet-shell.nix` — **新規**: dotnet-sdk_8 を含む mkShell を返す callPackage 関数。
- `flake.nix` — **変更**: `devShells.<system>.soarmvr` を追加（`SoArmVR/nix/dotnet-shell.nix` を import）。
- `SoArmVR/Assets/Msgs/geometry_msgs/msg/*.msg` — **新規**: geometry_msgs の入力定義（Point/Vector3/Quaternion/Pose/PoseStamped）。
- `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/Geometry/*.cs` — **再生成**: 旧 `Rclsharp.Msgs.Geometry` を削除し `ROSettaDDS.Msgs.Geometry` で再生成。
- `SoArmVR/Packages/manifest.json` / `packages-lock.json` — **変更**: rclsharp → rosettadds。
- `SoArmVR/Assets/_SoArmVR/Scripts/_SoArmVR.asmdef` — **変更**: 参照 `Rclsharp` → `ROSettaDDS`。
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/LocalNetwork.cs` — **新規**: NIC 自動検出ヘルパー。
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs` — **変更**: 名前空間・interop 対応。
- `SoArmVR/README.md` — **新規/変更**: ROSettaDDS 参照と msg 再生成手順。
- `docs/rosettadds-feature-gaps.md` — **新規**: 機能不足の洗い出し報告。

---

## Task 1: dotnet 専用 devShell を nix に追加

**Files:**
- Create: `SoArmVR/nix/dotnet-shell.nix`
- Modify: `flake.nix`

- [ ] **Step 1: `SoArmVR/nix/dotnet-shell.nix` を作成**

```nix
# msg 再生成 (rosettadds-genmsg) 用の dotnet SDK を含む devShell.
# default シェルを汚さないよう, SoArmVR 専用ツールはここに分離する.
{
  mkShell,
  dotnet-sdk_8,
}:
mkShell {
  packages = [ dotnet-sdk_8 ];
  shellHook = ''
    echo "SoArmVR dotnet shell (rosettadds-genmsg 用)"
  '';
}
```

- [ ] **Step 2: `flake.nix` の devShells に `soarmvr` を追加**

`flake.nix` の `default = pkgs.mkShell { ... };` ブロック直後（`podman` を足している `//` の連鎖）に、`podman` 定義と同じ階層でもう一つ `//` ブロックを追加する。`default` ブロックを閉じる `}` の直後、`// {  podman = ...` の前に挿入する:

```nix
        // {
          soarmvr = pkgs.callPackage ./SoArmVR/nix/dotnet-shell.nix { };
        }
```

- [ ] **Step 3: シェルを評価して dotnet が入ることを確認**

Run: `nix develop .#soarmvr --command dotnet --version`
Expected: `8.0.4xx` のようなバージョンが出力され、エラーが無い。

- [ ] **Step 4: Commit**

```bash
git add flake.nix SoArmVR/nix/dotnet-shell.nix
git commit -m "feat(nix): SoArmVR msg 再生成用の dotnet devShell (.#soarmvr) を追加"
```

---

## Task 2: geometry_msgs の入力 .msg を作成

**Files:**
- Create: `SoArmVR/Assets/Msgs/geometry_msgs/msg/Point.msg`
- Create: `SoArmVR/Assets/Msgs/geometry_msgs/msg/Vector3.msg`
- Create: `SoArmVR/Assets/Msgs/geometry_msgs/msg/Quaternion.msg`
- Create: `SoArmVR/Assets/Msgs/geometry_msgs/msg/Pose.msg`
- Create: `SoArmVR/Assets/Msgs/geometry_msgs/msg/PoseStamped.msg`

- [ ] **Step 1: 5 つの `.msg` を作成**

`Point.msg`:
```
float64 x
float64 y
float64 z
```

`Vector3.msg`:
```
float64 x
float64 y
float64 z
```

`Quaternion.msg`:
```
float64 x
float64 y
float64 z
float64 w
```

`Pose.msg`（同一パッケージ相対参照）:
```
Point position
Quaternion orientation
```

`PoseStamped.msg`（`std_msgs/Header` はネスト参照。生成 .cs は本体同梱型を参照）:
```
std_msgs/Header header
Pose pose
```

- [ ] **Step 2: Commit**

```bash
git add SoArmVR/Assets/Msgs/geometry_msgs/msg/
git commit -m "feat(soarmvr): geometry_msgs の入力 .msg を追加"
```

---

## Task 3: geometry_msgs を rosettadds-genmsg で再生成

**Files:**
- Delete: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/Geometry/*.cs`（旧 `Rclsharp.Msgs.Geometry`）
- Create: 同ディレクトリに `ROSettaDDS.Msgs.Geometry` の生成物

- [ ] **Step 1: ROSettaDDS を浅くクローン（生成ツール取得）**

Run:
```bash
git clone --depth 1 https://github.com/OJII3/ROSettaDDS.git /tmp/rosettadds-genmsg-src
```
Expected: clone 成功。`/tmp/rosettadds-genmsg-src/tools/rosettadds-genmsg/` が存在する。

- [ ] **Step 2: genmsg CLI を実行して生成**

Run（リポジトリルートから）:
```bash
nix develop .#soarmvr --command dotnet run \
  --project /tmp/rosettadds-genmsg-src/tools/rosettadds-genmsg -- \
  --input  SoArmVR/Assets/Msgs \
  --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs
```
Expected: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/Geometry/` に `Point.cs` `Vector3.cs` `Quaternion.cs` `Pose.cs` `PoseStamped.cs` が出力される。

- [ ] **Step 3: 旧名前空間が残っていないこと・新名前空間で生成されたことを確認**

Run:
```bash
grep -rl "Rclsharp" SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/ ; echo "---"; grep -rn "namespace ROSettaDDS.Msgs.Geometry\|rosettadds-genmsg" SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/Geometry/Point.cs
```
Expected: 1 行目（`Rclsharp` 検索）は**何も出力されない**。2 行目で `namespace ROSettaDDS.Msgs.Geometry` と `rosettadds-genmsg` コメントが出る。

- [ ] **Step 4: CDR 互換性を目視確認**

`PoseStamped.cs` を開き、以下を確認:
- `using ROSettaDDS.Cdr;` と `using ROSettaDDS.Msgs.Std;` が入っている。
- `GetSerializedSize` が `HeaderSerializer.Instance.GetSerializedSize` + 整列パディング + `PoseSerializer...` の構成（旧実装と同じ加算順）。
- `Serialize` が Header → Pose の順。

旧実装（git で確認可能）とフィールド構成・順序が一致していること。差分は名前空間と生成元コメントのみが期待値。

- [ ] **Step 5: 旧 .meta の確認**

生成 `.cs` は Assets 配下なので Unity が `.meta` を自動付与する。既存 `.cs` を上書きした場合は既存 `.meta` が再利用される。ファイル名が変わっていなければ追加対応不要。

- [ ] **Step 6: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/
git commit -m "feat(soarmvr): geometry_msgs を rosettadds-genmsg で再生成 (ROSettaDDS.Msgs.Geometry)"
```

---

## Task 4: UPM パッケージと asmdef を移行

**Files:**
- Modify: `SoArmVR/Packages/manifest.json:10`
- Modify: `SoArmVR/Packages/packages-lock.json`
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/_SoArmVR.asmdef`

- [ ] **Step 1: manifest.json の依存を差し替え**

`SoArmVR/Packages/manifest.json` の行:
```json
    "com.ojii3.rclsharp": "https://github.com/OJII3/rclsharp.git?path=/src/rclsharp",
```
を次に置換:
```json
    "com.ojii3.rosettadds": "https://github.com/OJII3/ROSettaDDS.git?path=/src/rosettadds",
```

- [ ] **Step 2: asmdef の参照を差し替え**

`SoArmVR/Assets/_SoArmVR/Scripts/_SoArmVR.asmdef` の `references` 内 `"Rclsharp"` を `"ROSettaDDS"` に置換:
```json
    "references": [
        "Unity.InputSystem",
        "ROSettaDDS"
    ],
```

- [ ] **Step 3: packages-lock.json の rclsharp エントリを削除**

`SoArmVR/Packages/packages-lock.json` の `"com.ojii3.rclsharp": { ... }` ブロック（先頭の依存エントリ）を削除する。lock は次の Unity 起動/解決時に rosettadds 用に再生成されるため、ここでは rclsharp エントリの除去のみ行う（手で hash を書かない）。

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Packages/manifest.json SoArmVR/Packages/packages-lock.json SoArmVR/Assets/_SoArmVR/Scripts/_SoArmVR.asmdef
git commit -m "feat(soarmvr): UPM 依存と asmdef を rclsharp から rosettadds へ移行"
```

---

## Task 5: NIC 自動検出ヘルパーを追加

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/LocalNetwork.cs`

- [ ] **Step 1: `LocalNetwork.cs` を作成**

```csharp
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using UnityEngine;

namespace SoArmVR.Teleoperation
{
    /// <summary>
    /// ローカル NIC の IPv4 アドレスを解決するヘルパー。
    ///
    /// 本来 ROSettaDDS (DDS/rmw 層) が全 NIC を自動列挙してユニキャスト locator を
    /// 広告すべきだが、現状は既定で loopback を広告するため、LAN 上の ROS 2 と
    /// 通信するには実 NIC の IP を明示する必要がある。
    /// これはその暫定対応であり、ROSettaDDS がインターフェース自動列挙に対応すれば不要になる。
    /// 参照: docs/rosettadds-feature-gaps.md
    /// </summary>
    public static class LocalNetwork
    {
        /// <summary>
        /// up・非 loopback・IPv4 の最初の有効なユニキャストアドレスを返す。
        /// 見つからなければ null。
        /// </summary>
        public static IPAddress ResolvePrimaryIPv4()
        {
            foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (nic.OperationalStatus != OperationalStatus.Up) continue;
                if (nic.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;

                foreach (var ua in nic.GetIPProperties().UnicastAddresses)
                {
                    if (ua.Address.AddressFamily != AddressFamily.InterNetwork) continue;
                    if (IPAddress.IsLoopback(ua.Address)) continue;
                    return ua.Address;
                }
            }

            Debug.LogWarning(
                "LocalNetwork: 有効な IPv4 NIC が見つかりませんでした。loopback にフォールバックします。");
            return null;
        }
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/LocalNetwork.cs
git commit -m "feat(soarmvr): ローカル NIC の IPv4 を解決するヘルパーを追加"
```

---

## Task 6: RosTeleoperationSink を移行・interop 対応

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`

- [ ] **Step 1: using とエイリアスを ROSettaDDS に置換し、QoS を追加**

ファイル冒頭の using 群（1–9 行目）を次に置換:

```csharp
using System.Net;
using UnityEngine;
using ROSettaDDS.Dds;
using ROSettaDDS.Dds.QoS;
using ROSettaDDS.Msgs.Geometry;
using ROSettaDDS.Msgs.Std;
using ROSettaDDS.Msgs.BuiltinInterfaces;

using RosQuaternion = ROSettaDDS.Msgs.Geometry.Quaternion;
using RosPose = ROSettaDDS.Msgs.Geometry.Pose;
using RosTime = ROSettaDDS.Msgs.BuiltinInterfaces.Time;
```

- [ ] **Step 2: `InitParticipant` で NIC 自動検出と QoS / DdsTypeName を反映**

`InitParticipant()` の本体を次に置換:

```csharp
        void InitParticipant()
        {
            if (_participant != null) return;

            var localIp = LocalNetwork.ResolvePrimaryIPv4();
            var options = new DomainParticipantOptions
            {
                DomainId = 0,
                EntityName = "soarmvr",
                LocalUnicastAddress = localIp,   // null の場合 ROSettaDDS 側で loopback にフォールバック
                MulticastInterface = localIp,
            };
            _participant = new DomainParticipant(options);
            _participant.Start();

            // target_pose は高頻度なので sensor-data 相当の BestEffort。
            _posePub = _participant.CreatePublisher<PoseStamped>(
                "/teleop/target_pose",
                PoseStampedSerializer.Instance,
                ReliabilityQos.BestEffort,
                DurabilityQos.Volatile,
                PoseStamped.DdsTypeName);

            _gripperPub = _participant.CreatePublisher<Float64Message>(
                "/teleop/gripper",
                Float64MessageSerializer.Instance,
                Float64Message.DdsTypeName);

            _activePub = _participant.CreatePublisher<BoolMessage>(
                "/teleop/active",
                BoolMessageSerializer.Instance,
                BoolMessage.DdsTypeName);
        }
```

- [ ] **Step 3: 他に `Rclsharp` 参照が残っていないことを確認**

Run:
```bash
grep -rn "Rclsharp" SoArmVR/Assets/_SoArmVR/
```
Expected: **何も出力されない**。

- [ ] **Step 4: Commit**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs
git commit -m "feat(soarmvr): RosTeleoperationSink を ROSettaDDS へ移行し interop 設定を追加"
```

---

## Task 7: ROSettaDDS 機能不足の報告書を作成

**Files:**
- Create: `docs/rosettadds-feature-gaps.md`

- [ ] **Step 1: 報告書を作成**

```markdown
# ROSettaDDS 機能不足の洗い出し (SoArmVR 移行時)

- 日付: 2026-06-13
- 出典: SoArmVR の rclsharp → ROSettaDDS 移行作業
- 対象コミット範囲: ROSettaDDS `main` (調査時点)

本書は SoArmVR 側で確認した ROSettaDDS の機能不足・ドキュメント問題の記録。
本リポジトリではこれらを修正せず、ROSettaDDS 側での対応候補として報告する。

## 1. ローカルインターフェースの自動列挙が無い (機能不足)

`src/rosettadds/Dds/ParticipantTransportSet.cs` の `Create` は
`options.LocalUnicastAddress ?? IPAddress.Loopback` を使い、実 IP を渡さない限り
loopback のユニキャスト locator を SPDP/SEDP に広告する。

標準的な ROS 2 (Fast DDS / Cyclone DDS) では DDS/rmw 層が全 NIC を自動列挙し、
各インターフェースのユニキャスト locator を広告する。ユーザーは自分の IP を指定せず、
`ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` / インターフェース whitelist (任意のフィルタ) 程度しか
触らない。インターフェース列挙はライブラリ (rmw/DDS) が負担すべき責務であり、
現状は各利用側が NIC 検出を再実装する必要がある。

- 影響: LAN 上の ROS 2 と通信する全アプリが NIC 検出を自前実装する必要がある。
  SoArmVR では `Assets/_SoArmVR/Scripts/Teleoperation/LocalNetwork.cs` で暫定対応した。
- 対応候補: `DomainParticipantOptions.LocalUnicastAddress` が null のとき、全 (または既定経路の)
  NIC を自動列挙して locator を広告する。`ROS_LOCALHOST_ONLY` 相当のオプトインで loopback に限定。

## 2. geometry_msgs を同梱していない (機能不足/不足機能)

標準 msg は `std_msgs` / `builtin_interfaces` のみ同梱。`geometry_msgs` 等の一般的な
ROS 型は利用側で `.msg` を用意し生成する必要がある。

- 影響: ロボティクスで多用する geometry_msgs/sensor_msgs 等を使うたびに利用側で生成・保守が要る。
- 対応候補: よく使う標準パッケージ (geometry_msgs 等) の同梱、または「標準 msg パック」の提供。

## 3. README / docs のサンプルが実 API と不一致 (ドキュメント不正確)

`README.md` / `README.ja.md` の QoS publisher 例:

    participant.CreatePublisher<StringMessage>(
        "chatter", StringMessageSerializer.Instance,
        ReliabilityQos.BestEffort, StringMessage.DdsTypeName);

は 4 引数だが、実シグネチャは
`CreatePublisher<T>(string, ICdrSerializer<T>, ReliabilityQos, DurabilityQos, string?)`
(5 引数) または `CreatePublisher<T>(string, ICdrSerializer<T>, string?)` (3 引数) で、
この呼び出しはコンパイルできない (`durability` が必須)。

実装は正しく、ドキュメントのサンプルが誤っている。docs 全般が現行 API に追従できていない
可能性があり、要見直し。

- 影響: README どおりに書くとビルドエラー。
- 対応候補: サンプルを 5 引数版に修正、または `(string, ICdrSerializer<T>, ReliabilityQos, string?)`
  の便宜オーバーロードを追加 (durability 既定 Volatile)。
```

- [ ] **Step 2: Commit**

```bash
git add docs/rosettadds-feature-gaps.md
git commit -m "docs: ROSettaDDS の機能不足の洗い出し報告を追加"
```

---

## Task 8: SoArmVR README に再生成手順を記載

**Files:**
- Create または Modify: `SoArmVR/README.md`

- [ ] **Step 1: README を作成/更新**

`SoArmVR/README.md` が無ければ新規作成、あれば該当節を追記する:

```markdown
# SoArmVR

SO-101 の VR/AR teleoperation プロジェクト (Unity)。ROS 2 とは
[ROSettaDDS](https://github.com/OJII3/ROSettaDDS) (`com.ojii3.rosettadds`) で
pure C# RTPS/DDS により直接 pub/sub する。

## ROS 2 連携

- 参照パッケージ: `com.ojii3.rosettadds` (UPM git 依存, `Packages/manifest.json`)
- 標準 msg (`std_msgs` / `builtin_interfaces`) は ROSettaDDS に同梱。
- `geometry_msgs` は ROSettaDDS 未同梱のため、当プロジェクトで生成・保守する
  (`Assets/Msgs/geometry_msgs/`, 生成物は `Assets/_SoArmVR/Scripts/GeneratedMsgs/`)。

### publish するトピック

| トピック | 型 | QoS |
| --- | --- | --- |
| `/teleop/target_pose` | `geometry_msgs/PoseStamped` | BestEffort |
| `/teleop/gripper` | `std_msgs/Float64` | Reliable |
| `/teleop/active` | `std_msgs/Bool` | Reliable |

## msg の再生成

`.msg` を変更したら `rosettadds-genmsg` で再生成する。dotnet SDK は専用 devShell に含まれる。

\`\`\`sh
# リポジトリルートで実行。ROSettaDDS は別途クローンしておく。
git clone --depth 1 https://github.com/OJII3/ROSettaDDS.git /tmp/rosettadds-genmsg-src
nix develop .#soarmvr --command dotnet run \
  --project /tmp/rosettadds-genmsg-src/tools/rosettadds-genmsg -- \
  --input  SoArmVR/Assets/Msgs \
  --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs
\`\`\`
```

> 注: 上記コードブロック内の `\`\`\`` はそのまま 3 連バッククォートで記述する（エスケープ不要、ここでは表示の都合でエスケープ表記）。

- [ ] **Step 2: Commit**

```bash
git add SoArmVR/README.md
git commit -m "docs(soarmvr): ROSettaDDS 連携と msg 再生成手順を README に追記"
```

---

## Task 9: Unity でコンパイル検証

**Files:** なし（検証のみ）

- [ ] **Step 1: Unity を起動/リフレッシュしてパッケージ解決とコンパイルを確認**

Unity Editor が起動済みであること（必要なら `nix develop --command uloop launch`）。
その上でコンパイルしコンソールを取得する:
```bash
nix develop --command uloop clear-console
nix develop --command uloop compile
nix develop --command uloop get-logs
```
Expected:
- `com.ojii3.rosettadds` が解決され `packages-lock.json` が rosettadds で更新される。
- コンパイルエラー・コンソールエラーが無い。
- `Rclsharp` 由来の未解決参照エラーが無い。

- [ ] **Step 2: 解決後の packages-lock.json をコミット**

Unity がパッケージ解決して lock を書き換えた場合:
```bash
git add SoArmVR/Packages/packages-lock.json
git commit -m "chore(soarmvr): rosettadds 解決後の packages-lock を更新"
```

- [ ] **Step 3: 実行時に解決 IP がログされることを確認（任意・環境があれば）**

Play して `LocalNetwork` の警告が出ない（= 実 NIC が取れている）こと、
`nix develop .#ros` 側で `ros2 topic list` に `/teleop/*` が見えることを確認。
LAN/実機環境が無ければコンパイル成功＋型名一致までで完了とする。

---

## 完了条件

- `grep -rn "Rclsharp" SoArmVR/Assets/ SoArmVR/Packages/` が空。
- Unity でコンパイルエラーなし。
- geometry_msgs が `ROSettaDDS.Msgs.Geometry` で再生成済み。
- `docs/rosettadds-feature-gaps.md` に 3 件の報告。
- `.#soarmvr` で dotnet が使える。
