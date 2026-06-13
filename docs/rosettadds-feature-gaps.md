# ROSettaDDS 機能不足の洗い出し (SoArmVR 移行時)

- 日付: 2026-06-13
- 出典: SoArmVR の rclsharp → ROSettaDDS 移行作業
- 対象コミット: ROSettaDDS `main` `8d1c383` (調査時点)

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
  SoArmVR では `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/LocalNetwork.cs` で暫定対応した。
- 対応候補: `DomainParticipantOptions.LocalUnicastAddress` が null のとき、全 (または既定経路の)
  NIC を自動列挙して locator を広告する。`ROS_LOCALHOST_ONLY` 相当のオプトインで loopback に限定。

## 2. geometry_msgs を同梱していない (不足機能)

標準 msg は `std_msgs` / `builtin_interfaces` のみ同梱。`geometry_msgs` 等の一般的な
ROS 型は利用側で `.msg` を用意し生成する必要がある。

- 影響: ロボティクスで多用する geometry_msgs/sensor_msgs 等を使うたびに利用側で生成・保守が要る。
  SoArmVR では `SoArmVR/Assets/Msgs/geometry_msgs/` に `.msg` を置き、`rosettadds-genmsg` で
  `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/` に生成・コミットして対応した。
- 対応候補: よく使う標準パッケージ (geometry_msgs 等) の同梱、または「標準 msg パック」の提供。

## 3. README / docs のサンプルが実 API と不一致 (ドキュメント不正確)

`README.md` / `README.ja.md` の QoS publisher 例:

    participant.CreatePublisher<StringMessage>(
        "chatter", StringMessageSerializer.Instance,
        ReliabilityQos.BestEffort, StringMessage.DdsTypeName);

は 4 引数だが、実シグネチャ (`src/rosettadds/Dds/DomainParticipant.cs`) は

    // 3 引数 (既定 Reliable / Volatile)
    public Publisher<T> CreatePublisher<T>(string topicName, ICdrSerializer<T> serializer, string? typeName = null)

    // 5 引数 (QoS 明示)
    public Publisher<T> CreatePublisher<T>(
        string topicName, ICdrSerializer<T> serializer,
        ReliabilityQos reliability, DurabilityQos durability, string? typeName = null)

の 2 つのみで、4 引数 `(string, ICdrSerializer<T>, ReliabilityQos, string?)` のオーバーロードは
存在しない。よって README どおりに書くと `durability` が欠けてコンパイルできない
(BestEffort を使うには 5 引数版で `DurabilityQos.Volatile` 等を渡す必要がある)。

実装は正しく、ドキュメントのサンプルが誤っている。docs 全般が現行 API に追従できていない
可能性があり、要見直し。

- 影響: README どおりに書くとビルドエラー。実際 SoArmVR 移行のコードレビューでも、この不一致を
  「実装側のバグ」と誤認するレビューが発生した (実際はドキュメントの誤り)。
- 対応候補: サンプルを 5 引数版に修正、または
  `(string, ICdrSerializer<T>, ReliabilityQos, string?)` の便宜オーバーロードを追加
  (durability 既定 Volatile)。
