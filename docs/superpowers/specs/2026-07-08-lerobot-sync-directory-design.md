# Design: lerobot/ ディレクトリ追加 (LeRobot CLI 環境)

## 概要

SO-101 の leader arm と follower arm を公式 LeRobot CLI で同期させ、ACT がそのまま使える環境を `lerobot/` ディレクトリに作る。

## 背景

- リポジトリは ROS 2 を使った制御 (`ros2_ws/`) が既にある
- 公式 LeRobot は Python ネイティブで `lerobot-teleoperate` / `lerobot-record` / `lerobot-train` / `lerobot-rollout` を提供
- README 方針では `act/`, `datasets/`, `tools/` など責務ごとにトップレベルを分ける
- Python は `uv` で管理する

## 範囲

追加するもの:

- `lerobot/` ディレクトリ
- `lerobot/pyproject.toml` — `uv` プロジェクト定義
- `lerobot/README.md` — 公式コマンドの SO-101 対応手順
- `lerobot/.python-version` — Python 3.12 ピン留め
- ルート `README.md` に `lerobot/` の記述追加

作らないもの:

- ROS 2 とのブリッジ
- LeRobot の vendoring

追加するもの (設計時の想定から変更):

- `flake.nix` に `ffmpeg-headless` と `LD_LIBRARY_PATH` を追加 (torchcodec の FFmpeg 共有ライブラリ依存)

## 設計

### ディレクトリ構成

```
lerobot/
├── pyproject.toml    # uv プロジェクト (lerobot[core_scripts,training,feetech])
├── .python-version   # 3.12
└── README.md         # SO-101 leader/follower 同期手順 + ACT
```

### pyproject.toml

```toml
[project]
name = "so-arm-lerobot"
version = "0.1.0"
description = "LeRobot CLI environment for SO-101 leader/follower sync and ACT"
requires-python = ">=3.12"
dependencies = [
    "lerobot[core_scripts,training,feetech]",
]

[tool.uv]
package = false
```

### README.md の内容

1. 概要 — このディレクトリが何をするか
2. セットアップ — `uv sync`
3. ハードウェア設定
   - `lerobot-find-port` で leader/follower の USB ポート特定
   - `lerobot-setup-motors` で follower の ID/波特率設定
   - `lerobot-setup-motors` で leader の ID/波特率設定
4. キャリブレーション
   - `lerobot-calibrate --robot.type=so101_follower`
   - `lerobot-calibrate --teleop.type=so101_leader`
5. テレオペレーション (leader/follower 同期)
   - `lerobot-teleoperate` で follower を leader で操作
6. データ収録
   - `lerobot-record` で dataset を収録
   - `lerobot-dataset-viz` で可視化
7. ACT 学習
   - `lerobot-train --policy.type=act`
8. 推論
   - `lerobot-rollout` で学習済み policy を実行

### ルート README.md 変更

既存の `構成` セクションに以下を追加:

```
- `lerobot/`: LeRobot CLI による SO-101 の leader/follower 同期・データ収録・ACT 学習環境。詳細は [`lerobot/README.md`](lerobot/README.md)
```

## 判断根拠

- `lerobot/` をトップレベルに置く: README 方針の「責務ごとにトップレベルを分ける」に従う
- vendoring しない: 公式 CLI を「そのまま使う」のが要件
- Nix shell を追加しない: 今回は不要。必要になれば別途追加可能
- `pyproject.toml` で `lerobot` を依存に入れる: `uv` での再現性を確保しつつ、公式パッケージを使う

## 検証方法

- `lerobot/` で `uv sync` が成功すること
- `uv run lerobot-info` が動くこと
- README の手順が公式ドキュメントと整合していること
