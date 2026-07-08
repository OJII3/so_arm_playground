# lerobot/ ディレクトリ追加 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SO-101 の leader arm と follower arm を公式 LeRobot CLI で同期できる環境を `lerobot/` に作り、ACT がそのまま使えるようにする。

**Architecture:** `lerobot/` に `uv` プロジェクトを作り、`lerobot[core_scripts,training,feetech]` を依存に入れる。README に公式コマンドを SO-101 用に整理する。ROS 2 とは独立。

**Tech Stack:** Python 3.12, uv, LeRobot (PyPI)

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `lerobot/pyproject.toml` | Create | uv プロジェクト定義 |
| `lerobot/.python-version` | Create | Python 3.12 ピン留め |
| `lerobot/README.md` | Create | SO-101 leader/follower 同期手順 |
| `README.md` | Modify | 構成セクションに lerobot/ を追加 |

---

### Task 1: lerobot/ プロジェクト基盤

**Files:**
- Create: `lerobot/pyproject.toml`
- Create: `lerobot/.python-version`

- [ ] **Step 1: lerobot/ ディレクトリを作成**

```bash
mkdir -p lerobot
```

- [ ] **Step 2: .python-version を作成**

```bash
echo "3.12" > lerobot/.python-version
```

- [ ] **Step 3: pyproject.toml を作成**

`lerobot/pyproject.toml` を作成:

```toml
[project]
name = "so-arm-lerobot"
version = "0.1.0"
description = "LeRobot CLI environment for SO-101 leader/follower sync and ACT"
requires-python = ">=3.12"
dependencies = [
    "lerobot[core_scripts,training,feetech]",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

- [ ] **Step 4: uv sync で依存をインストール**

```bash
cd lerobot && uv sync
```

- [ ] **Step 5: lerobot-info で動作確認**

```bash
cd lerobot && uv run lerobot-info
```

Expected: LeRobot のバージョン情報が表示される

- [ ] **Step 6: コミット**

```bash
git add lerobot/pyproject.toml lerobot/.python-version lerobot/uv.lock
git commit -m "feat(lerobot): add uv project with LeRobot CLI dependencies"
```

---

### Task 2: lerobot/README.md 作成

**Files:**
- Create: `lerobot/README.md`

- [ ] **Step 1: README.md を作成**

`lerobot/README.md` を作成:

```markdown
# lerobot — LeRobot CLI 環境 (SO-101)

SO-101 の leader arm と follower arm を公式 LeRobot CLI で同期し、
ACT ポリシーの収録・学習・推論を行うための環境。

## セットアップ

```bash
cd lerobot
uv sync
```

## ハードウェア設定

### USB ポートの特定

```bash
uv run lerobot-find-port
```

leader / follower それぞれのバスを接続して特定する。

### モーター ID・波特率の設定

```bash
# follower 側
uv run lerobot-setup-motors \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0

# leader 側
uv run lerobot-setup-motors \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1
```

## キャリブレーション

```bash
# follower
uv run lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower_arm

# leader
uv run lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader_arm
```

## テレオペレーション (leader/follower 同期)

```bash
uv run lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader_arm
```

## データ収録

```bash
uv run lerobot-record \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader_arm \
    --dataset.repo_id=${HF_USER}/so101_dataset \
    --dataset.num_episodes=50 \
    --dataset.single_task="pick up the block"
```

## 可視化

```bash
uv run lerobot-dataset-viz \
    --repo_id=${HF_USER}/so101_dataset
```

## ACT 学習

```bash
uv run lerobot-train \
    --policy.type=act \
    --dataset.repo_id=${HF_USER}/so101_dataset \
    --output_dir=outputs/train/act_so101 \
    --policy.device=cuda
```

## 推論

```bash
uv run lerobot-rollout \
    --policy.path=outputs/train/act_so101/checkpoints/last/pretrained_model \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower_arm \
    --task="pick up the block"
```
```

- [ ] **Step 2: コミット**

```bash
git add lerobot/README.md
git commit -m "docs(lerobot): add SO-101 leader/follower sync and ACT workflow"
```

---

### Task 3: ルート README.md 更新

**Files:**
- Modify: `README.md:7-9`

- [ ] **Step 1: 構成セクションに lerobot/ を追加**

既存の `ros2_ws/` の後に追加:

```markdown
- `lerobot/`: LeRobot CLI による SO-101 の leader/follower 同期・データ収録・ACT 学習環境。詳細は [`lerobot/README.md`](lerobot/README.md)
```

- [ ] **Step 2: コミット**

```bash
git add README.md
git commit -m "docs: add lerobot/ to repo structure in README"
```
