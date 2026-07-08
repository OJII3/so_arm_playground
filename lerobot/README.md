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

## Hugging Face Hub ログイン

データ収録・学習で Hub を使う場合は:

```bash
uv run hf auth login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
HF_USER=$(uv run hf auth whoami --quiet)
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
    --repo-id=${HF_USER}/so101_dataset \
    --episode-index=0
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
