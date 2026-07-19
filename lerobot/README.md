# lerobot — LeRobot CLI 環境 (SO-101)

SO-101 の leader arm と follower arm を公式 LeRobot CLI で同期し、
ACT ポリシーの収録・学習・推論を行うための環境。

## セットアップ

```bash
nix develop        # FFmpeg 等のシステム依存を提供
cd lerobot
uv sync
```

### CUDA の確認

Windows/Linux では `uv sync` が CUDA 13.0 版 PyTorch を導入する。学習前に GPU が認識されていることを確認する:

```bash
uv run python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

この環境での期待値は `2.11.0+cu130`、`True`、`NVIDIA GeForce RTX 3060`。

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
    --robot.cameras="{front: {type: opencv, index_or_path: /dev/video3, width: 640, height: 480, fps: 30}}" \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader_arm \
    --display_data=false \
    --play_sounds=false \
    --dataset.repo_id=${HF_USER}/so101_dataset \
    --dataset.num_episodes=50 \
    --dataset.single_task="pick up the block"
```

新規収録では、LeRobot が `dataset.repo_id` の末尾に収録開始日時を付加する。
収録ログまたは Hub で実際の ID を確認し、以降のコマンドで使用する:

```bash
DATASET_REPO_ID=${HF_USER}/so101_dataset_20260718_171048
```

新しく収録した場合は、`DATASET_REPO_ID` をその収録で生成された ID に更新する。

## 可視化

```bash
uv run lerobot-dataset-viz \
    --repo-id=${DATASET_REPO_ID} \
    --episode-index=0
```

## ACT 学習

```bash
uv run lerobot-train \
    --policy.type=act \
    --policy.repo_id=${HF_USER}/act_so101 \
    --dataset.repo_id=${DATASET_REPO_ID} \
    --dataset.video_backend=pyav \
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
