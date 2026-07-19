# Design: LeRobot ACT 学習の CUDA 対応

## 概要

`lerobot/` の `uv` 環境で Windows/Linux の PyTorch を CUDA 13.0 版へ固定し、RTX 3060 を使って ACT を学習できるようにする。動画デコードは、現在の Windows 環境で動作確認済みの PyAV を維持する。

## 背景

- ホストには NVIDIA GeForce RTX 3060 (12 GB) がある
- NVIDIA ドライバ 591.86 は CUDA 13.1 をサポートしている
- 現在の環境には `torch==2.11.0+cpu` が入り、`torch.cuda.is_available()` は `False` になる
- PyTorch 2.11 は Windows/Linux 向け CUDA 13.0 wheel を公式配布している
- `uv` は PyTorch 専用 index を `explicit = true` で定義し、対象パッケージを `tool.uv.sources` で固定する構成を推奨している
- TorchCodec は Windows で shared FFmpeg を必要とするため、データセットの動画デコードには PyAV を使う

## 目的

- `uv sync` だけで Windows/Linux に CUDA 13.0 版 PyTorch を導入する
- `uv.lock` に CUDA 版の解決結果を記録し、環境を再現可能にする
- ACT の実学習ステップが `policy.device=cuda` で完了することを確認する

## 対象外

- CUDA Toolkit のローカルインストール
- TorchCodec/NVDEC による GPU 動画デコード
- macOS 上の GPU 学習
- ACT のハイパーパラメータ調整
- 長時間の本番学習完了やモデル品質評価

## 設計

### 依存関係

`lerobot/pyproject.toml` に、LeRobot が現在使用している組み合わせを直接依存として追加する。

```toml
dependencies = [
    "lerobot[core_scripts,training,feetech]",
    "torch==2.11.0",
    "torchvision==0.26.0",
]
```

直接依存にすることで、使用する PyTorch 系パッケージと専用 index の対応を明示する。

### PyTorch CUDA index

Windows/Linux では CUDA 13.0 index を使用し、macOS では既定の PyPI にフォールバックする。

```toml
[tool.uv.sources]
torch = [
    { index = "pytorch-cu130", marker = "sys_platform == 'linux' or sys_platform == 'win32'" },
]
torchvision = [
    { index = "pytorch-cu130", marker = "sys_platform == 'linux' or sys_platform == 'win32'" },
]

[[tool.uv.index]]
name = "pytorch-cu130"
url = "https://download.pytorch.org/whl/cu130"
explicit = true
```

`explicit = true` により、LeRobot や一般ライブラリを PyTorch index から誤って解決しないようにする。

### 動画デコード

README の ACT 学習コマンドは `--dataset.video_backend=pyav` を維持する。CUDA 対応の対象は ACT policy の forward/backward と optimizer step であり、動画デコードは CPU 上の PyAV が担当する。

### セットアップと診断

README に次の確認コマンドを追加する。

```bash
uv sync
uv run python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

期待値は、PyTorch のバージョンが `2.11.0+cu130`、CUDA availability が `True`、デバイス名が `NVIDIA GeForce RTX 3060` になること。

## データフロー

1. `uv sync` が PyPI から LeRobot と一般依存を解決する
2. `torch` と `torchvision` だけを公式 `pytorch-cu130` index から解決する
3. PyAV がデータセット動画を CPU メモリ上のテンソルへデコードする
4. LeRobot の前処理がバッチを CUDA device へ転送する
5. ACT policy が RTX 3060 上で forward/backward と optimizer step を実行する

## エラー処理

- `torch.cuda.is_available()` が `False` の場合は学習を開始せず、PyTorch の版、`torch.version.cuda`、`nvidia-smi` を確認する
- `torch` が `+cpu` の場合は `uv.lock` または source marker が CUDA index を選んでいないため、依存解決を見直す
- CUDA out-of-memory が起きた場合は、依存設定を変えず `--batch_size` を下げる
- TorchCodec のロード失敗を避けるため、学習コマンドでは PyAV を明示する

## 検証方法

1. `uv sync` が成功する
2. `torch.__version__` が `2.11.0+cu130` になる
3. `torch.version.cuda` が `13.0` になる
4. `torch.cuda.is_available()` が `True` になる
5. `torch.cuda.get_device_name(0)` が `NVIDIA GeForce RTX 3060` を返す
6. CUDA tensor の演算と GPU メモリ割り当てが成功する
7. 既存データセットを使った ACT 学習を1ステップ実行し、設定ログが `policy.device='cuda'` のまま、`End of training` まで完了する
8. `git diff --check` が成功する

1ステップ検証では Hub への push と checkpoint 保存を無効にし、本番成果物を作らない。

## 変更対象

- `lerobot/pyproject.toml`
- `lerobot/uv.lock`
- `lerobot/README.md`

## 判断根拠

- 一時的な `uv pip install` ではなくプロジェクト設定へ index を記録し、`uv sync` の再現性を優先する
- optional extra は使わず、ユーザーが選択したとおり Windows/Linux の既定を CUDA にする
- ドライバは CUDA 13.0 wheel の実行要件を満たすため、CUDA Toolkit 自体は追加しない
- 動作確認済みの PyAV を維持し、モデル学習の GPU 化と動画デコード基盤の変更を分離する
