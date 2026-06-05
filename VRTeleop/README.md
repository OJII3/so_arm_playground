# VRTeleop

Godot 4 + OpenXR で Quest 3 の右コントローラー pose を読み、Python ブリッジで SO-101 follower arm の IK を解いて送信する VR teleoperation プロジェクトです。

## 主要ファイル

- `project.godot`: Godot プロジェクト
- `scenes/main.tscn`: XR scene
- `scripts/main.gd`: OpenXR 初期化
- `scripts/xr_controller_pose_sender.gd`: Quest controller pose の UDP 送信
- `vrteleop_bridge/`: IK と backend 実装
- `config/default.json`: IK、通信、安全制限の設定
- `docs/SPEC.md`: 仕様メモ

## Python ブリッジ

```bash
python -m venv .venv
. .venv/bin/activate
pip install -e .
python -m vrteleop_bridge --config config/default.json --backend dry-run
```

実機 SO-101 follower:

```bash
pip install "lerobot[feetech]"
python -m vrteleop_bridge \
  --config config/default.json \
  --backend real-lerobot \
  --port /dev/tty.usbmodemXXXX \
  --robot-id so101_follower
```

MuJoCo:

```bash
pip install mujoco
python -m vrteleop_bridge \
  --config config/default.json \
  --backend mujoco \
  --mjcf sim/so101_minimal.xml
```

`sim/so101_minimal.xml` は指令経路を検証するための最小モデルです。実機に近い干渉形状や慣性が必要な場合は、同じ actuator 名を持つ SO-101 MJCF に差し替えてください。

## Godot

Godot 4 で `project.godot` を開き、OpenXR plugin/runtime が有効な状態で実行してください。右 trigger を gripper 指令、右 grip を enable 入力として送ります。

安全のため、既定 backend は `dry-run` です。実機に送る前に `dry-run` と MuJoCo でワークスペース、座標方向、関節可動域を確認してください。
