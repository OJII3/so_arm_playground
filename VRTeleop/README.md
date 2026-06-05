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

Nix を使う場合:

```bash
cd ..
nix develop
cd VRTeleop
python -m vrteleop_bridge --config config/default.json --backend dry-run
pytest tests
```

MuJoCo Python API を使う場合は、nixpkgs 側の `mujoco` が broken になることがあるため、プロジェクト venv に入れます。

```bash
cd ..
nix develop
uv venv
uv pip install -e 'VRTeleop[sim]'
cd VRTeleop
python -m vrteleop_bridge --config config/default.json --backend mujoco --mjcf sim/so101_minimal.xml
```

Godot も Nix shell から試す場合:

```bash
cd ..
nix develop
godot4 --editor VRTeleop/project.godot
```

`nix develop` 内の `godot4` は Nix 版 Godot を OpenGL compatibility renderer で開く wrapper です。macOS では Nix 版 Godot の Metal renderer が起動時にクラッシュすることがあるため、編集時はこちらを使います。

Quest/OpenXR と GUI アプリ連携が絡むため、実機 VR 実行では Godot だけは公式配布の Godot.app やユーザー環境に入れたものを使う方が扱いやすいです。

Meta XR Simulator を使う場合、Apple Silicon Mac の `nix develop` では OpenXR runtime が shell 内で自動設定されます。

```bash
cd ..
nix develop
echo "$XR_RUNTIME_JSON"
godot4 --editor VRTeleop/project.godot
```

Mixed Reality 用の Synthetic Environment Server は、必要な部屋を 1 つだけ起動します。

```bash
meta-xr-sim-living-room
meta-xr-sim-game-room
meta-xr-sim-bedroom
```

venv を使う場合:

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
