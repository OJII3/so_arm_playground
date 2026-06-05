# so_arm_playground

SO-101 / SO-ARM 系の実験をまとめるモノレポです。現時点では Quest 3 + Godot 4 で SO-101 follower arm を操作する `VRTeleop` を最初のサブプロジェクトとして置いています。今後 ROS、ACT、VLA などの検証コードも同じリポジトリに追加する前提です。

## 構成

- `VRTeleop/`: Godot 4 OpenXR project と SO-101 teleoperation bridge
- `flake.nix`: モノレポ共通の開発 shell と checks
- `.python-version`: ローカル Python バージョンの目安

今後追加する実験は、責務ごとにトップレベルディレクトリを分けます。例: `ros/`, `act/`, `vla/`, `datasets/`, `tools/`。

## 開発環境

```bash
nix develop
pytest VRTeleop/tests
```

VRTeleop bridge の dry-run:

```bash
nix develop
cd VRTeleop
python -m vrteleop_bridge --config config/default.json --backend dry-run
```

MuJoCo や LeRobot は nixpkgs ではなく、プロジェクト venv に Python package として入れます。

```bash
nix develop
uv venv
uv pip install -e 'VRTeleop[sim]'
uv pip install -e 'VRTeleop[real]'
```

## Godot

Godot project は `VRTeleop/project.godot` です。

```bash
nix develop .#godot
godot4 --editor VRTeleop/project.godot
```

ただし Quest/OpenXR と GUI アプリ連携が絡むため、特に macOS では公式配布の Godot.app やユーザー環境に入れた Godot を使う方が扱いやすいです。
