# so_arm_playground

SO-101 / SO-ARM 系の実験をまとめるモノレポです。今後 ROS、ACT、VLA などの検証コードも同じリポジトリに追加していきます。

## 構成

- `ros2_ws/`: SO-101 の ROS 2 (Jazzy) / ros2_control / MoveIt 2 制御ワークスペース。詳細は [`ros2_ws/README.md`](ros2_ws/README.md)
- `flake.nix`: モノレポ共通の開発 shell

実験は責務ごとにトップレベルディレクトリを分けます。例: `ros2_ws/`, `act/`, `vla/`, `datasets/`, `tools/`。

## 開発環境

```bash
nix develop
```

ROS 2 開発 shell（Linux のみ）:

```bash
nix develop .#ros
```
