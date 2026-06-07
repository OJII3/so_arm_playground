# so_arm_playground

SO-101 / SO-ARM 系の実験をまとめるモノレポです。今後 ROS、ACT、VLA などの検証コードも同じリポジトリに追加していきます。

## 構成

- `SoArmVR/`: SO-101 の VR teleoperation プロジェクト（Unity）
- `flake.nix`: モノレポ共通の開発 shell

実験は責務ごとにトップレベルディレクトリを分けます。例: `ros/`, `act/`, `vla/`, `datasets/`, `tools/`。

## 開発環境

```bash
nix develop
```
