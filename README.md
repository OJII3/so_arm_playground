# so_arm_playground

SO-101 / SO-ARM 系の実験をまとめるモノレポです。今後 ROS、ACT、VLA などの検証コードも同じリポジトリに追加していきます。

## 構成

- `VRTeleop/`: SO-101 の VR teleoperation プロジェクト（Unity で再構築予定）
- `flake.nix`: モノレポ共通の開発 shell
- `.python-version`: ローカル Python バージョンの目安

実験は責務ごとにトップレベルディレクトリを分けます。例: `ros/`, `act/`, `vla/`, `datasets/`, `tools/`。

## 開発環境

```bash
nix develop
```

## Meta XR Simulator

Apple Silicon Mac の `nix develop` では Meta XR Simulator v71.0.0 も入り、OpenXR runtime は shell 内で自動設定されます。システム全体の OpenXR runtime symlink は変更しません。

```bash
nix develop
echo "$XR_RUNTIME_JSON"
```

Mixed Reality 用の Synthetic Environment Server は、必要な部屋を 1 つだけ起動します。

```bash
meta-xr-sim-living-room
meta-xr-sim-game-room
meta-xr-sim-bedroom
```
