#!/usr/bin/env bash
set -euo pipefail

# RoboStack 環境を作成する。
# 前提: micromamba がインストール済み (nix develop .#robostack で利用可能)。

ENV_FILE="$(cd "$(dirname "$0")" && pwd)/environment.yaml"

if ! command -v micromamba &>/dev/null; then
  echo "Error: micromamba が見つかりません。'nix develop .#robostack' を先に実行してください。" >&2
  exit 1
fi

export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/.mamba}"
export MAMBA_EXE="$(command -v micromamba)"

echo "==> RoboStack: ros_jazzy 環境を作成/更新します..."
"$MAMBA_EXE" env create -n ros_jazzy -f "$ENV_FILE" 2>/dev/null \
  || "$MAMBA_EXE" env update -n ros_jazzy -f "$ENV_FILE"

echo ""
echo "==> 完了! 以下のコマンドで環境を有効化してください:"
echo "    eval \$(\"$MAMBA_EXE\" shell activate -n ros_jazzy --shell bash)"
echo ""
echo "    または nix develop .#robostack (自動 activation)"
