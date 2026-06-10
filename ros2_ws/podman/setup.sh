#!/usr/bin/env bash
# podman machine のセットアップスクリプト.
#
# 使い方:
#   ./podman/setup.sh          # デフォルト設定で初期化・起動
#
# podman machine が未作成なら初期化し、停止中なら起動する.
set -euo pipefail

MACHINE_NAME="${MACHINE_NAME:-so101}"
CPUS="${CPUS:-4}"
MEMORY="${MEMORY:-4096}"
DISK_SIZE="${DISK_SIZE:-40}"

check_podman() {
  if ! command -v podman &>/dev/null; then
    echo "ERROR: podman がインストールされていません."
    echo "  nix develop で devShell に入ってから実行してください."
    exit 1
  fi
  echo "podman $(podman --version | awk '{print $NF}')"
}

setup_machine() {
  if podman machine inspect "$MACHINE_NAME" &>/dev/null; then
    echo ">>> machine '$MACHINE_NAME' は既に存在します."

    local state
    state=$(podman machine inspect "$MACHINE_NAME" --format '{{.State}}')
    if [[ "$state" != "running" ]]; then
      echo ">>> machine を起動します..."
      podman machine start "$MACHINE_NAME"
    fi

    echo ">>> machine '$MACHINE_NAME' is running."
    return
  fi

  echo ">>> machine '$MACHINE_NAME' を初期化します..."
  podman machine init \
    --cpus "$CPUS" \
    --memory "$MEMORY" \
    --disk-size "$DISK_SIZE" \
    --rootful \
    --now \
    "$MACHINE_NAME"
}

verify() {
  echo ""
  echo "=== セットアップ完了 ==="
  podman machine ls
  echo ""
  echo "次のステップ:"
  echo "  ./podman/run.sh                      # コンテナ起動 (対話シェル)"
  echo ""
  echo "実機制御 (macOS):"
  echo "  ターミナル1: ./podman/serial-bridge.sh   # USB -> TCP ブリッジ"
  echo "  ターミナル2: ./podman/run.sh             # コンテナ (自動で仮想シリアル作成)"
}

check_podman
setup_machine
verify
