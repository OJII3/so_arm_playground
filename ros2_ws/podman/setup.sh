#!/usr/bin/env bash
# podman machine のセットアップスクリプト.
#
# 使い方:
#   ./podman/setup.sh          # デフォルト設定で初期化・起動
#
# 環境変数:
#   MACHINE_NAME  machine 名 (default: so101)
#   NET_IFACE     vmnet-bridged に使う物理 IF (default: en0 = Wi-Fi)
#                 LAN 内の ROS 2 通信に必要. 空文字でデフォルト NAT に戻す.
#
# podman machine が未作成なら初期化し、停止中なら起動する.
set -euo pipefail

MACHINE_NAME="${MACHINE_NAME:-so101}"
CPUS="${CPUS:-4}"
MEMORY="${MEMORY:-4096}"
DISK_SIZE="${DISK_SIZE:-40}"
NET_IFACE="${NET_IFACE:-en0}"

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

  net_args=()
  if [[ -n "$NET_IFACE" ]]; then
    echo ">>> vmnet-bridged: interface=$NET_IFACE (LAN 直接接続)"
    net_args=(--network "vmnet-bridged:interface=$NET_IFACE")
  fi

  podman machine init \
    --cpus "$CPUS" \
    --memory "$MEMORY" \
    --disk-size "$DISK_SIZE" \
    --rootful \
    --now \
    "${net_args[@]}" \
    "$MACHINE_NAME"
}

verify() {
  echo ""
  echo "=== セットアップ完了 ==="
  podman machine ls
  echo ""

  local vm_ip
  vm_ip=$(podman machine ssh "$MACHINE_NAME" -- ip -4 addr show 2>/dev/null \
    | awk '/inet / {split($2,a,"/"); print a[1]}' | grep -v '127.0.0.1' | head -1) || true
  if [[ -n "$vm_ip" ]]; then
    echo "VM IP: $vm_ip"
  fi

  echo ""
  echo "次のステップ:"
  echo "  ./podman/run.sh                      # コンテナ起動 (対話シェル)"
  echo ""
  echo "実機制御 (macOS):"
  echo "  ターミナル1: ./podman/serial-bridge.sh   # USB -> TCP ブリッジ"
  echo "  ターミナル2: ./podman/run.sh             # コンテナ (自動で仮想シリアル作成)"
  echo ""
  echo "machine の再作成 (ネットワーク設定変更時):"
  echo "  podman machine rm $MACHINE_NAME && ./podman/setup.sh"
}

check_podman
setup_machine
verify
