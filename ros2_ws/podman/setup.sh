#!/usr/bin/env bash
# podman machine のセットアップスクリプト (macOS 向け).
#
# 使い方:
#   ./podman/setup.sh                              # デフォルト設定で初期化
#   ./podman/setup.sh --usb vendor=1a86,product=7523  # USB パススルー付き
#
# USB パススルーは QEMU プロバイダーでのみ動作する.
# Feetech サーボでよく使われる USB-Serial アダプタ:
#   CH340/CH341:  vendor=1a86, product=7523
#   CP2102:       vendor=10c4, product=ea60
#   FTDI FT232:   vendor=0403, product=6001
#
# 接続中のデバイスの vendor/product ID を確認するには:
#   macOS:  system_profiler SPUSBDataType
#   Linux:  lsusb
set -euo pipefail

MACHINE_NAME="${MACHINE_NAME:-so101}"
CPUS="${CPUS:-4}"
MEMORY="${MEMORY:-4096}"
DISK_SIZE="${DISK_SIZE:-40}"

USB_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --usb)
      USB_ARGS+=(--usb "$2")
      shift 2
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

check_podman() {
  if ! command -v podman &>/dev/null; then
    echo "ERROR: podman がインストールされていません."
    echo "  macOS:  brew install podman"
    echo "  Linux:  https://podman.io/docs/installation"
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

    if [[ ${#USB_ARGS[@]} -gt 0 ]]; then
      echo ">>> USB デバイスを設定します (machine の再起動が必要です)..."
      podman machine stop "$MACHINE_NAME" 2>/dev/null || true
      podman machine set "${USB_ARGS[@]}" "$MACHINE_NAME"
      podman machine start "$MACHINE_NAME"
    fi

    echo ">>> machine '$MACHINE_NAME' is running."
    return
  fi

  echo ">>> machine '$MACHINE_NAME' を初期化します..."

  local init_args=(
    --cpus "$CPUS"
    --memory "$MEMORY"
    --disk-size "$DISK_SIZE"
    --rootful
  )

  if [[ ${#USB_ARGS[@]} -gt 0 ]]; then
    init_args+=(--provider qemu)
    init_args+=("${USB_ARGS[@]}")
    echo ">>> USB パススルーが指定されたため QEMU プロバイダーを使用します."
  fi

  podman machine init "${init_args[@]}" "$MACHINE_NAME"

  echo ">>> machine を起動します..."
  podman machine start "$MACHINE_NAME"
}

verify() {
  echo ""
  echo "=== セットアップ完了 ==="
  podman machine ls
  echo ""

  if [[ ${#USB_ARGS[@]} -gt 0 ]]; then
    echo ">>> USB デバイスの確認 (machine 内):"
    podman machine ssh "$MACHINE_NAME" -- ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || \
      echo "  (シリアルデバイスが見つかりません. デバイスが接続されているか確認してください)"
    echo ""
  fi

  echo "次のステップ:"
  echo "  ./podman/run.sh                  # コンテナ起動 (対話シェル)"
  echo "  ./podman/run.sh ros2 launch ...  # コマンド指定"
}

check_podman
setup_machine
verify
