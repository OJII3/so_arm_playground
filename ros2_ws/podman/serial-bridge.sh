#!/usr/bin/env bash
# macOS ホスト上でシリアルポートを TCP に公開する.
# コンテナ側は run.sh が自動で仮想シリアルポートを作成する.
#
# 使い方:
#   ./podman/serial-bridge.sh                          # デフォルト設定
#   ./podman/serial-bridge.sh /dev/tty.usbmodem...     # ポート指定
#   USB_PORT=/dev/tty.usbmodem... BAUD=1000000 ./podman/serial-bridge.sh
#
# 停止: Ctrl+C
set -euo pipefail

USB_PORT="${1:-${USB_PORT:-}}"
BAUD="${BAUD:-1000000}"
BRIDGE_PORT="${BRIDGE_PORT:-5555}"

if [[ -z "$USB_PORT" ]]; then
  candidates=(/dev/tty.usbmodem* /dev/tty.usbserial*)
  for c in "${candidates[@]}"; do
    if [[ -e "$c" ]]; then
      USB_PORT="$c"
      break
    fi
  done
fi

if [[ -z "$USB_PORT" || ! -e "$USB_PORT" ]]; then
  echo "ERROR: シリアルデバイスが見つかりません." >&2
  echo "  接続を確認するか、引数でパスを指定してください:" >&2
  echo "  $0 /dev/tty.usbmodemXXXX" >&2
  exit 1
fi

echo ">>> serial bridge: $USB_PORT (${BAUD}bps) -> tcp://0.0.0.0:${BRIDGE_PORT}"
echo ">>> コンテナから接続するには: ./podman/run.sh"
echo ">>> 停止: Ctrl+C"

exec socat \
  TCP-LISTEN:"$BRIDGE_PORT",reuseaddr,fork \
  FILE:"$USB_PORT",ispeed="$BAUD",ospeed="$BAUD",raw,echo=0
