#!/usr/bin/env bash
# SO-101 ROS 2 コンテナを起動する. 引数はコンテナ内で実行するコマンド (省略時は bash).
#
# 例:
#   ./podman/run.sh                       # 対話シェル
#   ./podman/run.sh ros2 launch lerobot_description so101_display.launch.py
#
# 環境変数:
#   USB_PORT  実機シリアルポート (default: /dev/ttyACM0)
#   IMAGE     イメージ名 (default: so101-ros2:jazzy)
#   REBUILD   "1" にするとイメージを強制リビルド
set -euo pipefail

IMAGE="${IMAGE:-so101-ros2:jazzy}"
USB_PORT="${USB_PORT:-/dev/ttyACM0}"

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${REBUILD:-}" == "1" ]] || ! podman image exists "$IMAGE"; then
  echo ">>> building image $IMAGE ..."
  podman build --format docker -t "$IMAGE" -f "$WS_ROOT/podman/Containerfile" "$WS_ROOT"
fi

run_args=(--rm -it --network host)

# --- USB デバイスの検出 ---
# macOS: デバイスは podman machine (VM) 内にあるため、SSH で確認する.
# Linux: ホストに直接存在するかチェック.
device_found=false
if [[ "$(uname)" == "Darwin" ]]; then
  machine_name=$(podman machine ls --format '{{.Name}}' --noheading 2>/dev/null | head -1)
  if [[ -n "$machine_name" ]] && \
     podman machine ssh "$machine_name" -- test -e "$USB_PORT" 2>/dev/null; then
    device_found=true
  fi
else
  if [[ -e "$USB_PORT" ]]; then
    device_found=true
  fi
fi

if $device_found; then
  run_args+=(--device "$USB_PORT")
  echo ">>> device: $USB_PORT"
else
  echo ">>> note: $USB_PORT が見つかりません."
  if [[ "$(uname)" == "Darwin" ]]; then
    echo "   macOS では ./podman/setup.sh --usb vendor=XXXX,product=XXXX で USB パススルーを設定してください."
  fi
fi

# rviz/MoveIt GUI 用 X11 forwarding (Linux ホスト前提).
if [[ -n "${DISPLAY:-}" ]]; then
  run_args+=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

# src をマウントして編集を即反映 (再ビルドは colcon build).
run_args+=(-v "$WS_ROOT/src:/ros2_ws/src:Z")

exec podman run "${run_args[@]}" "$IMAGE" "$@"
