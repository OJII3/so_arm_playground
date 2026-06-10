#!/usr/bin/env bash
# SO-101 ROS 2 コンテナを起動する. 引数はコンテナ内で実行するコマンド (省略時は bash).
#
# 例:
#   ./podman/run.sh                       # 対話シェル
#   ./podman/run.sh ros2 launch lerobot_description so101_display.launch.py
#
# 環境変数:
#   USB_PORT      コンテナ内のシリアルポートパス (default: /dev/ttyACM0)
#   IMAGE         イメージ名 (default: so101-ros2:jazzy)
#   REBUILD       "1" にするとイメージを強制リビルド
#   BRIDGE_PORT   socat ブリッジのポート番号 (default: 5555, macOS のみ)
set -euo pipefail

IMAGE="${IMAGE:-so101-ros2:jazzy}"
USB_PORT="${USB_PORT:-/dev/ttyACM0}"
BRIDGE_PORT="${BRIDGE_PORT:-5555}"

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${REBUILD:-}" == "1" ]] || ! podman image exists "$IMAGE"; then
  echo ">>> building image $IMAGE ..."
  podman build --format docker -t "$IMAGE" -f "$WS_ROOT/podman/Containerfile" "$WS_ROOT"
fi

run_args=(--rm -it --network host)

# --- USB デバイスの接続 ---
if [[ "$(uname)" == "Darwin" ]]; then
  if nc -z localhost "$BRIDGE_PORT" 2>/dev/null; then
    echo ">>> serial bridge detected (tcp://localhost:$BRIDGE_PORT -> $USB_PORT)"
    run_args+=(
      -e SERIAL_BRIDGE_PORT="$BRIDGE_PORT"
      -e USB_PORT="$USB_PORT"
    )
  else
    echo ">>> note: serial bridge が検出されません."
    echo "   実機制御には別ターミナルで先に実行してください:"
    echo "   ./podman/serial-bridge.sh"
  fi
else
  if [[ -e "$USB_PORT" ]]; then
    run_args+=(--device "$USB_PORT")
    echo ">>> device: $USB_PORT"
  else
    echo ">>> note: $USB_PORT が見つかりません."
  fi
fi

# rviz/MoveIt GUI 用 X11 forwarding (Linux ホスト前提).
if [[ -n "${DISPLAY:-}" ]]; then
  run_args+=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

# src をマウントして編集を即反映 (再ビルドは colcon build).
run_args+=(-v "$WS_ROOT/src:/ros2_ws/src:Z")

# macOS socat ブリッジ: コンテナ起動時に仮想シリアルポートを維持するデーモンを起動.
if [[ "$(uname)" == "Darwin" ]] && nc -z localhost "$BRIDGE_PORT" 2>/dev/null; then
  # socat を自動再起動するラッパー (PTY が消えないようにする)
  read -r -d '' bridge_script <<'INNER' || true
#!/bin/bash
USB_PORT="${USB_PORT:-/dev/ttyACM0}"
BRIDGE_PORT="${SERIAL_BRIDGE_PORT:-5555}"
while true; do
  socat PTY,link="$USB_PORT",raw,echo=0 TCP:host.containers.internal:"$BRIDGE_PORT" 2>/dev/null
  sleep 0.5
done
INNER

  if [[ $# -eq 0 ]]; then
    exec podman run "${run_args[@]}" "$IMAGE" \
      bash -c "bash -c '$bridge_script' &
               sleep 1
               echo '>>> virtual serial: $USB_PORT ready'
               exec bash"
  else
    exec podman run "${run_args[@]}" "$IMAGE" \
      bash -c "bash -c '$bridge_script' &
               sleep 1
               echo '>>> virtual serial: $USB_PORT ready'
               $*"
  fi
else
  exec podman run "${run_args[@]}" "$IMAGE" "$@"
fi
