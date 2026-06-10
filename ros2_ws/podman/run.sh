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
#   ZENOH_PEER    Zenoh 接続先 (例: tcp/192.168.1.10:7447). LAN 上の ROS 2 と通信する場合に指定.
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

# DDS をコンテナ内ループバックに制限 (LAN 通信は Zenoh ブリッジ経由).
run_args+=(-e "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")

# Zenoh ブリッジ設定.
if [[ -n "${ZENOH_PEER:-}" ]]; then
  run_args+=(-e "ZENOH_PEER=$ZENOH_PEER")
fi

# src をマウントして編集を即反映 (再ビルドは colcon build).
run_args+=(-v "$WS_ROOT/src:/ros2_ws/src:Z")

# --- コンテナ内で実行する起動スクリプトを組み立て ---
startup_cmds=()

# macOS socat ブリッジ: 仮想シリアルポートを維持するデーモン.
if [[ "$(uname)" == "Darwin" ]] && nc -z localhost "$BRIDGE_PORT" 2>/dev/null; then
  startup_cmds+=('
    (while true; do
      socat PTY,link="$USB_PORT",raw,echo=0 \
        TCP:host.containers.internal:"$SERIAL_BRIDGE_PORT" 2>/dev/null
      sleep 0.5
    done) &
    sleep 1
    echo ">>> virtual serial: $USB_PORT ready"
  ')
fi

# Zenoh ブリッジ: ZENOH_PEER が設定されていれば自動起動.
if [[ -n "${ZENOH_PEER:-}" ]]; then
  startup_cmds+=('
    ip link set lo multicast on 2>/dev/null || true
    zenoh-bridge-ros2dds -m client -e "$ZENOH_PEER" &
    sleep 1
    echo ">>> zenoh bridge: connected to $ZENOH_PEER"
  ')
fi

if [[ ${#startup_cmds[@]} -gt 0 ]]; then
  startup=$(printf '%s\n' "${startup_cmds[@]}")
  if [[ $# -eq 0 ]]; then
    exec podman run "${run_args[@]}" "$IMAGE" \
      bash -c "$startup"$'\nexec bash'
  else
    exec podman run "${run_args[@]}" "$IMAGE" \
      bash -c "$startup"$'\n'"$*"
  fi
else
  exec podman run "${run_args[@]}" "$IMAGE" "$@"
fi
