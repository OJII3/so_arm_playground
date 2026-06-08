#!/usr/bin/env bash
# SO-101 ROS 2 コンテナを起動する. 引数はコンテナ内で実行するコマンド (省略時は bash).
#
# 例:
#   ./podman/run.sh                       # 対話シェル
#   ./podman/run.sh ros2 launch lerobot_description so101_display.launch.py
#
# 環境変数:
#   USB_PORT  実機シリアルポート (default: /dev/ttyACM0). 存在すれば --device で渡す.
#   IMAGE     イメージ名 (default: so101-ros2:jazzy)
set -euo pipefail

IMAGE="${IMAGE:-so101-ros2:jazzy}"
USB_PORT="${USB_PORT:-/dev/ttyACM0}"

# ros2_ws ルート (このスクリプトの親ディレクトリ).
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! podman image exists "$IMAGE"; then
  echo ">>> building image $IMAGE ..."
  podman build -t "$IMAGE" -f "$WS_ROOT/podman/Containerfile" "$WS_ROOT"
fi

run_args=(--rm -it --network host)

# 実機シリアルポート (存在する場合のみ).
if [[ -e "$USB_PORT" ]]; then
  run_args+=(--device "$USB_PORT")
else
  echo ">>> note: $USB_PORT が見つかりません (sim のみ / Linux ホストで実行してください)"
fi

# rviz/MoveIt GUI 用 X11 forwarding (Linux ホスト前提). 必要なら事前に: xhost +local:
if [[ -n "${DISPLAY:-}" ]]; then
  run_args+=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

# src をマウントして編集を即反映 (再ビルドは colcon build).
run_args+=(-v "$WS_ROOT/src:/ros2_ws/src:Z")

exec podman run "${run_args[@]}" "$IMAGE" "$@"
