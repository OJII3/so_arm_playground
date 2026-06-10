#!/usr/bin/env bash
# macOS ホスト上でシリアルポートを TCP に公開する.
# コンテナ側は run.sh が自動で仮想シリアルポートを作成する.
#
# 使い方:
#   ./podman/serial-bridge.sh                          # デフォルト設定
#   ./podman/serial-bridge.sh /dev/cu.usbmodemXXX      # ポート指定
#   USB_PORT=/dev/cu.usbmodemXXX BAUD=1000000 ./podman/serial-bridge.sh
#
# 停止: Ctrl+C
set -euo pipefail

USB_PORT="${1:-${USB_PORT:-}}"
BAUD="${BAUD:-1000000}"
BRIDGE_PORT="${BRIDGE_PORT:-5555}"

if [[ -z "$USB_PORT" ]]; then
  candidates=(/dev/cu.usbmodem* /dev/cu.usbserial*)
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
  echo "  $0 /dev/cu.usbmodemXXXX" >&2
  exit 1
fi

echo ">>> serial bridge: $USB_PORT (${BAUD}bps) -> tcp://0.0.0.0:${BRIDGE_PORT}"
echo ">>> コンテナから接続するには: ./podman/run.sh"
echo ">>> 停止: Ctrl+C"

exec python3 -u - "$USB_PORT" "$BAUD" "$BRIDGE_PORT" <<'PYEOF'
import sys, os, socket, select, threading, signal, fcntl, ctypes, termios

serial_path, baud, port = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

def open_serial(path, baud_rate):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    # cfmakeraw equivalent
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                   termios.ISTRIP | termios.INLCR | termios.IGNCR |
                   termios.ICRNL | termios.IXON)
    attrs[1] &= ~termios.OPOST
    attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                   termios.ISIG | termios.IEXTEN)
    attrs[2] &= ~(termios.CSIZE | termios.PARENB)
    attrs[2] |= termios.CS8
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # macOS: IOSSIOSPEED ioctl でカスタムボーレートを設定
    IOSSIOSPEED = 0x80045402
    speed = ctypes.c_uint(baud_rate)
    fcntl.ioctl(fd, IOSSIOSPEED, speed)
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.set_blocking(fd, True)
    return fd

def bridge(conn, serial_fd):
    try:
        while True:
            readable, _, _ = select.select([conn, serial_fd], [], [], 1.0)
            for r in readable:
                if r is conn:
                    data = conn.recv(4096)
                    if not data:
                        return
                    os.write(serial_fd, data)
                elif r == serial_fd:
                    try:
                        data = os.read(serial_fd, 4096)
                        if not data:
                            return
                        conn.sendall(data)
                    except BlockingIOError:
                        pass
    except (ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        conn.close()

serial_fd = open_serial(serial_path, baud)
print(f"serial port opened: {serial_path} @ {baud} baud")

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", port))
srv.listen(1)
print(f"listening on tcp://0.0.0.0:{port}")

def shutdown(sig, frame):
    os.close(serial_fd)
    srv.close()
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

while True:
    conn, addr = srv.accept()
    print(f"connection from {addr}")
    t = threading.Thread(target=bridge, args=(conn, serial_fd), daemon=True)
    t.start()
PYEOF
