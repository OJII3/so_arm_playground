# ros2_ws — SO-101 ROS 2 制御ワークスペース

SO-101 を ROS 2 (Jazzy) / ros2_control / MoveIt 2 で制御するための colcon ワークスペース。
シミュレーション (rviz / Gazebo) と、Feetech サーボによる実機制御の両方に対応する。

## 由来 (vendored via git subtree)

| パッケージ | 上流 | ブランチ |
| --- | --- | --- |
| `lerobot_description` / `lerobot_controller` / `lerobot_moveit` | [Ar-Ray-code/lerobot_ws](https://github.com/Ar-Ray-code/lerobot_ws) | `sts_servo` |
| `feetech_ros2_driver` | [Ar-Ray-code/feetech_ros2_driver](https://github.com/Ar-Ray-code/feetech_ros2_driver) | `auto_calib` |

`git subtree --squash` で取り込んでおり、出典コミットはリポジトリ履歴に残る。上流更新は
`git subtree pull --prefix=ros2_ws <upstream> <branch> --squash` で取り込む。

参考: [SO-101 を ROS 2 で実機制御する記事](https://ar-ray.hatenablog.com/entry/2025/10/31/203953)

### 上流からの変更点

- `feetech_ros2_driver/package.xml`: `rclcpp_lifecycle` / `lifecycle_msgs` / `sensor_msgs` /
  `std_srvs` を追記 (CMake/ソースで使用しているのに未宣言だった)。
- `lerobot_{moveit,controller}/package.xml`: `moveit_config_utils` → `moveit_configs_utils` (typo 修正)。

## ディレクトリ

```
ros2_ws/
├── src/        … vendored ROS 2 パッケージ
├── nix/        … ros2nix 生成の Nix 式 + LibSerial derivation
└── podman/     … コンテナによる退避路 (macOS など)
```

`build/` `install/` `log/` はリポジトリルートの `.gitignore` で除外。

## 起動方法 A: Nix (Linux 推奨)

[nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay) を使う。**Linux 専用**
(`devShells.ros` は Linux でのみ定義)。Nix 式は `package.xml` から
[ros2nix](https://github.com/wentasah/ros2nix) で生成している (`nix/` 配下)。

```bash
nix develop .#ros            # リポジトリルートで
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

direnv を使う場合は `cd ros2_ws` するだけで ROS シェルに自動で切り替わる
(`ros2_ws/.envrc` がルート flake の `#ros` を読み込む)。`colcon build` 済みなら
`install/setup.bash` も自動 source される。初回だけ `direnv allow` が必要。
macOS では native に動かないため podman を案内する (上記 方法 B)。

`nix/` を再生成する場合 (`package.xml` を変更したとき):

```bash
cd ros2_ws
nix run github:wentasah/ros2nix -- \
  --distro jazzy --output-dir nix --output-as-nix-pkg-name --nixfmt \
  $(find src -name package.xml)
```

> LibSerial (crayzeewulf/LibSerial) は nixpkgs に無いため `nix/libserial.nix` で自作し、
> flake から `extraPkgs.libserial-dev` として注入している。

## 起動方法 B: podman (macOS / 退避路)

`podman` はシステムに用意しておくこと（nix では入れていない）。macOS は初回に
`podman machine init && podman machine start` が必要。

```bash
./podman/run.sh                  # イメージを build して対話シェル
./podman/run.sh ros2 launch lerobot_description so101_display.launch.py
```

- 実機シリアルは `USB_PORT`(default `/dev/ttyACM0`)が存在すれば `--device` で渡す。
- rviz/MoveIt GUI は X11 forwarding (`DISPLAY` と `/tmp/.X11-unix`)。事前に `xhost +local:` が必要な場合あり。
- **macOS の制約**: `podman machine` (Linux VM) はホスト USB を直接渡せないため、Mac 単体では
  実機制御不可。実機は Linux ホスト (PC / Raspberry Pi) で実行すること。Mac はシミュレーション・開発まで。

## 代表的なコマンド

```bash
# シミュレーション表示 (rviz)
ros2 launch lerobot_description so101_display.launch.py
# Gazebo シミュレーション
ros2 launch lerobot_description so101_gazebo.launch.py

# フォロワー実機 + コントローラ (is_sim:=True でシミュレーション)
ros2 launch lerobot_controller so101_follower_controller.launch.py \
  is_sim:=False usb_port:=/dev/ttyACM0
# リーダー・フォロワー (teleoperation)
ros2 launch lerobot_controller so101_leader_follower.launch.py

# MoveIt 2
ros2 launch lerobot_moveit so101_moveit.launch.py is_sim:=False

# サーボ・キャリブレーション (結果を JSON 保存)
ros2 run feetech_ros2_driver feetech_calibration_node --ros-args \
  -p usb_port:=/dev/ttyACM0 -p save_path:=./calib.json
```

## 検証状況

- Nix: `nix develop .#ros` の評価 (`x86_64-linux` / `aarch64-linux`) まで確認済み。
- **未検証 (要 Linux 実機/VM)**: `colcon build` の実通過、rviz/Gazebo/MoveIt の起動、
  `/dev/ttyACM0` 経由の実機サーボ動作・キャリブレーション。macOS 開発機では実行不可のため、
  Linux ホストでの確認が必要。

---

ライセンス: 各 vendored パッケージのライセンスに従う
([lerobot_ws](https://github.com/Ar-Ray-code/lerobot_ws) は Apache-2.0、
[feetech_ros2_driver](https://github.com/Ar-Ray-code/feetech_ros2_driver) は BSD)。
URDF/メッシュは [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) に基づく。
