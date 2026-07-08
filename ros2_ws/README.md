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
├── nix/         … ros2nix 生成の Nix 式 + LibSerial derivation
└── robostack/   … RoboStack (conda) 環境定義 (macOS 推奨)
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
(`ros2_ws/.envrc` がルート flake の `#ros` (Linux) または `#robostack` (macOS) を読み込む)。
`colcon build` 済みなら `install/setup.bash` も自動 source される。初回だけ `direnv allow` が必要。

`nix/` を再生成する場合 (`package.xml` を変更したとき):

```bash
cd ros2_ws
nix run github:wentasah/ros2nix -- \
  --distro jazzy --output-dir nix --output-as-nix-pkg-name --nixfmt \
  $(find src -name package.xml)
```

> LibSerial (crayzeewulf/LibSerial) は nixpkgs に無いため `nix/libserial.nix` で自作し、
> flake から `extraPkgs.libserial-dev` として注入している。

## 起動方法 B: RoboStack (macOS 推奨)

[RoboStack](https://robostack.github.io/) は conda-forge ベースの ROS 2 ディストリビューション。
macOS でもネイティブに ROS 2 (Jazzy) が動く (一部未検証)。Nix devShell `#robostack` が `micromamba` を提供する。

### 初回セットアップ

```bash
nix develop .#robostack          # micromamba を含む shell に入る
cd ros2_ws
./robostack/setup.sh             # ros_jazzy 環境を作成 (時間がかかる)
```

### 日常の使い方

```bash
nix develop .#robostack          # micromamba + ros_jazzy 環境を自動 activation
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

direnv を使う場合は `cd ros2_ws` するだけで自動で切り替わる
(`ros2_ws/.envrc` がルート flake の `#robostack` を読み込む)。

### macOS で実機制御 (未検証)

macOS では `/dev/tty.usbmodem*` として認識されるシリアルポートを直接使用できる想定 (socat ブリッジ不要)。
実機制御は `libserial` の conda パッケージが未確認のため、実際の動作確認が必要。

```bash
# シリアルポートを確認
ls /dev/tty.usbmodem*
# 実機起動 (要検証)
ros2 launch lerobot_controller so101_follower_controller.launch.py \
  is_sim:=False usb_port:=/dev/tty.usbmodemXXXX
```

rviz/MoveIt GUI は macOS ネイティブで動作する想定 (X11 不要)。こちらも未検証。

> **Note**: `feetech_ros2_driver` が依存する `libserial` (crayzeewulf/LibSerial) は conda-forge にパッケージが無い可能性がある。
> ビルド時に `pkg_check_modules(SERIAL libserial REQUIRED)` が失敗する場合は、
> `brew install libserial` か手動インストールが必要。


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

`teleop_ik_node` は follower controller と組み合わせて使用する。`/teleop/target` を購読し、
IK が収束しない場合は関節指令を publish しない。IK の各反復と出力値は URDF の関節上下限に clamp される。
joint 4・5 を回転目標に固定した上で、joint 1〜3 のみを位置 IK で解く。
各フレームは直前に成功した IK 解を初期値にし、非収束時は最後の成功解を維持する。

### Gamepad 制御

ゲームパッド (joy_node) で SO-101 follower を制御するには:

ros2 launch teleop_ik gamepad_teleop.launch.py

## 検証状況

- Nix: `nix develop .#ros` の評価 (`x86_64-linux` / `aarch64-linux`) まで確認済み。
- RoboStack: `nix develop .#robostack` の評価 (`aarch64-darwin`) まで確認済み。
  `micromamba env create` / `colcon build` / rviz 起動は未検証。
- **未検証 (要 Linux 実機/VM)**: `colcon build` の実通過、rviz/Gazebo/MoveIt の起動、
  `/dev/ttyACM0` 経由の実機サーボ動作・キャリブレーション。macOS 開発機では実行不可のため、
  Linux ホストでの確認が必要。

---

ライセンス: 各 vendored パッケージのライセンスに従う
([lerobot_ws](https://github.com/Ar-Ray-code/lerobot_ws) は Apache-2.0、
[feetech_ros2_driver](https://github.com/Ar-Ray-code/feetech_ros2_driver) は BSD)。
URDF/メッシュは [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) に基づく。
