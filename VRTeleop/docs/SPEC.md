# VRTeleop 仕様

## 目的

Quest 3 の右コントローラー位置を SO-101 follower arm のエンドエフェクタ目標として扱い、Python ブリッジ側で IK を解いて関節指令に変換する。指令の送信先は実機と MuJoCo シミュレーションを切り替えられるようにする。

## 構成

- Godot 4 OpenXR プロジェクト
  - `XRController3D` から右コントローラーの pose と入力値を読む。
  - UDP JSON で Python ブリッジへ送信する。
  - ロボット制御ロジックは持たない。
- Python ブリッジ
  - UDP で controller pose を受信する。
  - 座標変換、ワークスペース制限、IK、関節速度制限を行う。
  - `dry-run`、`real-lerobot`、`mujoco` の backend に同じ joint action を送る。

## 通信

Godot から Python へ UDP `127.0.0.1:50530` に JSON を送る。

```json
{
  "version": 1,
  "seq": 1,
  "timestamp_usec": 123456,
  "source": "quest3/right_controller",
  "position": [0.2, 1.1, -0.5],
  "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
  "trigger": 0.0,
  "grip": 0.0,
  "enabled": true
}
```

`enabled=false` のフレームでは送信先 backend に指令しない。

## 関節名

LeRobot の SO follower 系に合わせ、次の action key を使う。

- `shoulder_pan.pos`
- `shoulder_lift.pos`
- `elbow_flex.pos`
- `wrist_flex.pos`
- `wrist_roll.pos`
- `gripper.pos`

実機 backend は degree 指令、MuJoCo backend は actuator `ctrl` に radian 指令を入れる。`gripper.pos` は 0..100 のスカラーとして扱う。

## IK

初期実装は SO-101 の 5 軸アーム + gripper に対する軽量な幾何 IK とする。

- base yaw: XY 平面の目標方向から解く。
- shoulder / elbow: 2 リンク平面 IK で解く。
- wrist flex: controller pitch から前腕姿勢を補正する。
- wrist roll: controller roll を使う。
- gripper: Quest 3 trigger を 0..100 に線形変換する。

特異点、到達不能点、急激な姿勢変化は Python ブリッジで clamp する。精密な URDF/MJCF ベース IK は後続で差し替えられるよう、`IKSolver` interface に閉じ込める。

## 安全

- 既定 backend は `dry-run`。
- 実機 backend は `--backend real-lerobot` を明示した時だけ使う。
- `max_step_deg` で 1 フレームあたりの関節変化を制限する。
- `enabled` が false、または受信 timeout の場合は新規指令を送らない。
- 実機 disconnect 時は LeRobot の `disable_torque_on_disconnect` を有効にする。

## 起動

```bash
python -m vrteleop_bridge --config config/default.json --backend dry-run
python -m vrteleop_bridge --config config/default.json --backend real-lerobot --port /dev/tty.usbmodemXXXX --robot-id so101_follower
python -m vrteleop_bridge --config config/default.json --backend mujoco --mjcf sim/so101_minimal.xml
```

Godot 側は `project.godot` を Godot 4 で開き、OpenXR 対応ランタイムに接続して実行する。

`sim/so101_minimal.xml` は backend 検証用の最小モデルであり、精密な実機 twin ではない。実機に近い検証を行う場合は、同じ actuator 名を持つ MJCF に差し替える。
