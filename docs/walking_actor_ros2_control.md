# walking_actor ROS 2 制御手順書

## 目的と構成

`walking_actor` は Gazebo System プラグイン `ActorCommandPlugin` により、Gazebo の更新周期で直接移動します。ROS 2 の指令は `ros_gz_bridge` が Gazebo Transport に変換します。従来の `human_cmd_vel_controller.py` は `walking_actor` では起動しません。`person_standing` と `custom_human` は従来どおり `set_pose` サービスによる制御です。

## ビルド

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select gz_human_sim sobits_teleop
source install/setup.bash
```

Gazebo world を別途起動してから、Actor を spawn します。

```bash
ros2 launch gz_human_sim spawn_human.launch.py \
  human_model:=walking_actor model_name:=walker namespace:=human1
```

## 初期高さ

歩行メッシュの足先は Actor 原点から約 `-0.859 m` にあるため、モデル定義で `+0.86 m` の高さ補正を設定しています。通常は spawn 時に `z:=1.0` を指定してください。

## ROS 2 API

| 用途 | namespace なし | `namespace:=human1` | 型 |
| --- | --- | --- | --- |
| 速度指令 | `/cmd_vel` | `/human1/cmd_vel` | `geometry_msgs/msg/Twist` |
| 経路指令 | `/cmd_path` | `/human1/cmd_path` | `geometry_msgs/msg/PoseArray` |

複数 Actor を使うときは、`namespace` と `model_name` の両方を一意にしてください。

### 速度制御例

```bash
ros2 topic pub --rate 10 /human1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}'
```

停止は速度がすべて 0 の `Twist` を publish します。`linear.x` は前後、`linear.y` は左右、`angular.z` はヨー角速度です。

### 経路追従例

`/human1/cmd_path` に `PoseArray` を publish すると、各 pose の XY 座標へ順に移動します。waypoint 到達時には pose の姿勢を適用し、次の waypoint がある場合は続けて追従します。

## 計算式

速度制御は Actor 座標系の速度 `(v_x, v_y)` を world 座標系へ回転して、シミュレーション刻み `dt` ごとに積分します。現在のヨー角を `theta` とすると、

```text
x_next = x + (v_x cos(theta) - v_y sin(theta)) dt
y_next = y + (v_x sin(theta) + v_y cos(theta)) dt
theta_next = theta + omega_z dt
```

です。これにより、前進・横移動の向きは Actor の現在姿勢に追従します。経路追従では目標 waypoint への単位方向ベクトルに `linear_velocity * dt` を掛けて進め、残距離が `linear_tolerance` 以下なら waypoint へ到達したものと判定します。

歩行アニメーション時刻は移動距離 `d` に対して `animation_factor * d` 秒だけ進めます。回転だけではアニメーションを進めないため、停止中の足踏みを避けます。

## 主なプラグイン設定

`models/walking_actor/model.sdf` の `vel_topic`、`path_topic`、`linear_velocity`、`linear_tolerance`、`animation_factor` を変更できます。launch は namespace に合わせて最初の二つを展開済み SDF に自動設定します。

## トラブルシュート

- Actor が動かない場合は、`ros2 topic echo /<namespace>/cmd_vel` で指令を確認し、bridge のログにエラーがないことを確認します。
- mesh の URI エラーが出る場合は、`gz_human_sim` を再ビルド・source してから起動します。
- 複数 Actor が同時に動く場合は、各起動で異なる `namespace` を指定してください。
