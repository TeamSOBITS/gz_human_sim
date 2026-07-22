<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# GZ HUMAN SIM

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
    </li>
    <li>
      <a href="#実行・操作方法">実行・操作方法</a>
      <ul>
        <li><a href="#humanをspawnする">humanをspawnする</a></li>
        <li><a href="#spawnと同時にteleopを起動する">spawnと同時にteleopを起動する</a></li>
        <li><a href="#複数のhumanをspawnする">複数のhumanをspawnする</a></li>
        <li><a href="#walking_actorを座らせる">walking_actorを座らせる</a></li>
        <li><a href="#主なlaunch引数">主なlaunch引数</a></li>
      </ul>
    </li>
    <li><a href="#パッケージ構成">パッケージ構成</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#参考資料">参考資料</a></li>
  </ol>
</details>



<!-- 概要 -->
## 概要

`gz_human_sim` は Gazebo Sim 上の human model を扱うためのパッケージです．

このパッケージでは主に以下を担当します．

- human model の spawn
- actor animation の設定
- `cmd_vel` に基づく human pose 更新
- `sobits_teleop` と連携した Gazebo human teleop

現在の役割分担は以下の通りです．

- `sobits_teleop`
  - keyboard, PS4, PS5, Meta Quest などの入力デバイスから `cmd_vel` を生成する
- `gz_human_sim`
  - Gazebo 上の human を spawn し，`cmd_vel` に従って pose を更新する

`walking_actor` を使用する場合は Gazebo actor animation を利用できます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 環境構築 -->
## 環境構築

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Gazebo | Harmonic |
| Python | 3.12~ |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROS の `src` フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. 本レポジトリを clone します．
    ```sh
    $ git clone https://github.com/TeamSOBITS/gz_human_sim
    ```

3. ワークスペースをビルドします．
    ```bash
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install --packages-select gz_human_sim sobits_teleop
    $ source ~/colcon_ws/install/setup.bash
    ```

> [!NOTE]
> `gz_human_sim` は `sobits_teleop` と連携して human teleop を行うため，基本的には両方をビルドしてください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

`gz_human_sim` を使う上での基本的な流れ

1. Gazebo world を起動する
   - `spawn_human.launch.py` から human を spawn する world を先に起動する．
2. human を spawn する
   - `gz_human_sim` の launch から human model を Gazebo に配置する．
3. 必要であれば teleop を起動する
   - `enable_teleop:=true` にすると `sobits_teleop` 側の human teleop stack も同時に起動する．

### humanモデルをRVizで表示する

human モデルの関節を GUI で調整しながら確認したい場合は、RViz 表示用の launch を使います。

```bash
$ ros2 launch gz_human_sim display_human.launch.py
```

`use_gui:=True` のときは `joint_state_publisher_gui` が起動し、`use_gui:=False` のときは `joint_state_publisher` が起動します。RViz には human モデル表示に必要な RobotModel と TF の設定が読み込まれます。

#### 表示されない場合のチェック

- RVizの`RobotModel`が有効で、`Robot Description`が`robot_description`になっているか
- `Fixed Frame`がURDFの基準リンク(例: `Pelvis`)と一致しているか
- `ros2 pkg prefix gz_human_sim`でパッケージが解決できるか(インストールと`source`漏れの確認)
- `/joint_states`が出ているか (`use_gui:=True`時はGUI操作で値が更新されるか)
- `use_sim_time`を使う場合は `use_sim_time:=True` を指定して時間基準を揃える

#### 推奨起動例

```bash
$ ros2 launch gz_human_sim display_human.launch.py use_gui:=True use_sim_time:=False
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### humanをspawnする

human だけを Gazebo に spawn します．

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py
```

例:

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    model_name:=gz_human \
    human_model:=person_standing \
    x:=-2.0 y:=1.5 z:=0.0 yaw:=0.0
```

custom human の姿勢を使う場合は、`human_model:=custom_human` と `human_pose:=...` を指定します。利用可能な姿勢は `config/human_pose_presets.yaml` に定義されています。

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
  model_name:=gz_human \
  human_model:=custom_human \
  human_pose:=raise_right_hand
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### spawnと同時にteleopを起動する

spawn と同時に `sobits_teleop` の Gazebo human teleop を起動します．

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py enable_teleop:=true
```

keyboard を使う例:

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    enable_teleop:=true \
    device:=keyboard
```

PS4 を使う例:

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    enable_teleop:=true \
    device:=ps4
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 複数のhumanをspawnする

複数の human を使う場合は，少なくとも `namespace` と `model_name` を変えてください．

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    namespace:=human1 model_name:=gz_human_1 x:=-2.0 y:=1.5 enable_teleop:=true

$ ros2 launch gz_human_sim spawn_human.launch.py \
    namespace:=human2 model_name:=gz_human_2 x:=-1.0 y:=1.5 enable_teleop:=true
```

> [!NOTE]
> `namespace` は ROS node / topic の衝突を避けるために使用します．
> `model_name` は Gazebo 上の entity 名なので，複数 spawn する場合は必ず一意にしてください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### walking_actorを座らせる

`walking_actor` は Gazebo GUI の Human Control パネルから座らせることができます
（`sit_down` → `sitting` → `stand_up` のアニメーション遷移込み）．座っている間は
その場に固定され，移動コマンドは無視されます．

- キーボード `K` を押している間だけ座り，離すと立ちます．
- `K` を押しながら `Space` を押すと座ったまま固定され，`K` を離しても座り続けます．
  もう一度 `Space` を押すと固定を解除して立ちます．
- パネル内の「座らせる（固定）」ボタンでも同じ操作ができます（`K` を使わずに固定できます）．

`DoctorFemaleWalk` には座りアニメーション用のメッシュがないため，この機能は
`walking_actor` のみが対象です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 主なlaunch引数

`spawn_human.launch.py` の主な引数は以下の通りです．

| Argument | Description |
| --- | --- |
| `namespace` | ROS namespace |
| `world_name` | Gazebo world 名 |
| `enable_teleop` | `true` のとき `sobits_teleop` の human teleop を同時起動 |
| `device` | `keyboard`, `ps4`, `ps5`, `quest` |
| `model_name` | Gazebo 上の human entity 名 |
| `human_model` | `person_standing` または `walking_actor` |
| `human_pose` | `custom_human` 使用時の姿勢プリセット。`config/human_pose_presets.yaml` に定義 |
| `model_file` | 明示的な SDF ファイルパス．指定時は `human_model` より優先 |
| `x`, `y`, `z`, `yaw` | spawn pose |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- パッケージ構成 -->
## パッケージ構成

- `launch/spawn_human.launch.py`
  - Gazebo 上に human を spawn する launch
- `scripts/human_cmd_vel_controller.py`
  - `cmd_vel` を受け取り Gazebo の human pose を更新するノード
- `models/walking_actor.sdf`
  - actor animation 用の SDF
- `src/actor_command_plugin.cpp`
  - actor の移動（cmd_vel/cmd_path/follow_mode）・jump・座り状態機械
    （sit_down/sitting/stand_up の切り替えと移動の凍結）を担当する Gazebo plugin
- `src/actor_animation_control_plugin.cpp`
  - actor の animation 制御用 Gazebo plugin

関連パッケージ:

- `sobits_teleop/launch/gz_human_teleop.launch.py`
  - Gazebo human 用 teleop launch
- `sobits_teleop/config/gz_human/`
  - human 用 controller 設定

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [ ] sam3_body を用いた human model 生成
- [ ] body link remapping 対応
- [ ] actor animation の拡張

現時点のバグや新規機能の依頼を確認するために [Issue ページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考資料 -->
## 参考資料

- [blackcoffeerobotics/gazebo-ros-actor-plugin](https://github.com/blackcoffeerobotics/gazebo-ros-actor-plugin)
  - ROS actor plugin．`src/gazebo-ros-actor-plugin`（`jazzy-harmonic` ブランチ，Gazebo Sim / ROS 2 Jazzy 対応版）を vendor しており，`actor_command_plugin.cpp` / `actor_animation_control_plugin.cpp` の元となった参考実装．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/gz_human_sim.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/gz_human_sim/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/gz_human_sim.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/gz_human_sim/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/gz_human_sim.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/gz_human_sim/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/gz_human_sim.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/gz_human_sim/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/gz_human_sim.svg?style=for-the-badge
[license-url]: LICENSE
