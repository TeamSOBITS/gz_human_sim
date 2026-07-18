# Agents.md - Development Rules & Mission

## 1. System Environment
* **OS**: Ubuntu 24.04
* **Environment**: ROS 2 Jazzy Jalisco
* **Language Requirement**: All instructions, conversations, and explanations must be provided in **Japanese (日本語)**.

---

## 2. Coding & Development Rules
Agentは、コードの変更や新規作成を行う際、以下のルールを厳格に遵守すること。

* **既存コードの保護（原則）**: 
  既存のコードは基本的に汚さず、元のロジックや構造を最大限尊重すること。
* **例外時の報告**: 
  どうしても既存コードの修正・破壊的変更が必要な場合は、作業前に必ずユーザーへ理由を報告し、許可を得ること。
* **新規作成・変更時のプロセス**:
  1. 新規ファイルの作成や既存ファイルの変更を行う際、必ず**手順書（作業内容のサマリーや実行コマンド等）**を作成する。
  2. 実装前に変更方針を報告し、ユーザーの確認を得てから反映すること。
* **詳細な解説の義務**:
  作成・変更したコードの内容、および実装されている「計算式」については、その根拠や意味を詳細に説明すること。

---

## 3. Mission & Project Goal
### 目的
`gz_human_sim` パッケージ内にある `walking_actor` の機能に対して、`gazebo-ros-actor-plugin` の機能を移植・実装し、ROS 2 Jazzy環境下で正常に動作（速度コマンドによる制御など）を行えるようにする。

### ワークスペース構造の参考
Agentは以下の既存構造を把握し、適切なファイル変更やプラグインの統合を行うこと。

#### 対象パッケージ 1: gz_human_sim[cite: 1]
人型モデルの管理やアニメーション制御を行うメインパッケージ。
```text
gz_human_sim/
├── CMakeLists.txt
├── package.xml
├── models/
│   └── walking_actor/       # 本ミッションの対象モデル
│       ├── meshes/          # *.dae, *.bvh などのアニメーション群
│       ├── model.config
│       └── model.sdf        # プラグインの組み込み対象
├── scripts/
│   ├── human_cmd_vel_controller.py
│   └── human_model_utils.py
└── src/
    └── actor_animation_control_plugin.cpp

```

#### 対象パッケージ 2: gazebo-ros-actor-plugin



移植元となる、Gazebo上のActorをROSから制御するためのプラグインパッケージ。

```text
gazebo-ros-actor-plugin/
├── CMakeLists.txt
├── package.xml
├── include/gazebo_ros_actor_plugin/
│   └── gazebo_ros_actor_command.h
└── src/
    └── gazebo_ros_actor_command.cpp

```

### 実装へのアプローチ

1. `gazebo-ros-actor-plugin` の実装（`gazebo_ros_actor_command.cpp` など）を解析する。


2. `gz_human_sim` の環境（ROS 2 Jazzy / 新しいGazebo）に適合するよう、`walking_actor` の `model.sdf` へのプラグイン記述追加、または `src/` 内のプラグインコードへの統合を行う。


3. 変更を加える前に、必ず「どのファイルをどう変えるか」の手順書を作成して提示すること。

```
