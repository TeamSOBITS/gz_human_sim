<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# GZ HUMAN SIM

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#overview">Overview</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
    </li>
    <li>
      <a href="#usage">Usage</a>
      <ul>
        <li><a href="#spawn-a-human">Spawn a Human</a></li>
        <li><a href="#spawn-and-start-teleop">Spawn and Start Teleop</a></li>
        <li><a href="#spawn-multiple-humans">Spawn Multiple Humans</a></li>
        <li><a href="#main-launch-arguments">Main Launch Arguments</a></li>
      </ul>
    </li>
    <li><a href="#package-layout">Package Layout</a></li>
    <li><a href="#milestones">Milestones</a></li>
  </ol>
</details>



<!-- Overview -->
## Overview

`gz_human_sim` is a package for handling simulated human models in Gazebo Sim.

This package is mainly responsible for:

- spawning human models
- configuring actor animation
- updating human pose from `cmd_vel`
- integrating Gazebo human teleop with `sobits_teleop`

The current responsibility split is:

- `sobits_teleop`
  - generates `cmd_vel` from input devices such as keyboard, PS4, PS5, and Meta Quest
- `gz_human_sim`
  - spawns the human in Gazebo and updates its pose from `cmd_vel`

When using `walking_actor`, Gazebo actor animation can also be used.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Setup -->
## Setup

This section explains how to set up the package.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Requirements

Please prepare the following environment before installation.

| System | Version |
| --- | --- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |
| Gazebo | Harmonic |
| Python | 3.12~ |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1. Move to the ROS `src` directory.
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/gz_human_sim
    ```

3. Build the workspace.
    ```bash
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install --packages-select gz_human_sim sobits_teleop
    $ source ~/colcon_ws/install/setup.bash
    ```

> [!NOTE]
> `gz_human_sim` works together with `sobits_teleop` for human teleoperation, so in most cases both packages should be built.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Usage -->
## Usage

Typical workflow when using `gz_human_sim`:

1. Start a Gazebo world.
   - The world where the human will be spawned should already be running.
2. Spawn the human.
   - Use the `gz_human_sim` launch file to place a human model in Gazebo.
3. Start teleop if needed.
   - Set `enable_teleop:=true` to launch the human teleop stack from `sobits_teleop` at the same time.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Spawn a Human

Spawn only the human entity in Gazebo.

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py
```

Example:

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    model_name:=gz_human \
    human_model:=person_standing \
    x:=-2.0 y:=1.5 z:=0.0 yaw:=0.0
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Spawn and Start Teleop

Spawn the human and start Gazebo human teleop at the same time.

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py enable_teleop:=true
```

Example with keyboard:

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    enable_teleop:=true \
    device:=keyboard
```

Example with PS4:

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    enable_teleop:=true \
    device:=ps4
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Spawn Multiple Humans

When using multiple humans, change at least `namespace` and `model_name`.

```bash
$ ros2 launch gz_human_sim spawn_human.launch.py \
    namespace:=human1 model_name:=gz_human_1 x:=-2.0 y:=1.5 enable_teleop:=true

$ ros2 launch gz_human_sim spawn_human.launch.py \
    namespace:=human2 model_name:=gz_human_2 x:=-1.0 y:=1.5 enable_teleop:=true
```

> [!NOTE]
> `namespace` avoids ROS node and topic conflicts.
> `model_name` is the Gazebo entity name, so it must be unique for each spawned human.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Main Launch Arguments

Main arguments of `spawn_human.launch.py`:

| Argument | Description |
| --- | --- |
| `namespace` | ROS namespace |
| `world_name` | Gazebo world name |
| `enable_teleop` | Launch human teleop from `sobits_teleop` when `true` |
| `device` | `keyboard`, `ps4`, `ps5`, `quest` |
| `model_name` | Human entity name in Gazebo |
| `human_model` | `person_standing` or `walking_actor` |
| `model_file` | Explicit SDF path. If set, it overrides `human_model` |
| `x`, `y`, `z`, `yaw` | Spawn pose |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Package Layout -->
## Package Layout

- `launch/spawn_human.launch.py`
  - Launch file for spawning a human in Gazebo
- `scripts/human_cmd_vel_controller.py`
  - Node that updates the Gazebo human pose from `cmd_vel`
- `models/walking_actor.sdf`
  - SDF for actor-based animation
- `src/actor_animation_control_plugin.cpp`
  - Gazebo plugin for actor animation control

Related package paths:

- `sobits_teleop/launch/gz_human_teleop.launch.py`
  - Teleop launch for Gazebo humans
- `sobits_teleop/config/gz_human/`
  - Human controller configs

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- Milestones -->
## Milestones

- [ ] Human model generation using sam3_body
- [ ] Body link remapping support
- [ ] Extended actor animation support

Please see the [Issues page][issues-url] for current bugs and feature requests.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


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
