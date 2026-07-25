"""Minimal SfmCrowdSystem demo: gz sim + worlds/sfm_crowd_demo.world (flat
floor, two corridor walls, 3 fixed SFM goals) with three walking_actor
humans spawned into it via the existing spawn_human.launch.py -- one
IncludeLaunchDescription per namespace (human1/human2/human3), matching the
collision_model/cmd_vel_topic names the world file's <human> blocks
already declare. See this package's README for the full walkthrough
(build/run/spawn/enable-disable SFM/verify).

enable_teleop defaults to false for all three here (SfmCrowdSystem is meant
to be the one driving cmd_vel for this demo); publish `false` on a given
human's sfm_enable_topic first if you want to drive that one manually
instead, so its own SFM commands stop competing with your teleop input.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('gz_human_sim'), 'worlds', 'sfm_crowd_demo.world'])

    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r ', world_file]}.items())

    def spawn_human(namespace, x, y, yaw):
        return IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('gz_human_sim'), 'launch', 'spawn_human.launch.py']),
            launch_arguments={
                'namespace': namespace, 'model_name': namespace,
                'world_name': 'sfm_crowd_demo', 'human_model': 'walking_actor',
                'enable_teleop': 'false',
                'x': x, 'y': y, 'z': '1.0', 'yaw': yaw,
            }.items())

    # Spawn poses match this world's <human> goal cycles (see
    # worlds/sfm_crowd_demo.world) so each human starts already near its
    # own first goal's approach line, instead of walking in from an
    # unrelated corner on the very first tick.
    return LaunchDescription([
        gz_sim,
        spawn_human('human1', '-2.0', '-1.0', '0.0'),
        spawn_human('human2', '10.0', '1.0', '3.14159'),
        spawn_human('human3', '4.0', '-1.8', '1.5708'),
    ])
