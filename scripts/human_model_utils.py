import os
import re
import subprocess
import tempfile

import yaml

CUSTOM_HUMAN_Z_OFFSET = 0.9673


# Actor models that ship with a ready-to-spawn model.sdf under models/<name>/
# and can be driven by ActorCommandPlugin. DoctorFemaleWalk carries no
# baked-in plugin (one is injected below), while walking_actor already has
# one with placeholder /cmd_vel /cmd_path /remove_actor topics.
#
# person_walking used to be listed here too; dropped because it's the same
# "Mingfei" generic-actor mesh walking_actor already vendors locally, just
# fetched from a remote Fuel URL at spawn time instead, and without its own
# plugin -- a strictly worse duplicate, not a second distinct avatar.
ACTOR_MODEL_NAMES = ('walking_actor', 'DoctorFemaleWalk')


def resolve_actor_model(package_share, model_name, velocity_topic, path_topic,
                         remove_topic, follow_mode_topic, jump_topic='/cmd_jump',
                         sit_topic='/cmd_sit',
                         collision_model_name='', collision_cmd_vel_topic='',
                         follow_mode='auto', animation_name='walk', animation_factor=4.0,
                         linear_velocity=1.0, linear_tolerance=0.1):
    """Return a path to a <model_name> model.sdf ready for runtime spawn.

    gzserver is typically already running by the time this package's
    spawn_human.launch.py runs. Setting GZ_SIM_RESOURCE_PATH from within
    spawn_human.launch.py only affects processes it spawns itself, not the
    already-running gzserver/gzclient, so model://<model_name>/... URIs
    inside the actor's own model.sdf never get resolved by the server.
    Rewriting them to absolute paths sidesteps resource-path resolution
    entirely, the same way generate_custom_human_model() already does.

    The model must carry a gz_human_sim::ActorCommandPlugin block wired to
    velocity_topic/path_topic: one is injected if the model.sdf doesn't
    already have a <plugin>, otherwise the existing block's topics are
    rewritten (this is what walking_actor's model.sdf relies on).
    """
    model_dir = os.path.join(package_share, 'models', model_name)
    mesh_dir = os.path.join(model_dir, 'meshes')
    package_prefix = os.path.dirname(os.path.dirname(package_share))
    plugin_library = os.path.join(
        package_prefix, 'lib', 'libgz_human_actor_command.so'
    )
    if not os.path.isfile(plugin_library):
        raise RuntimeError(f'Actor command plugin not found: {plugin_library}')
    source_sdf = os.path.join(model_dir, 'model.sdf')

    with open(source_sdf, 'r', encoding='utf-8') as sdf_file:
        sdf_text = sdf_file.read()

    resolved_text = sdf_text.replace(f'model://{model_name}/meshes/', f'{mesh_dir}/')

    if '<plugin' in resolved_text:
        resolved_text = resolved_text.replace(
            'filename="libgz_human_actor_command.so"',
            f'filename="{plugin_library}"',
            1,
        )
        resolved_text = resolved_text.replace(
            '<vel_topic>/cmd_vel</vel_topic>', f'<vel_topic>{velocity_topic}</vel_topic>', 1)
        resolved_text = resolved_text.replace(
            '<path_topic>/cmd_path</path_topic>', f'<path_topic>{path_topic}</path_topic>', 1)
        resolved_text = resolved_text.replace(
            '<remove_topic>/remove_actor</remove_topic>',
            f'<remove_topic>{remove_topic}</remove_topic>', 1)
        resolved_text = resolved_text.replace(
            '<follow_mode_topic>/set_follow_mode</follow_mode_topic>',
            f'<follow_mode_topic>{follow_mode_topic}</follow_mode_topic>', 1)
        resolved_text = resolved_text.replace(
            '<jump_topic>/cmd_jump</jump_topic>', f'<jump_topic>{jump_topic}</jump_topic>', 1)
        resolved_text = resolved_text.replace(
            '<sit_topic>/cmd_sit</sit_topic>', f'<sit_topic>{sit_topic}</sit_topic>', 1)
        resolved_text = resolved_text.replace(
            '<collision_model_name></collision_model_name>',
            f'<collision_model_name>{collision_model_name}</collision_model_name>', 1)
        resolved_text = resolved_text.replace(
            '<collision_cmd_vel_topic></collision_cmd_vel_topic>',
            f'<collision_cmd_vel_topic>{collision_cmd_vel_topic}</collision_cmd_vel_topic>', 1)
        resolved_text = resolved_text.replace(
            '<follow_mode>auto</follow_mode>', f'<follow_mode>{follow_mode}</follow_mode>', 1)
    else:
        plugin_block = (
            f'<plugin filename="{plugin_library}" '
            'name="gz_human_sim::ActorCommandPlugin">'
            f'<vel_topic>{velocity_topic}</vel_topic>'
            f'<path_topic>{path_topic}</path_topic>'
            f'<remove_topic>{remove_topic}</remove_topic>'
            f'<follow_mode_topic>{follow_mode_topic}</follow_mode_topic>'
            f'<jump_topic>{jump_topic}</jump_topic>'
            f'<collision_model_name>{collision_model_name}</collision_model_name>'
            f'<collision_cmd_vel_topic>{collision_cmd_vel_topic}</collision_cmd_vel_topic>'
            f'<follow_mode>{follow_mode}</follow_mode>'
            f'<animation_name>{animation_name}</animation_name>'
            f'<animation_factor>{animation_factor}</animation_factor>'
            f'<linear_tolerance>{linear_tolerance}</linear_tolerance>'
            f'<linear_velocity>{linear_velocity}</linear_velocity>'
            '</plugin>'
        )
        resolved_text, count = re.subn(
            r'(<actor\s+name="[^"]*">)', r'\1' + plugin_block, resolved_text, count=1
        )
        if count == 0:
            raise RuntimeError(f'No <actor name="..."> element found in {source_sdf}.')

    cache_dir = os.path.join(tempfile.gettempdir(), 'gz_human_sim')
    os.makedirs(cache_dir, exist_ok=True)
    topic_key = re.sub(r'[^A-Za-z0-9_.-]', '_', f'{model_name}_{velocity_topic}')
    generated_sdf = os.path.join(cache_dir, f'{topic_key}.sdf')
    with open(generated_sdf, 'w', encoding='utf-8') as sdf_file:
        sdf_file.write(resolved_text)

    return generated_sdf


def resolve_collision_body_model(package_share, spawn_name, cmd_vel_topic):
    """Return a path to a human_collision_body model.sdf ready to spawn
    alongside an actor named spawn_name, with its VelocityControl plugin
    listening on cmd_vel_topic.

    See models/human_collision_body/model.sdf's own comment for why this
    body exists at all: gz-sim actors are pure kinematic TrajectoryPose
    overwrites with no physics/collision involvement, so this ordinary
    dynamic model (with real mass + collision) stands in for the actor's
    footprint instead -- ActorCommandPlugin drives it over cmd_vel_topic
    and reads its physics-resolved pose back each tick (see
    collision_model_name/collision_cmd_vel_topic in resolve_actor_model()
    above). No plugin-library path rewriting is needed here (unlike
    resolve_actor_model()) since VelocityControl is a stock gz-sim system,
    not one this package builds.
    """
    model_dir = os.path.join(package_share, 'models', 'human_collision_body')
    source_sdf = os.path.join(model_dir, 'model.sdf')

    with open(source_sdf, 'r', encoding='utf-8') as sdf_file:
        sdf_text = sdf_file.read()

    resolved_text = sdf_text.replace(
        '<model name="human_collision_body">',
        f'<model name="{spawn_name}">', 1)
    resolved_text = resolved_text.replace(
        '<topic>/model/human_collision_body/cmd_vel</topic>',
        f'<topic>{cmd_vel_topic}</topic>', 1)

    cache_dir = os.path.join(tempfile.gettempdir(), 'gz_human_sim')
    os.makedirs(cache_dir, exist_ok=True)
    name_key = re.sub(r'[^A-Za-z0-9_.-]', '_', spawn_name)
    generated_sdf = os.path.join(cache_dir, f'{name_key}_collision.sdf')
    with open(generated_sdf, 'w', encoding='utf-8') as sdf_file:
        sdf_file.write(resolved_text)

    return generated_sdf


def set_fixed_joint_pose(sdf_text, joint_name, child_link_name, rpy):
    joint_pattern = rf"(<joint name='{re.escape(joint_name)}' type=)'[^']+('>)"
    sdf_text, joint_count = re.subn(joint_pattern, r"\1'fixed\2", sdf_text, count=1)
    if joint_count != 1:
        raise RuntimeError(f"Failed to convert joint '{joint_name}' to fixed.")

    pose_pattern = (
        rf"(<link name='{re.escape(child_link_name)}'>\s+"
        rf"<pose relative_to='{re.escape(joint_name)}'>)"
        r"[^<]+"
        r"(</pose>)"
    )
    replacement = rf"\g<1>0 0 0 {rpy[0]} {rpy[1]} {rpy[2]}\g<2>"
    new_text, count = re.subn(pose_pattern, replacement, sdf_text, count=1)
    if count != 1:
        raise RuntimeError(
            f"Failed to bake pose for child '{child_link_name}' driven by joint '{joint_name}'."
        )
    return new_text


def set_root_link_height_offset(sdf_text, root_link_name, z_offset):
    pattern = rf"(<link name='{re.escape(root_link_name)}'>)"
    replacement = rf"\1\n      <pose>0 0 {z_offset} 0 0 0</pose>"
    new_text, count = re.subn(pattern, replacement, sdf_text, count=1)
    if count != 1:
        raise RuntimeError(
            f"Failed to apply z offset to root link '{root_link_name}'."
        )
    return new_text


def load_pose_presets(package_share):
    config_path = os.path.join(package_share, 'config', 'human_pose_presets.yaml')
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}

    pose_presets = config.get('pose_presets', {})
    if not pose_presets:
        raise RuntimeError(f'No pose_presets found in {config_path}.')

    return pose_presets


def generate_custom_human_model(package_share, human_pose):
    pose_presets = load_pose_presets(package_share)

    if human_pose not in pose_presets:
        supported = ', '.join(sorted(pose_presets))
        raise RuntimeError(
            f"Unsupported human_pose '{human_pose}'. Supported values: {supported}."
        )

    model_dir = os.path.join(package_share, 'models', 'human_gazebo_raised_hand')
    mesh_dir = os.path.join(model_dir, 'meshes')
    package_prefix = os.path.dirname(os.path.dirname(package_share))
    plugin_library = os.path.join(
        package_prefix, 'lib', 'libgz_human_actor_command.so'
    )
    if not os.path.isfile(plugin_library):
        raise RuntimeError(f'Actor command plugin not found: {plugin_library}')
    source_urdf = os.path.join(model_dir, 'humanSubjectWithMesh.urdf')

    cache_dir = os.path.join(tempfile.gettempdir(), 'gz_human_sim')
    os.makedirs(cache_dir, exist_ok=True)
    generated_sdf = os.path.join(cache_dir, f'custom_human_{human_pose}.sdf')

    result = subprocess.run(
        ['gz', 'sdf', '-p', source_urdf],
        check=True,
        capture_output=True,
        text=True,
    )

    sdf_text = result.stdout
    sdf_text = sdf_text.replace('<uri>meshes/', f'<uri>{mesh_dir}/')
    sdf_text = sdf_text.replace(
        "<model name='HumanModel'>",
        "<model name='HumanModel'>\n    <static>true</static>",
        1,
    )
    sdf_text = set_root_link_height_offset(sdf_text, 'Pelvis', CUSTOM_HUMAN_Z_OFFSET)

    for pose in pose_presets[human_pose]:
        joint_name = pose['joint_name']
        child_link_name = pose['child_link_name']
        rpy = pose['rpy']
        sdf_text = set_fixed_joint_pose(sdf_text, joint_name, child_link_name, rpy)

    with open(generated_sdf, 'w', encoding='utf-8') as sdf_file:
        sdf_file.write(sdf_text)

    return generated_sdf