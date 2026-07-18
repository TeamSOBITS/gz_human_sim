import os
import re
import subprocess
import tempfile

import yaml

CUSTOM_HUMAN_Z_OFFSET = 0.9673


def resolve_walking_actor_model(package_share, velocity_topic, path_topic):
    """Return a path to a walking_actor model.sdf with model:// mesh URIs
    baked into absolute paths.

    gzserver is typically already running (started by an outer launch file,
    e.g. sobit_edu's gz_minimal.launch.py) by the time this package's
    spawn_human.launch.py runs. Setting GZ_SIM_RESOURCE_PATH from within
    spawn_human.launch.py only affects processes it spawns itself, not the
    already-running gzserver/gzclient, so model://walking_actor/... URIs
    inside the actor's own model.sdf never get resolved by the server.
    Rewriting them to absolute paths sidesteps resource-path resolution
    entirely, the same way generate_custom_human_model() already does.
    """
    model_dir = os.path.join(package_share, 'models', 'walking_actor')
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

    resolved_text, count = re.subn(
        r'model://walking_actor/meshes/', f'{mesh_dir}/', sdf_text
    )
    resolved_text = resolved_text.replace(
        'filename="libgz_human_actor_command.so"',
        f'filename="{plugin_library}"',
        1,
    )
    resolved_text = resolved_text.replace("<vel_topic>/cmd_vel</vel_topic>", f"<vel_topic>{velocity_topic}</vel_topic>", 1)
    resolved_text = resolved_text.replace("<path_topic>/cmd_path</path_topic>", f"<path_topic>{path_topic}</path_topic>", 1)
    if count == 0:
        raise RuntimeError(
            f"No 'model://walking_actor/meshes/' URIs found in {source_sdf}. "
            "Check that the file wasn't changed to a different URI style."
        )

    cache_dir = os.path.join(tempfile.gettempdir(), 'gz_human_sim')
    os.makedirs(cache_dir, exist_ok=True)
    topic_key = re.sub(r'[^A-Za-z0-9_.-]', '_', velocity_topic)
    generated_sdf = os.path.join(cache_dir, f'walking_actor_{topic_key}.sdf')
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