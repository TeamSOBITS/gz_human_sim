import importlib.util
import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_human_model_utils(context):
    package_share = FindPackageShare('gz_human_sim').perform(context)
    module_path = os.path.join(package_share, 'scripts', 'human_model_utils.py')
    spec = importlib.util.spec_from_file_location('human_model_utils', module_path)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError(f'Failed to load helper module: {module_path}')
    spec.loader.exec_module(module)
    return module


def _actor_topic(namespace, topic_name):
    normalized_namespace = namespace.strip('/')
    return f'/{normalized_namespace}/{topic_name}' if normalized_namespace else f'/{topic_name}'


def _generate_custom_human_model(context):
    package_share = FindPackageShare('gz_human_sim').perform(context)
    human_pose = LaunchConfiguration('human_pose').perform(context)
    return _load_human_model_utils(context).generate_custom_human_model(
        package_share, human_pose)


def _resolve_actor_model(context, namespace, model_name):
    package_share = FindPackageShare('gz_human_sim').perform(context)
    follow_mode = LaunchConfiguration('follow_mode').perform(context)
    return _load_human_model_utils(context).resolve_actor_model(
        package_share, model_name, _actor_topic(namespace, 'cmd_vel'),
        _actor_topic(namespace, 'cmd_path'), _actor_topic(namespace, 'remove_actor'),
        _actor_topic(namespace, 'set_follow_mode'),
        jump_topic=_actor_topic(namespace, 'cmd_jump'), follow_mode=follow_mode)


def _spawn_human_cmd(context, *_args, **_kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    human_model = LaunchConfiguration('human_model').perform(context)
    model_file = LaunchConfiguration('model_file').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    actor_model_names = _load_human_model_utils(context).ACTOR_MODEL_NAMES

    if not model_file:
        if human_model == 'person_standing':
            model_file = PathJoinSubstitution([
                FindPackageShare('gz_human_sim'), 'models',
                'person_standing', 'model.sdf']).perform(context)
        elif human_model in actor_model_names:
            model_file = _resolve_actor_model(context, namespace, human_model)
        elif human_model == 'custom_human':
            model_file = _generate_custom_human_model(context)
        else:
            raise RuntimeError(
                f"Unsupported human_model '{human_model}'. Use 'person_standing', "
                f"one of {actor_model_names}, 'custom_human', or pass model_file "
                "explicitly.")

    actions = []
    if human_model in actor_model_names:
        velocity_topic = _actor_topic(namespace, 'cmd_vel')
        path_topic = _actor_topic(namespace, 'cmd_path')
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='actor_command_bridge', namespace=namespace,
            output='screen', arguments=[
                f'{velocity_topic}@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'{path_topic}@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
            ]))

    actions.append(Node(
        package='ros_gz_sim', executable='create', name='spawn_human',
        namespace=namespace, output='screen', arguments=[
            '-world', world_name, '-file', model_file, '-name', model_name,
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ]))
    return actions


def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='ROS namespace for the spawn node')
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name', default_value='follower_env',
        description='Gazebo world name')
    declare_enable_teleop_cmd = DeclareLaunchArgument(
        'enable_teleop', default_value='true',
        description='Launch human teleop stack together with the spawned human')
    declare_device_cmd = DeclareLaunchArgument(
        'device', default_value='keyboard',
        description='Input device type: keyboard, ps4, ps5, quest')
    declare_model_name_cmd = DeclareLaunchArgument(
        'model_name', default_value='gz_human', description='Spawned human model name')
    declare_human_model_cmd = DeclareLaunchArgument(
        'human_model', default_value='walking_actor',
        description=(
            'Human model preset: person_standing, custom_human, or an actor '
            'model (walking_actor, DoctorFemaleWalk)'
        ))
    declare_human_pose_cmd = DeclareLaunchArgument(
        'human_pose', default_value='cross_arms', description='Custom human pose preset')
    declare_follow_mode_cmd = DeclareLaunchArgument(
        'follow_mode', default_value='auto',
        description=(
            "ActorCommandPlugin command source (actor models only): 'auto' "
            "(path takes over whenever one is active, else velocity -- the "
            "default), 'path' (ignores teleop/cmd_vel entirely), or "
            "'velocity' (ignores cmd_path entirely)"
        ))
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file', default_value='', description='Optional explicit SDF file path')
    declare_x_cmd = DeclareLaunchArgument('x', default_value='-2.0', description='Spawn x position')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='1.5', description='Spawn y position')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='1.0', description='Spawn z position')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw angle')

    namespace = LaunchConfiguration('namespace')
    human_model = LaunchConfiguration('human_model')
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    set_gz_resource_path_human = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('gz_human_sim'), 'models']))
    set_gz_resource_path_worlds = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('sobits_gazebo_worlds'), 'models']))
    human_set_pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='human_set_pose_bridge', namespace=namespace, output='screen', arguments=[[
            '/world/', world_name, '/set_pose',
            '@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean']])
    human_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('sobits_teleop'), 'launch', 'gz_human_teleop.launch.py'])]),
        launch_arguments={
            'namespace': namespace, 'device': LaunchConfiguration('device'),
            'world_name': world_name, 'model_name': model_name,
            'x': x, 'y': y, 'z': z, 'yaw': yaw,
            'use_pose_controller': PythonExpression([
                "'", human_model,
                "' not in ('walking_actor', 'DoctorFemaleWalk')"]),
        }.items(), condition=IfCondition(LaunchConfiguration('enable_teleop')))

    return LaunchDescription([
        declare_namespace_cmd, declare_world_name_cmd, declare_enable_teleop_cmd,
        declare_device_cmd, declare_model_name_cmd, declare_human_model_cmd,
        declare_human_pose_cmd, declare_follow_mode_cmd, declare_model_file_cmd,
        declare_x_cmd, declare_y_cmd,
        declare_z_cmd, declare_yaw_cmd, set_gz_resource_path_human,
        set_gz_resource_path_worlds, human_set_pose_bridge,
        OpaqueFunction(function=_spawn_human_cmd), human_teleop,
    ])
