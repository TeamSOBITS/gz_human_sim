import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_human_model_utils(context):
    package_share = FindPackageShare('gz_human_sim').perform(context)
    module_path = os.path.join(
        package_share,
        'scripts',
        'human_model_utils.py',
    )
    spec = importlib.util.spec_from_file_location('human_model_utils', module_path)
    module = importlib.util.module_from_spec(spec)
    if spec.loader is None:
        raise RuntimeError(f'Failed to load helper module: {module_path}')
    spec.loader.exec_module(module)
    return module


def _generate_custom_human_model(context):
    package_share = FindPackageShare('gz_human_sim').perform(context)
    human_pose = LaunchConfiguration('human_pose').perform(context)
    human_model_utils = _load_human_model_utils(context)
    return human_model_utils.generate_custom_human_model(package_share, human_pose)


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

    standing_model_file = PathJoinSubstitution(
        [
            FindPackageShare('sobits_gazebo_worlds'),
            'models',
            'person_standing',
            'model.sdf',
        ]
    ).perform(context)
    walking_model_file = PathJoinSubstitution(
        [
            FindPackageShare('gz_human_sim'),
            'models',
            'walking_actor.sdf',
        ]
    ).perform(context)
    if not model_file:
        if human_model == 'person_standing':
            model_file = standing_model_file
        elif human_model == 'walking_actor':
            model_file = walking_model_file
        elif human_model == 'custom_human':
            model_file = _generate_custom_human_model(context)
        else:
            raise RuntimeError(
                "Unsupported human_model '{}'. Use 'person_standing', "
                "'walking_actor', 'custom_human', or pass "
                "model_file explicitly.".format(human_model)
            )

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_human',
            namespace=namespace,
            output='screen',
            arguments=[
                '-world',
                world_name,
                '-file',
                model_file,
                '-name',
                model_name,
                '-x',
                x,
                '-y',
                y,
                '-z',
                z,
                '-Y',
                yaw,
            ],
        )
    ]


def generate_launch_description():
    # Launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for the spawn node'
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='rcjo2026_arena',
        description='Gazebo world name'
    )
    declare_enable_teleop_cmd = DeclareLaunchArgument(
        'enable_teleop',
        default_value='false',
        description='Launch human teleop stack together with the spawned human'
    )
    declare_device_cmd = DeclareLaunchArgument(
        'device',
        default_value='keyboard',
        description='Input device type: keyboard, ps4, ps5, quest'
    )
    declare_model_name_cmd = DeclareLaunchArgument(
        'model_name',
        default_value='gz_human',
        description='Spawned human model name'
    )
    declare_human_model_cmd = DeclareLaunchArgument(
        'human_model',
        default_value='custom_human',
        description='Human model preset: person_standing, walking_actor, or custom_human'
    )
    declare_human_pose_cmd = DeclareLaunchArgument(
        'human_pose',
        default_value='cross_arms',
        description='Custom human pose preset. Supported: raise_right_hand, raise_left_hand, raise_both_hands, cross_arms, lie_down, sit_on_chair, point_right_hand, point_left_hand, crouch. Custom poses can be defined in human_pose_presets.yaml'
    )
    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value='',
        description='Optional explicit SDF file path. If set, it overrides human_model.'
    )
    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value='-2.0',
        description='Spawn x position'
    )
    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value='1.5',
        description='Spawn y position'
    )
    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Spawn z position'
    )
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Spawn yaw angle'
    )

    # LaunchConfiguration handles runtime values
    namespace = LaunchConfiguration('namespace')
    device = LaunchConfiguration('device')
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Bridge Gazebo set_pose service
    human_set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='human_set_pose_bridge',
        namespace=namespace,
        output='screen',
        arguments=[
            [
                '/world/',
                world_name,
                '/set_pose',
                '@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            ]
        ],
    )

    human_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobits_teleop'),
                'launch',
                'gz_human_teleop.launch.py',
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'device': device,
            'world_name': world_name,
            'model_name': model_name,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_teleop'))
    )


    return LaunchDescription([
        declare_namespace_cmd,
        declare_world_name_cmd,
        declare_enable_teleop_cmd,
        declare_device_cmd,
        declare_model_name_cmd,
        declare_human_model_cmd,
        declare_human_pose_cmd,
        declare_model_file_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        human_set_pose_bridge,
        OpaqueFunction(function=_spawn_human_cmd),
        human_teleop,
    ])
