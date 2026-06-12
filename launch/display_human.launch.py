from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui', default='True')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    package_name = "gz_human_sim"
    urdf_relative_path = 'models/human_gazebo_raised_hand/humanSubjectWithMesh.urdf'

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        name='use_gui', default_value='True', choices=['True', 'False'],
        description='Flag to enable joint_state_publisher_gui'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('gz_human_sim'),
            'rviz',
            'display_human.rviz'
        ]),
        description='RViz config file for human model display'
    ))

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time', default_value='False', choices=['True', 'False'],
        description='Use simulation time if available'
    ))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': package_name,
            'urdf_package_path': PathJoinSubstitution([urdf_relative_path])
        }.items()
    ))

    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_gui)
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_gui)
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config]
    ))
    return ld