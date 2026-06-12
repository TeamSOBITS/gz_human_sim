from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	declare_world_name_cmd = DeclareLaunchArgument(
		'world_name',
		default_value='rcjo2026_arena',
		description='Gazebo world name'
	)
	declare_model_name_cmd = DeclareLaunchArgument(
		'model_name',
		default_value='gz_human',
		description='Default model name for spawned entity'
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
	declare_refresh_cmd = DeclareLaunchArgument(
		'refresh_ms',
		default_value='1000',
		description='Model list refresh interval (ms)'
	)

	world_name = LaunchConfiguration('world_name')
	model_name = LaunchConfiguration('model_name')
	x = LaunchConfiguration('x')
	y = LaunchConfiguration('y')
	z = LaunchConfiguration('z')
	yaw = LaunchConfiguration('yaw')
	refresh_ms = LaunchConfiguration('refresh_ms')

	models_root = PathJoinSubstitution([
		FindPackageShare('gz_human_sim'),
		'models',
	])

	gui_script = PathJoinSubstitution([
		FindPackageShare('gz_human_sim'),
		'scripts',
		'spawn_human_gui.py',
	])

	create_remove_bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		output='screen',
		arguments=[
			[
				'/world/',
				world_name,
				'/create',
				'@ros_gz_interfaces/srv/SpawnEntity',
			],
			[
				'/world/',
				world_name,
				'/remove',
				'@ros_gz_interfaces/srv/DeleteEntity',
			],
		],
	)

	gui_process = ExecuteProcess(
		cmd=[
			'python3',
			gui_script,
			'--models-root',
			models_root,
			'--world-name',
			world_name,
			'--model-name',
			model_name,
			'--x',
			x,
			'--y',
			y,
			'--z',
			z,
			'--yaw',
			yaw,
			'--refresh-ms',
			refresh_ms,
		],
		output='screen',
	)

	return LaunchDescription([
		declare_world_name_cmd,
		declare_model_name_cmd,
		declare_x_cmd,
		declare_y_cmd,
		declare_z_cmd,
		declare_yaw_cmd,
		declare_refresh_cmd,
		create_remove_bridge,
		gui_process,
	])
