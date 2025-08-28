import os
import shutil
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
	pkg_share = get_package_share_directory('rover_gz_teleop')
	world_path = os.path.join(pkg_share, 'worlds', 'rover_world.sdf')
	headless = LaunchConfiguration('headless')


	# Choose correct Gazebo CLI: prefer `gz sim`; fall back to `ign gazebo`
	gz_cli = shutil.which('gz')
	ign_cli = shutil.which('ign')
	if gz_cli:
		sim_cmd = ['gz', 'sim']
	elif ign_cli:
		sim_cmd = ['ign', 'gazebo']
	else:
		raise RuntimeError("Neither 'gz' nor 'ign' CLI found. Install Gazebo (Fortress/Garden/Harmonic).")


	# Run server-only in Docker/headless by default
	sim_args = sim_cmd + ['-v', '3']
	# Server-only (no GUI) works on both `gz sim` and `ign gazebo`
	sim_args += ['-r']
	sim_args += [world_path]


	sim = ExecuteProcess(cmd=sim_args, output='screen')


	bridge = Node(
	package='ros_gz_bridge',
	executable='parameter_bridge',
	name='ros_gz_bridge',
	output='screen',
	arguments=[
    '/model/rover/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    '/model/rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
	]
	)


	return LaunchDescription([
	DeclareLaunchArgument('headless', default_value='true'),
	sim,
	bridge,
	])