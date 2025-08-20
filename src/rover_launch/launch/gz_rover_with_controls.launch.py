# rover_launch/launch/gz_rover.launch.py
import shutil
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Paths to world, URDF, and configuration files.
    world_path = PathJoinSubstitution([FindPackageShare('rover_description'), 'worlds', 'empty_local.sdf'])
    urdf_path  = PathJoinSubstitution([FindPackageShare('rover_description'), 'urdf', 'rover_control.urdf'])
    control_yaml = PathJoinSubstitution([FindPackageShare('rover_launch'), 'config', 'rover_ros2_control.yaml'])

    # xacro to string
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    ign_bin = shutil.which('ign')
    if not ign_bin:
        raise RuntimeError('Ignition Fortress CLI "ign" not found. Install ignition-fortress.')
    
    # Setting the env variable.
    set_plugin_path = SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib:')

    # Running the gazebo simulation.
    gazebo = ExecuteProcess(cmd=[ign_bin, 'gazebo', world_path, '-r', '-v', '3'], output='screen')

    # Robot State Publisher and Spawner Nodes
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}], output='screen'
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'rover', '-topic', '/robot_description', '-z', '0.03'], output='screen'
    )

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
               output='screen')
    diff = Node(package='controller_manager', executable='spawner',
               arguments=['diff_drive_controller', '--controller-manager', '/controller_manager', '--param-file', control_yaml],
                output='screen')


    return LaunchDescription([gazebo, rsp, spawn, TimerAction(period=2.0, actions=[jsb, diff])])
