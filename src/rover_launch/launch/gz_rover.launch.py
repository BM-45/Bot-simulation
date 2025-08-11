from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_pkg_arg = DeclareLaunchArgument('urdf_package', default_value='rover_description')
    urdf_relpath_arg = DeclareLaunchArgument('urdf_relative_path', default_value='urdf/rover.urdf')
    name_arg = DeclareLaunchArgument('name', default_value='rover')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    use_xacro_arg = DeclareLaunchArgument('use_xacro', default_value='false')

    urdf_package = LaunchConfiguration('urdf_package')
    urdf_relative_path = LaunchConfiguration('urdf_relative_path')
    urdf_path = PathJoinSubstitution([FindPackageShare(urdf_package), urdf_relative_path])

    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x'); y = LaunchConfiguration('y'); z = LaunchConfiguration('z')
    R = LaunchConfiguration('roll'); P = LaunchConfiguration('pitch'); Y = LaunchConfiguration('yaw')
    use_xacro = LaunchConfiguration('use_xacro')

    # Build robot_description: xacro for .xacro, cat for .urdf
    robot_desc_cmd = Command([
        'bash', '-c',
        'if [ "$(echo ', use_xacro, ')" = "true" ]; then xacro ', urdf_path,
        '; else cat ', urdf_path, '; fi'
    ])

    # 1) Start Gazebo Fortress (ignition)
    ign_proc = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', 'empty.sdf'],
        output='screen'
    )

    # 2) Spawn the robot via ros_ign_gazebo
    #    (we use a small delay so the sim is ready)
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_ign_gazebo', 'create',
            '-name', name,
            '-allow_renaming', 'true',
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Y,
            '-string', robot_desc_cmd,
        ],
        output='screen'
    )

    return LaunchDescription([
        urdf_pkg_arg, urdf_relpath_arg, name_arg, x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg, use_xacro_arg,
        ign_proc,
        TimerAction(period=2.0, actions=[spawn]),
    ])

