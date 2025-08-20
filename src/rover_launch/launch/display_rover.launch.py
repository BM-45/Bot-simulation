from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Rviz Launch file for the rover.
def generate_launch_description():

    # paths for the packages.
    urdf_pkg_arg = DeclareLaunchArgument(
            'urdf_package', default_value='rover_description',
            description='Package that contains the URDF/Xacro'
            )
    urdf_relpath_arg = DeclareLaunchArgument(
            'urdf_relative_path', default_value='urdf/rover.urdf',
            description='Path to URDF/Xacro within that package'
            )

    urdf_package = LaunchConfiguration('urdf_package')
    urdf_relative_path = LaunchConfiguration('urdf_relative_path')

    urdf_path = PathJoinSubstitution([FindPackageShare(urdf_package), urdf_relative_path])

    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        urdf_pkg_arg,
        urdf_relpath_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
            ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
            ),
        Node(
            package='rviz2',
            executable='rviz2'
            ),
        ])

