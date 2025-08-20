from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_share = get_package_share_directory('rover_description')  
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover_diff.urdf')  
    world_path = os.path.join(pkg_share, 'worlds', 'empty_local.sdf') 

    # Start Gazebo (GUI) with world
    gz_sim = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:={world_path}'],
        output='screen'
    )

    # Spawn the URDF into Gazebo
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'simple_rover',
            '-file', urdf_path,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # bridge: cmd_vel and odometry
    bridge_cmd_vel = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/simple_rover/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    bridge_odom = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/simple_rover/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn,
        bridge_cmd_vel,
        bridge_odom
    ])
