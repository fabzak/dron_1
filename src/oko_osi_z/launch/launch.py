from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

def generate_urdf_from_xacro(context, *args, **kwargs):
    xacro_path = os.path.join(get_package_share_directory('oko_osi_z'), 'urdf', 'crazyflie_with_lidars.urdf.xacro')
    urdf_out_path = '/tmp/crazyflie.urdf'
    subprocess.run(['xacro', xacro_path, '-o', urdf_out_path], check=True)

def generate_launch_description():
    package_dir = get_package_share_directory('oko_osi_z')
    world_path = os.path.join(package_dir, 'worlds', 'crazyflie_world.sdf')
    rviz_config = os.path.join(package_dir, 'rviz', 'crazyflie_lidar.rviz')
    urdf_out_path = '/tmp/crazyflie.urdf'

    return LaunchDescription([
        OpaqueFunction(function=generate_urdf_from_xacro),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(package_dir, 'models')
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-v4', world_path],
            output='screen'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'spawn_entity.py',
                '-entity', 'crazyflie',
                '-file', urdf_out_path,
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Most za bridgeanje LIDAR topika (možeš kasnije proširiti na više lidarova)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            output='screen'
        ),

        Node(
            package='oko_osi_z',
            executable='explorer_node',
            name='crazyflie_explorer',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf_out_path]
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d'],
            output='screen'
        ),
    ])
