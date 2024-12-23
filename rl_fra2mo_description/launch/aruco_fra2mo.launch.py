from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    # Percorsi dei file
    explore_path = os.path.join(
        get_package_share_directory('rl_fra2mo_description'),
        'launch',
        'fra2mo_explore.launch.py'
    )

    aruco_path = os.path.join(
        get_package_share_directory('aruco_ros'),
        'launch',
        'single.launch.py'
    )

    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Exploration node
    explore_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(explore_path)
    )

    # ArUco ros node
    aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_path),
        launch_arguments={"marker_id": "115", "marker_size": "0.1"}.items(),
    )

    # ArUco tf publisher node
    aruco_tf = Node(
        package='rl_fra2mo_description',
        executable='aruco_tf_publisher',
        name='aruco_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        explore_node,
        aruco_node,
        aruco_tf,
    ])
