import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Percorsi ai file
    rviz_explore_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'rviz_conf', 'explore.rviz')
    rviz_aruco_file = os.path.join(get_package_share_directory('rl_fra2mo_description'), 'rviz_conf', 'aruco_view.rviz')
    
    # Configurazione per l'uso del tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_explore = LaunchConfiguration('use_explore', default='false')
    use_slam = LaunchConfiguration('use_slam', default='false')
    


    # Nodo RViz2
    rviz_node_explore = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_explore_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition = IfCondition(use_explore),
    )

    rviz_node_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_aruco_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition = IfCondition(use_slam),
    )

    # nodes_to_start = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]
    nodes_to_start = [rviz_node_slam, rviz_node_explore]

    return LaunchDescription(nodes_to_start)
