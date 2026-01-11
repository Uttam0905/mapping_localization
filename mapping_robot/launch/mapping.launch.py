from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro



def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('mapping_robot'))
    xacro_file_bot1 = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config_bot1 = xacro.process_file(xacro_file_bot1)
    world_path = os.path.join(pkg_path, 'worlds', 'obstacle.world')
    slam_params = os.path.join(pkg_path,'config','mapper_params_online_async.yaml')

    rsp1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config_bot1.toxml(),
                    "use_sim_time": True}],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': world_path}.items()
    )

    spawn_entity1 = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                  '-entity', 'bot1',
                  '-x','0',
                  '-y','0',
                  '-z','0',
                  '-Y','0',],
                    output='screen')
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True}
        ]
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp1, spawn_entity1,
        slam_node, rviz,
    ])