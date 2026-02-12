import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('cansat_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    model_path = os.path.join(pkg_path, 'models', 'cansat', 'modelrigid.sdf')
    
    world_file_path = "/sim_ws/src/cansat/worlds/cansat.sdf" 
    world_file_path = os.path.join(pkg_path, 'worlds', 'cansat.sdf')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'rviz.rviz')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --params-file ' + os.path.join(pkg_path, 'config', 'gazebo_params.yaml') if os.path.exists(os.path.join(pkg_path, 'config', 'gazebo_params.yaml')) else '--ros-args -s libgazebo_ros_init.so -s libgazebo_ros_factory.so',
            'pause': 'true'
            
        }.items(),
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cansat_model',  
            '-file', model_path,        
            '-z', '0.0',                
            '-x', '0.0',
            '-y', '0.0'
        ],
        output='screen',
    )
    
    with open(model_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )
        
    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )



    return LaunchDescription([
        gz_server,
        gz_client,
        spawn_model,
        robot_state_publisher_node,
        rqt_image_view_node
    ])