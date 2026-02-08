import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('cansat_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    model_path = os.path.join(pkg_path, 'models', 'cansat', 'model.sdf')
    
    world_file_path = "/sim_ws/src/cansat/worlds/cansat_world.sdf" 
    
    if os.path.exists(world_file_path):
        print(f"[INFO] [FILE] World file found at {world_file_path}")
    else:
        world_file_path = os.path.join(pkg_path, 'worlds', 'cansat_world.sdf')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --params-file ' + os.path.join(pkg_path, 'config', 'gazebo_params.yaml') if os.path.exists(os.path.join(pkg_path, 'config', 'gazebo_params.yaml')) else '--ros-args -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
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
            '-z', '50.0'                
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_server,
        gz_client,
        spawn_model
    ])