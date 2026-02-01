import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 기본 설정
    pkg_path = get_package_share_directory('cansat_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    model_path = os.path.join(pkg_path, 'models', 'cansat', 'model.sdf')
    #world_file_path = os.path.join(pkg_path, 'worlds', 'cansat_world.sdf')
    #world_file_path = "/home/user/sim_ws/src/cansat/worlds/cansat_world.sdf"
    world_file_path = "/sim_ws/src/cansat/worlds/cansat_world.sdf"
    if os.path.exists(world_file_path):
        print(f"[INFO] [FILE] World file found at {world_file_path}")
    # 2. Gazebo World Launch
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file_path} -v 4 --render-engine ogre2'}.items(),
    )
    
    
    
    
    
    # 3. CanSat 모델 스폰 (Spawn)
    # GZ_SIM_RESOURCE_PATH에 등록된 'cansat' 모델명을 사용합니다.
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'models',
            '-file', model_path, # model.config에 정의된 이름
            '-z', '50.0'       # 시작 고도 10m
        ],
        output='screen',
    )

    # 4. ROS-Gazebo Bridge (토픽 연결)
    # IMU 데이터와 외부 힘(Wrench) 입력을 위한 브리지
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/cansat_world/model/cansat_model/link/body/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/cansat_model/joint/parafoil_joint/cmd_force@geometry_msgs/msg/Wrench]gz.msgs.Wrench'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_model,
        bridge
    ])