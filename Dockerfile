# Dockerfile
FROM osrf/ros:humble-desktop-full

# 1. 필수 유틸리티 및 Gazebo 관련 패키지 설치
RUN apt-get update && apt-get install -y \
    git \
    vim \
    python3-pip \
    libeigen3-dev \
    python3-colcon-common-extensions \
    ros-humble-ros-gz \
    ros-humble-xacro \
    terminator \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

# 2. 작업 공간 설정
WORKDIR '/sim_ws'

# 3. 의존성 설치 및 빌드를 위한 초기화 (소스는 docker-compose에서 마운트)
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source install/local_setup.bash" >> ~/.bashrc
