
# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images_ros2/create_ros_core_image.Dockerfile.em
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu22.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO rolling

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# -------- ROS 설치 -------- #
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-ros-core=0.10.0-2* \
    ros-rolling-ros-base=0.10.0-2* \
    ros-rolling-perception=0.10.0-2* \
    && rm -rf /var/lib/apt/lists/*

# Turtlesim package install
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-turtlesim x11-apps mesa-utils\
    && rm -rf /var/lib/apt/lists/*

# -------- Gazebo 설치 -------- #
# gazebo - harmonic install
RUN apt-get update
RUN apt-get install lsb-release -y
RUN apt-get install wget -y
RUN apt-get install gnupg -y
RUN	wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN	apt-get update 
RUN	apt-get install gz-harmonic -y

# --------- ROS-GZ 설치 --------- #
RUN mkdir -p ~/ros_gz_ws/src
WORKDIR /root/ros_gz_ws/src
RUN git clone https://github.com/gazebosim/ros_gz.git -b ros2
WORKDIR /root/ros_gz_ws
RUN rosdep install -r --from-paths src -i -y --rosdistro rolling
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-actuator-msgs \
    ros-rolling-vision-msgs \
    ros-rolling-xacro \
    ros-rolling-gps-msgs \
    ros-rolling-sdformat-urdf \
    ros-rolling-rviz2 \
    ros-rolling-rqt-topic \
    ros-rolling-rqt-plot \
    ros-rolling-rqt-image-view \
    libgz-transport12-dev \
    libgz-sim7-dev \
    && rm -rf /var/lib/apt/lists/*
RUN /bin/bash -c "source /opt/ros/rolling/setup.bash && colcon build"

# setup entrypoint
COPY ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
RUN  chmod 755 /usr/local/bin/ros_entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
