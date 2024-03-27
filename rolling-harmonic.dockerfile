FROM ubuntu:jammy

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
    nano \
    build-essential \
    cmake \
    cppcheck \
    curl \
    git \
    gnupg \
    libeigen3-dev \
    libgles2-mesa-dev \
    lsb-release \
    pkg-config \
    protobuf-compiler \
    python3-dbg \
    python3-pip \
    python3-venv \
    qtbase5-dev \
    ruby \
    software-properties-common \
    sudo \
    wget \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Set User and give add as sudo user
RUN addgroup --gid 1000 ioes-docker \
    && adduser --uid 1000 --ingroup ioes-docker --home /home/ioes-docker --shell /bin/sh --disabled-password --gecos '' ioes-docker
RUN adduser ioes-docker sudo
RUN echo 'ioes-docker ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Color bash terminal
ENV TERM=xterm-256color
RUN echo "PS1='\[\e[01;36m\]\u\[\e[01;37m\]@\[\e[01;33m\]\H\[\e[01;37m\]:\[\e[01;32m\]\w\[\e[01;37m\]\$\[\033[0;37m\] '" >> /home/ioes-docker/.bashrc

# ------ ROS 2 설치 ------ #

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN cp /etc/apt/trusted.gpg /etc/apt/trusted.gpg.d

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO rolling

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
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

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-ros-core=0.10.0-2* \
    ros-rolling-ros-base=0.10.0-2* \
    ros-rolling-perception=0.10.0-2* \
    && rm -rf /var/lib/apt/lists/*

# Turtlesim package install
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-turtlesim \
    && rm -rf /var/lib/apt/lists/*

# Source rolling
RUN echo "source /opt/ros/rolling/setup.bash" >> /home/ioes-docker/.bashrc

# -------- Gazebo 설치 -------- #
# gazebo - harmonic install
RUN	wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --------- ROS-GZ 컴파일 테스트 --------- #
RUN mkdir -p /home/ioes-docker/ros_gz_ws/src
WORKDIR /home/ioes-docker/ros_gz_ws/src
RUN git clone https://github.com/gazebosim/ros_gz.git -b ros2
WORKDIR /home/ioes-docker/ros_gz_ws
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
COPY ros_entrypoint.sh /home/ros_entrypoint.sh
RUN  chmod +x /home/ros_entrypoint.sh
ENTRYPOINT ["/home/ros_entrypoint.sh"]
USER ioes-docker:ioes-docker
CMD ["/bin/bash"]