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

# -------- ROS Installation -------- #
# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-ros-core=0.10.0-2* \
    ros-rolling-ros-base=0.10.0-2* \
    ros-rolling-perception=0.10.0-2* \
    && rm -rf /var/lib/apt/lists/*

# Turtlesim 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-turtlesim x11-apps \
    && rm -rf /var/lib/apt/lists/*

# # -------- Gazebo Installation -------- #
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS_DISTRO}-ros-gz

# setup entrypoint
# COPY ./ros_entrypoint.sh /

# ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]