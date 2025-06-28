FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID
ARG USER_GID
ARG USERNAME

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# switch DDS implementation to zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-rmw-zenoh-cpp \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Dependencies for panda_ros2
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-sdformat-urdf \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-ros2controlcli \
        ros-${ROS_DISTRO}-controller-interface \
        ros-${ROS_DISTRO}-hardware-interface-testing \
        ros-${ROS_DISTRO}-ament-cmake-clang-format \
        ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-ros2-control-test-assets \
        ros-${ROS_DISTRO}-hardware-interface \
        ros-${ROS_DISTRO}-control-msgs \
        ros-${ROS_DISTRO}-backward-ros \
        ros-${ROS_DISTRO}-generate-parameter-library \
        ros-${ROS_DISTRO}-realtime-tools \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-moveit-ros-move-group \
        ros-${ROS_DISTRO}-moveit-kinematics \
        ros-${ROS_DISTRO}-moveit-planners-ompl \
        ros-${ROS_DISTRO}-moveit-ros-visualization \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-moveit-simple-controller-manager \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-xacro \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*
# Dependencies for libfranka and libfranka compilation
ARG https_proxy
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends libpoco-dev \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/frankaemika/libfranka.git ~/libfranka \
    && cd ~/libfranka \
    && git switch --detach 0.9.2 \
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF .. \
    && make -j$(nproc) \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

ARG IDE_TYPE
# make workspace dir and ide-specific dir. note that .vscode work for both vscode and cursor
RUN mkdir /home/${USERNAME}/.$IDE_TYPE-server \
    && mkdir -p /home/${USERNAME}/ros2_ws/.vscode \
    && mkdir /home/${USERNAME}/ros2_ws/src

WORKDIR /home/$USERNAME/ros2_ws
