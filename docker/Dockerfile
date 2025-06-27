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
        vim \
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

# Install some ROS 2 dependencies to create a cache layer
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-rviz2 \
        ros-humble-xacro \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

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

# switch DDS implementation to zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN sudo apt-get update \
    && sudo apt-get install -y ros-$ROS_DISTRO-rmw-zenoh-cpp \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

ARG IDE_TYPE
# make workspace dir and ide-specific dir. note that .vscode work for both vscode and cursor
RUN mkdir /home/${USERNAME}/.$IDE_TYPE-server \
    && mkdir -p /home/${USERNAME}/ros2_ws/.vscode \
    && mkdir /home/${USERNAME}/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws
