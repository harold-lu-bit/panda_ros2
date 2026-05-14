FROM ros:jazzy

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID
ARG USER_GID
ARG USERNAME

# Install essential packages and ROS development tools
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
    locales \
    bash-completion \
    git \
    openssh-client \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    sudo \
    vim \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen \
  && locale-gen
ENV LANG=en_US.UTF-8 \
  LANGUAGE=en_US:en \
  LC_ALL=en_US.UTF-8

# Setup user configuration
RUN --mount=type=secret,id=sudo_password,target=/tmp/sudo_password.secret \
  deluser --remove-home ubuntu && groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
  && echo "${USERNAME}:$(cat /tmp/sudo_password.secret)" | chpasswd \
  && usermod -aG sudo,adm,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev ${USERNAME} \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
  && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc

# switch DDS implementation to zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN apt-get update \
  && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Dependencies for panda_ros2
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2controlcli \
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
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
# Dependencies for libfranka and libfranka compilation
RUN apt-get update \
  && apt-get install -y --no-install-recommends libpoco-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  && git clone https://github.com/frankaemika/libfranka.git ~/libfranka \
  && cd ~/libfranka \
  && git switch --detach 0.9.2 \
  && git submodule init \
  && git submodule update \
  && mkdir build && cd build \
  && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF .. \
  && make -j$(nproc) \
  && cpack -G DEB \
  && dpkg -i libfranka*.deb

# topic-tools for throttle topic rate
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-jazzy-topic-tools \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir /workspace && chown -R ${USERNAME}:${USERNAME} /workspace

# entry point to set xauthority for xwayland
COPY --chmod=755 inin_entrypoint.sh /inin_entrypoint.sh

USER ${USERNAME}

RUN mkdir -p /workspace/ros2_ws/src \
  && mkdir ~/.vscode-server \
  && mkdir ~/.cache
  
WORKDIR /workspace/ros2_ws
