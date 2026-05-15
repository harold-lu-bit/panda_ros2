FROM osrf/ros:kilted-desktop

# setup user and timezone
ARG USER_NAME
ARG USER_UID
ARG USER_GID
ARG TZ
RUN --mount=type=secret,id=sudo_password,target=/tmp/sudo_password.secret \
  deluser --remove-home ubuntu \
  && echo ${TZ} | tee /etc/timezone \
  && ln -sf /usr/share/zoneinfo/${TZ} /etc/localtime \
  && apt-get update && apt-get install -y \
    tzdata \
    bash-completion \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  && groupadd --gid ${USER_GID} ${USER_NAME} \
  && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USER_NAME} \
  && echo "${USER_NAME}:$(cat /tmp/sudo_password.secret)" | chpasswd \
  && usermod -aG sudo ${USER_NAME} \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc \
  && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USER_NAME}/.bashrc \
  && echo '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' >> ~/.bashrc 

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

# switch DDS implementation to zenoh
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN apt-get update \
  && apt-get install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# topic-tools for throttle topic rate and other handy tools
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-topic-tools \
    vim \
    tmux \
    ripgrep \
    clang-format \
    iputils-ping \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# create workspace folder
RUN mkdir -p /workspace/ros2_ws/src \
  && chown -R ${USER_NAME}:${USER_NAME} /workspace

################ non-root user ################

USER ${USER_NAME}

# xauth with wayland on host system
RUN echo 'AUTH_FILE=$(find /tmp/runtime_dir -maxdepth 2 -type f \( -name ".mutter-Xwaylandauth.*" -o -name "*Xauthority*" \) 2>/dev/null | head -n 1); [ -n "$AUTH_FILE" ] && export XAUTHORITY="$AUTH_FILE"' >> ~/.bashrc

# handy tools
RUN git clone https://github.com/magicmonty/bash-git-prompt.git ~/.bash-git-prompt --depth=1 \
  && echo "GIT_PROMPT_ONLY_IN_REPO=1; source ~/.bash-git-prompt/gitprompt.sh" >> ~/.bashrc \
  && echo "[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

# for persitent cache
RUN mkdir ~/.vscode-server \
  && mkdir ~/.cache
  
WORKDIR /workspace/ros2_ws
