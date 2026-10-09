FROM ros:jazzy-ros-core-noble

ARG USER_ID=robonet
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    git-lfs \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-cffi \
    vim \
    wget \
    curl \
    lsb-release \
    sudo \
    android-tools-adb \
    libglew-dev \
    patchelf \
    libosmesa6-dev \
    v4l-utils \
    keyboard-configuration \
    tzdata \
    unzip \
    ffmpeg \
    x11-apps \
    kmod \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN set -euxo pipefail; \
    apt-get update -y; \
    ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F\" '{print $4}')"; \
    CODENAME="$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})"; \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"; \
    dpkg -i /tmp/ros2-apt-source.deb; \
    rm -f /tmp/ros2-apt-source.deb; \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update -y && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-ros2-control-test-assets \
    ros-jazzy-realsense2-camera \
    ros-jazzy-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

RUN git lfs install

RUN userdel -r ubuntu 2>/dev/null || true

RUN adduser --disabled-password --gecos '' "${USER_ID}" && \
    adduser "${USER_ID}" sudo && \
    adduser "${USER_ID}" plugdev && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ${USER_ID}
WORKDIR /home/${USER_ID}

RUN mkdir -p /home/${USER_ID}/interbotix_ws/src

# Install script from our fork, patched for Jazzy / Ubuntu 24.04
RUN curl -fsSL \
      'https://raw.githubusercontent.com/montrealrobotics/interbotix_ros_manipulators/refs/heads/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' \
      -o /tmp/xsarm_amd64_install.sh && \
    chmod +x /tmp/xsarm_amd64_install.sh && \
    /tmp/xsarm_amd64_install.sh -n -d jazzy && \
    rm -f /tmp/xsarm_amd64_install.sh

ENV PYTHONPATH="${PYTHONPATH}:/home/${USER_ID}/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox"

COPY widowx_envs/requirements.txt /tmp/requirements.txt

RUN echo 'numpy<2' > /home/${USER_ID}/pip-constraints.txt
ENV PIP_CONSTRAINT=/home/${USER_ID}/pip-constraints.txt

RUN set -exo pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    source /home/${USER_ID}/interbotix_ws/install/setup.bash || true; \
    python3 -m venv --system-site-packages /home/${USER_ID}/myenv; \
    source /home/${USER_ID}/myenv/bin/activate; \
    pip install --no-cache-dir -U pip wheel; \
    pip install --no-cache-dir -r /tmp/requirements.txt

COPY --chown=${USER_ID}:${USER_ID} ./widowx_envs /home/${USER_ID}/widowx_envs
COPY --chown=${USER_ID}:${USER_ID} ./multicam_server /home/${USER_ID}/multicam_server
COPY --chown=${USER_ID}:${USER_ID} ./widowx_controller /home/${USER_ID}/widowx_controller
COPY --chown=${USER_ID}:${USER_ID} ./widowx_controller_interfaces /home/${USER_ID}/widowx_controller_interfaces

RUN ln -sf /home/${USER_ID}/widowx_envs /home/${USER_ID}/interbotix_ws/src/ && \
    ln -sf /home/${USER_ID}/multicam_server /home/${USER_ID}/interbotix_ws/src/ && \
    ln -sf /home/${USER_ID}/widowx_controller /home/${USER_ID}/interbotix_ws/src/ && \
    ln -sf /home/${USER_ID}/widowx_controller_interfaces /home/${USER_ID}/interbotix_ws/src/

ENV PYTHONPATH="${PYTHONPATH}:/home/${USER_ID}/interbotix_ws/src/widowx_envs"
ENV PYTHONPATH="${PYTHONPATH}:/home/${USER_ID}/interbotix_ws/src/widowx_controller"

RUN set -exo pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    cd /home/${USER_ID}/interbotix_ws; \
    rosdep update; \
    sudo apt-get update -y; \
    rosdep install --from-paths src --ignore-src --skip-keys "gazebo_ros2_control gazebo_ros" -r -y; \
    source /home/${USER_ID}/myenv/bin/activate; \
    rm -rf build log install; \
    colcon build --symlink-install; \
    touch /home/${USER_ID}/.built

RUN sudo ln -sf /home/${USER_ID}/widowx_envs/scripts/go_to_sleep_pose.py /usr/local/bin/go_sleep && \
    sudo ln -sf /home/${USER_ID}/widowx_envs/scripts/go_to_neutral_pose.py /usr/local/bin/go_neutral

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/${USER_ID}/.bashrc && \
    echo 'source ~/interbotix_ws/install/setup.bash' >> /home/${USER_ID}/.bashrc && \
    echo 'source ~/myenv/bin/activate' >> /home/${USER_ID}/.bashrc && \
    echo 'alias widowx_env_service="python3 ~/widowx_envs/widowx_envs/widowx_env_service.py"' >> /home/${USER_ID}/.bashrc && \
    echo 'export ROBONETV2_ARM=wx250s' >> /home/${USER_ID}/.bashrc

RUN set -exo pipefail; \
    git clone https://github.com/youliangtan/edgeml /home/${USER_ID}/edgeml; \
    source /home/${USER_ID}/myenv/bin/activate; \
    pip install --no-cache-dir -e /home/${USER_ID}/edgeml

RUN source /home/${USER_ID}/myenv/bin/activate && \
    pip install --no-cache-dir "numpy<2" --force-reinstall && \
    pip install --no-cache-dir --upgrade "coverage>=7.3,<8"
