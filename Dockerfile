FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ENV LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=America/Los_Angeles
ARG USER_ID=robonet
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-pip \
    python3-dev \
    vim \
    wget \
    curl \
    lsb-release \
    sudo \
    android-tools-adb \
    libglew-dev \
    patchelf \
    libosmesa6-dev \
    python3-venv \
    python3-cffi \
    v4l-utils \
    keyboard-configuration \
    tzdata \
    unzip \
    ffmpeg \
    x11-apps \
    kmod \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && apt-get install -y \
      ros-humble-ros-base \
      ros-humble-realsense2-camera \
      ros-humble-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt-get install git-lfs
RUN git lfs install

RUN adduser --disabled-password --gecos '' ${USER_ID} && \
    adduser ${USER_ID} sudo && \
    adduser ${USER_ID} plugdev && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ${USER_ID}
WORKDIR /home/${USER_ID}

RUN mkdir -p ~/interbotix_ws/src

RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/refs/heads/humble/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' \
    > /tmp/xsarm_amd64_install.sh \
    && chmod +x /tmp/xsarm_amd64_install.sh \
    && "yes" | /tmp/xsarm_amd64_install.sh \
    && rm /tmp/xsarm_amd64_install.sh
ENV PYTHONPATH="${PYTHONPATH}:~/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox"

# Install Python dependencies
COPY widowx_envs/requirements.txt /tmp/requirements.txt
RUN source /opt/ros/humble/setup.bash && source ~/interbotix_ws/install/setup.bash && python3 -m venv --system-site-packages ~/myenv
RUN source ~/myenv/bin/activate && pip install wheel && pip install -r /tmp/requirements.txt

COPY --chown=${USER_ID}:${USER_ID} ./widowx_envs /home/${USER_ID}/widowx_envs
COPY --chown=${USER_ID}:${USER_ID} ./multicam_server /home/${USER_ID}/multicam_server
COPY --chown=${USER_ID}:${USER_ID} ./widowx_controller /home/${USER_ID}/widowx_controller
COPY --chown=${USER_ID}:${USER_ID} ./widowx_controller_interfaces /home/${USER_ID}/widowx_controller_interfaces
RUN ln -s ~/widowx_envs ~/interbotix_ws/src/
RUN ln -s ~/multicam_server ~/interbotix_ws/src/
RUN ln -s ~/widowx_controller ~/interbotix_ws/src/
RUN ln -s ~/widowx_controller_interfaces ~/interbotix_ws/src/
ENV PYTHONPATH="${PYTHONPATH}:/home/${USER_ID}/interbotix_ws/src/widowx_envs"
ENV PYTHONPATH="${PYTHONPATH}:/home/${USER_ID}/interbotix_ws/src/widowx_controller"

RUN source /opt/ros/humble/setup.bash && \
    cd ~/interbotix_ws && \
    rosdep update && \
    sudo apt update && \
    rosdep install --from-paths src --ignore-src --skip-keys "gazebo_ros2_control gazebo_ros" -r -y && \
    source ~/myenv/bin/activate && \
    rm -rf build log install && \
    colcon build --symlink-install && \
    touch ~/.built

RUN sudo ln -s ~/widowx_envs/scripts/go_to_sleep_pose.py /usr/local/bin/go_sleep && \
    sudo ln -s ~/widowx_envs/scripts/go_to_neutral_pose.py /usr/local/bin/go_neutral

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source ~/interbotix_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'source ~/myenv/bin/activate' >> ~/.bashrc

RUN git clone https://github.com/youliangtan/edgeml && \
    cd edgeml && \
    source ~/myenv/bin/activate && \
    pip3 install -e .

RUN source ~/myenv/bin/activate && pip install "numpy<2" --force-reinstall
RUN source ~/myenv/bin/activate && pip install --upgrade "coverage>=7.3,<8"

RUN echo 'alias widowx_env_service="python3 ~/widowx_envs/widowx_envs/widowx_env_service.py"' >> ~/.bashrc
RUN echo "export ROBONETV2_ARM=wx250s" >> ~/.bashrc && source ~/.bashrc

