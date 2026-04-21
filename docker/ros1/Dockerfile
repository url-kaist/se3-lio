FROM nvidia/cuda:11.8.0-base-ubuntu20.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl gnupg2 lsb-release build-essential \
    python3-pip python3-dev python3-venv \
    && add-apt-repository ppa:deadsnakes/ppa -y \
    && python3 -m pip install --no-cache-dir --upgrade pip setuptools \
    && pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update && apt-get install -y ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools \
    && rosdep init && rosdep update \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ws/standalone && cd /ws/standalone \
    && git clone https://github.com/MIT-SPARK/TEASER-plusplus.git \
    && cd TEASER-plusplus && mkdir build && cd build \
    && cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF -DBUILD_PYTHON_BINDINGS=OFF && make install -j$(($(nproc)-1)) \
    && cd /ws/standalone && git clone https://github.com/borglab/gtsam.git \
    && cd gtsam && git checkout release/4.2 && mkdir build && cd build \
    && cmake .. -DGTSAM_USE_SYSTEM_EIGEN=True && make -j$(($(nproc)-1)) && make install && cd /ws/standalone && rm -rf TEASER-plusplus gtsam

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
    && cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j$(($(nproc)-1)) && make install

RUN mkdir -p /ws/src \
    && cd /ws/src && git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
    && cp /ws/src/livox_ros_driver2/package_ROS1.xml /ws/src/livox_ros_driver2/package.xml

RUN pip uninstall netifaces -y && pip install netifaces

RUN source /opt/ros/noetic/setup.bash \
    && cd /ws \
    && catkin build --cmake-args -DROS_EDITION=ROS1

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && echo "source /ws/devel/setup.bash" >> ~/.bashrc \
    && echo 'alias cb="catkin build"' >> ~/.bashrc \
    && echo 'alias sd="source devel/setup.bash"' >> ~/.bashrc \
    && echo 'export PS1="\[\e[32m\]$(cat /proc/self/cgroup | grep docker | tr "/" "\n" | tail -1 | cut -c1-12)\[\e[34m\]:\[\e[36m\]\w\[\e[0m\]\$ "' >> ~/.bashrc
