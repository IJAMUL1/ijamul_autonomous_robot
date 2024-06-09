# Use an Ubuntu 22.04 base image compatible with ARM64 architecture
FROM arm64v8/ubuntu:22.04 AS base

# Set the environment variable to avoid user interaction during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install timezone and basic dependencies
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && apt-get upgrade -y \
  && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    nano \
    sudo \
    minicom \
    screen \
    software-properties-common \
    jstest-gtk \
    wget \
    libi2c-dev \
  && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 apt repository to your system
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    libserial-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=humble \
    AMENT_PREFIX_PATH=/opt/ros/humble \
    COLCON_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib \
    PATH=/opt/ros/humble/bin:$PATH \
    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2

###########################################
#  Develop image
###########################################

FROM base AS dev

# Install additional development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  vim \
  openssh-client \
  ros-dev-tools \
  ros-humble-ament-* \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized" && rosdep update

# Setup the environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# User configuration
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && usermod -a -G dialout $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Setup sudo for the user
RUN apt-get update \
  && apt-get install -y sudo \
  && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Full image
###########################################

FROM dev AS full

# Install full ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-desktop \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  && rm -rf /var/lib/apt/lists/*

###########################################
#  Full+Gazebo image
###########################################
FROM full AS gazebo

# Install Gazebo
RUN apt-get update && apt-get install -y \
  libignition-gazebo6 \
  libignition-gazebo6-dev \
  libignition-gazebo6-plugins \
  && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
