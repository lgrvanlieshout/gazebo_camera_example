##################################################################################################
# Copied and adjusted from https://github.com/athackst/dockerfiles/blob/main/ros2/jazzy.Dockerfile
##################################################################################################

###########################################
# Base image
###########################################
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

ARG ROS_DISTRO=humble
SHELL ["/bin/bash", "-c"]

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Update packages
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 and rosdep
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-desktop \
    python3-rosdep \
  && rm -rf /var/lib/apt/lists/*

# Install other necessary programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    openssh-client \
    ros-dev-tools \
    ros-$ROS_DISTRO-ament-* \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=$ROS_DISTRO
ENV AMENT_PREFIX_PATH=/opt/ros/$ROS_DISTRO
ENV COLCON_PREFIX_PATH=/opt/ros/$ROS_DISTRO
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/opt/rviz_ogre_vendor/lib:/opt/ros/$ROS_DISTRO/lib/x86_64-linux-gnu:/opt/ros/$ROS_DISTRO/lib
ENV PATH=/opt/ros/$ROS_DISTRO/bin:$PATH
ENV PYTHONPATH=/opt/ros/$ROS_DISTRO/local/lib/python3.12/dist-packages:/opt/ros/$ROS_DISTRO/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

###########################################
#  Base + Gazebo harmonic image
###########################################
FROM base AS gazebo_harmonic

ENV DEBIAN_FRONTEND=noninteractive

# Install gazebo
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-gz \
  && rm -rf /var/lib/apt/lists/*

# Setup the workspace
RUN mkdir -p /simulation/src
WORKDIR /simulation

# Install dependencies
RUN rosdep init || echo "rosdep already initialized" \
  && rosdep update --rosdistro=$ROS_DISTRO \
  && apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

ENV DEBIAN_FRONTEND=

###########################################
#  Develop image for gazebo harmonic
###########################################
FROM gazebo_harmonic AS dev_harmonic

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Enable entrypoint
COPY ./entrypoint.sh /simulation/entrypoint.sh
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["ros2 launch camera camera.launch.py"]

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

###########################################
#  Base + Gazebo garden image
###########################################
FROM base AS gazebo_garden

ENV DEBIAN_FRONTEND=noninteractive

# Install gazebo garden
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    gz-garden \
    ros-humble-ros-gzgarden \
  && rm -rf /var/lib/apt/lists/*

RUN export GZ_VERSION=garden

# Setup the workspace
RUN mkdir -p /simulation/src
WORKDIR /simulation

# Install dependencies
RUN rosdep init || echo "rosdep already initialized" \
  && rosdep update --rosdistro=$ROS_DISTRO \
  && apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

###########################################
#  Develop image for gazebo garden
###########################################
FROM gazebo_garden AS dev_garden

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*
  
# Enable entrypoint
COPY ./entrypoint.sh /simulation/entrypoint.sh
ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
CMD ["ros2 launch camera camera.launch.py"]

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
