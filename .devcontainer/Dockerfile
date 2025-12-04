# Base image: ROS 2 Foxy with desktop tools
FROM osrf/ros:foxy-desktop

# Arguments for creating a non-root user
ARG USERNAME=amr
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user and give sudo access
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# System upgrade + basic utilities
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        python3-pip \
        git \
        wget

# Install ROS 2 development tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-tk \
    python3.8-venv \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true

# Setup ROS 2 environment
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc


ENV SHELL=/bin/bash

# NVIDIA GPU support
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Workspace
WORKDIR /workspace

# Switch to non-root user
USER $USERNAME

CMD ["/bin/bash"]

