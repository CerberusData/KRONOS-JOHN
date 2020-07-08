# -----------------------------------------------------------------------------
# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# -----------------------------------------------------------------------------
FROM ubuntu:bionic

# Or your actual UID, GID on Linux if not the default 1000
ARG USERNAME=ada
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-utils \
    dialog \
    iproute2 \
    procps \
    curl \
    gnupg \
    python3 \
    build-essential \
    git \
    jq \
    sudo \
    vim \
    tar \
    bzip2 \
    lbzip2 \ 
    wget \ 
    cmake \ 
    openssh-client && \
    # Install Docker CE CLI
    apt-get install -y apt-transport-https ca-certificates curl gnupg-agent software-properties-common lsb-release && \
    curl -fsSL https://download.docker.com/linux/$(lsb_release -is | tr '[:upper:]' '[:lower:]')/gpg | (OUT=$(apt-key add - 2>&1) || echo $OUT) && \
    add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/$(lsb_release -is | tr '[:upper:]' '[:lower:]') $(lsb_release -cs) stable" && \
    apt-get update && \
    apt-get install -y docker-ce-cli  && \
    # Install Docker Compose
    curl -sSL "https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose && \
    chmod +x /usr/local/bin/docker-compose && \
    # Install Node
    curl -sL https://deb.nodesource.com/setup_12.x | bash - && \
    apt-get install -y nodejs && \
    # Create a non-root user to use if preferred - see 
    # https://aka.ms/vscode-remote/containers/non-root-user.
    groupadd --gid $USER_GID $USERNAME  && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    # [Optional] Add sudo support for non-root user
    apt-get install -y sudo && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    #
    # Clean up
    apt-get autoremove -y && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

RUN npm install balena-cli -g --production --unsafe-perm && \
    curl -L https://raw.githubusercontent.com/balena-io/balena-cli/master/balena-completion.bash -o /etc/bash_completion.d/balena-completion.bash

# -----------------------------------------------------------------------------
# Install and upgrade pip3
RUN \
    apt-get update && \
    apt-get -y install python3-pip && \
    pip3 install -U pip testresources \
    setuptools

# -----------------------------------------------------------------------------
# Installing GStreaming
# https://developer.download.nvidia.com/embedded/L4T/r32_Release_v1.0/Docs/
# Accelerated_GStreamer_User_Guide.pdf?cYoROY2BkiZ8J2ngNzXuQ7y63YLmK6ua_gy3K
# fRsgIIZkVSd4ZG31nlTlQ2zbwqlBze2rgRmJRkazxM7Eqdq17xRnRF309ua3GTFufz3sRx2Pnu
# TGXknJ0pnYRpVUn0jlS_OGEK5my4Xj7Ma8L5kCowDsZ_6_UcqkuzDJ3VeWVuveH1npLs
# RUN \
#     apt-get update && \
#     apt-get install -y  \
#     gstreamer1.0-tools gstreamer1.0-alsa \
#     gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
#     gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
#     gstreamer1.0-libav && \
#     apt-get install -y \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev \
#     libgstreamer-plugins-good1.0-dev \
#     libgstreamer-plugins-bad1.0-dev \
#     xterm
# ENV DISPLAY=:0

# -----------------------------------------------------------------------------
# Installing ros2
# https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/
ENV ROS_DISTRO=dashing

# Editors and so on
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
RUN \ 
    apt update && \ 
    apt-get install -y curl \ 
    gnupg2 \ 
    lsb-release && \ 
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update && \ 
    # ROS-Base Install (Bare Bones): Communication libraries, message packages, 
    # command line tools. No GUI tools.
    apt-get install -y ros-dashing-ros-base \ 
    python3-colcon-common-extensions \ 
    # ROS 2 command line tools use argcomplete to autocompletion
    python3-argcomplete

# Rosedep
RUN \ 
    pip3 install rosdep && \ 
    rosdep init && rosdep update

# Logging variables
# https://index.ros.org/doc/ros2/Tutorials/Logging-and-logger-configuration/
# # If you would like more or less verbose formatting
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{time}][{name}]: {message}"
# If you would like to force enabling or disabling it
# the output is colorized when it’s targeting a terminal. 
ENV RCUTILS_COLORIZED_OUTPUT=1                                   
# By default, info and debug log calls aren’t line buffered. 
# You can force it using 
ENV RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1   

# -----------------------------------------------------------------------------
# OpenCV Installation with contrib
# https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/
ENV OPENCV_VERSION=3.3.0
ENV OPENCV_CONTRIB_VERSION=3.3.0
ENV OPENCV_LOG_LEVEL=ERROR
RUN  \ 
    apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \ 
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran  \
    openexr \
    libatlas-base-dev \
    # python3-dev \
    python3-numpy \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
    udev
RUN \
    mkdir ~/opencv_build && \
    cd ~/opencv_build && \
    git clone -q --branch $OPENCV_VERSION https://github.com/opencv/opencv.git && \
    git clone -q --branch $OPENCV_CONTRIB_VERSION https://github.com/opencv/opencv_contrib.git && \
    cd ~/opencv_build/opencv && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D WITH_CUDA=OFF \
    -D WITH_CUBLAS=OFF \ 
    -D WITH_V4L=ON \    
    -D ENABLE_FAST_MATH=1 \        
    -D CUDA_FAST_MATH=0 \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j 4 && \
    make install && \
    rm -r ~/opencv_build

# -----------------------------------------------------------------------------
# for cv_bridge package
RUN  \
    apt-get install -y cmake \ 
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev  \
    can-utils

RUN export PATH="${PATH}:/home/ada/.local/bin/rosdep-source" && \
    export PATH="${PATH}:/home/ada/.local/bin"

# http://wiki.ros.org/docker/Tutorials/GUI
RUN  \
    sudo apt install -y \
    libcanberra-gtk-module \
    libcanberra-gtk3-module

# -----------------------------------------------------------------------------
# Add Python dependencies in here
RUN \
    python3 -m pip install pyserial \
    socketIO-client3

# -----------------------------------------------------------------------------
# Install stuff of freedoom roboticsp
# https://docs.freedomrobotics.ai/docs/install
# USER ada 
RUN \
    curl -sSf "https://api.freedomrobotics.ai/accounts/INSTALL_ONLY/devices/GENERIC_DEVICE/installscript?mc_token=INSTALL_ONLY_DEVICE_TOKEN&install_elements=no_ssh,no_credentials,webrtc,service_none,ros2" | python3 

# -----------------------------------------------------------------------------
# For wifi configuration and settings
ENV DBUS_SYSTEM_BUS_ADDRESS=unix:path=/host/run/dbus/system_bus_socket
RUN  \
    apt-get install -y network-manager

# -----------------------------------------------------------------------------
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=

CMD ["/bin/bash"]