# -----------------------------------------------------------------------------
# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# -----------------------------------------------------------------------------
FROM ubuntu:18.04

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
    python \
    build-essential \
    git \
    jq \
    curl \
    sudo \
    vim \
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
RUN \
    apt-get update && \ 
    apt-get -y install python3-pip 

# -----------------------------------------------------------------------------
# Installing ros2
# https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/
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

# ROS 2
RUN \ 
    pip3 install rosdep && \ 
    rosdep init && rosdep update
    
# Logging variables
# https://index.ros.org/doc/ros2/Tutorials/Logging-and-logger-configuration/
# If you would like more or less verbose formatting
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
ENV OPENCV_VERSION=4.2.0
ENV OPENCV_CONTRIB_VERSION=4.2.0
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
    python3-dev \
    python3-numpy \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev
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
# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=

CMD ["/bin/bash"]