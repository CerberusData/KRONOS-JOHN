# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# https://www.balena.io/docs/reference/base-images/base-images/#balena-base-images?ref=dockerhub
FROM balenalib/jetson-tx2-ubuntu:bionic

ENV DEBIAN_FRONTEND noninteractive

WORKDIR /usr/src/app

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    wget \
    bzip2 \
    lbzip2 \ 
    tar && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Script get all the packages from official nvidia packages.
# https://www.balena.io/docs/learn/welcome/production-plan/ 
COPY ./packages/download.sh .
RUN bash download.sh
RUN \
    # Nvidia dependencies
    dpkg -i cuda-repo-l4t-10-0-local-10.0.326_1.0-1_arm64.deb \
    libcudnn7_7.5.0.56-1+cuda10.0_arm64.deb \
    libcudnn7-dev_7.5.0.56-1+cuda10.0_arm64.deb && \
    sudo apt-key add /var/cuda-repo-10-0-local-10.0.326/7fa2af80.pub && \
    apt-get update && \
    apt-get install --no-install-recommends -y apt-utils && \
    apt-get install -y cuda-toolkit-10-0 cuda-libraries-dev-10-0 && \
    dpkg -i libnvinfer5_5.1.6-1+cuda10.0_arm64.deb \
    libnvinfer-dev_5.1.6-1+cuda10.0_arm64.deb && \
    # Remove packages files
    rm -rf *.deb && \
    dpkg --remove cuda-repo-l4t-10-0-local-10.0.326 && \
    dpkg -P cuda-repo-l4t-10-0-local-10.0.326

# -----------------------------------------------------------------------------
# Install Tensorflow and its dependencies
# https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html
# https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform-release-notes/tf-jetson-rel.html#tf-jetson-rel
ENV JP_VERSION=43
ENV TF_VERSION=2.0.0
RUN \
    apt-get update && \ 
    ldconfig && \ 
    # Install system packages required by TensorFlow
    apt-get -y install libhdf5-serial-dev \
    hdf5-tools \
    libhdf5-dev \
    zlib1g-dev \
    zip \
    libjpeg8-dev \
    liblapack-dev \
    libblas-dev \
    gfortran \
    python3-h5py && \
    # Install and upgrade pip3
    apt-get -y install python3-pip && \
    pip3 install -U pip testresources \
    setuptools && \
    # Install the Python package dependencies.
    pip3 install -U numpy==1.16.1 \
    future==0.17.1 \
    mock==3.0.5 \
    # h5py==2.9.0 \ 
    keras_preprocessing==1.0.5 \
    keras_applications==1.0.8 \
    gast==0.2.2 \
    futures \
    protobuf \
    pybind11 && \
    # Installing TensorFlow
    pip3 install --pre --extra-index-url \
    https://developer.download.nvidia.com/compute/redist/jp/v$JP_VERSION \
    tensorflow-gpu==$TF_VERSION

# Watch out for the libGL cp, we need a suitable version that cotains grm 
# ssymbols to develop.
RUN apt-get update && apt-get install --no-install-recommends -y \
    bzip2 \
    xorg && \
    tar xjf nvidia_drivers.tbz2 -C / && \
    tar xjf config.tbz2 -C / --exclude=etc/hosts --exclude=etc/hostname && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" > /etc/ld.so.conf.d/nvidia-tegra.conf && \
    # cp /usr/lib/aarch64-linux-gnu/tegra/libGL.so.1 /usr/lib/aarch64-linux-gnu/mesa/ && \
    ldconfig && \
    tar xvf nv_tools.tbz2 -C / --exclude=/home && \
    tar xvf nvgstapps.tbz2 -C / && \
    # bash -c "yes | gst-install --prefix=/usr --version=1.14.5" && \
    tar -C /usr/src/app --strip-components=2 -xvf nv_tools.tbz2 usr/bin/ && \
    ln -s /etc/nvpmodel/nvpmodel_t186.conf /etc/nvpmodel.conf && \
    rm *.tbz2 && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Installing GStreaming
# https://developer.download.nvidia.com/embedded/L4T/r32_Release_v1.0/Docs/
# Accelerated_GStreamer_User_Guide.pdf?cYoROY2BkiZ8J2ngNzXuQ7y63YLmK6ua_gy3K
# fRsgIIZkVSd4ZG31nlTlQ2zbwqlBze2rgRmJRkazxM7Eqdq17xRnRF309ua3GTFufz3sRx2Pnu
# TGXknJ0pnYRpVUn0jlS_OGEK5my4Xj7Ma8L5kCowDsZ_6_UcqkuzDJ3VeWVuveH1npLs
RUN \
    apt-get update && \
    apt-get install -y  \
    gstreamer1.0-tools gstreamer1.0-alsa \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav && \
    apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    xterm
ENV DISPLAY=:0

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
    # gfortran  \
    openexr \
    libatlas-base-dev \
    # python3-dev \
    # python3-numpy \
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
    -D WITH_CUDA=ON \
    -D WITH_CUBLAS=ON \ 
    -D WITH_V4L=ON \    
    -D ENABLE_FAST_MATH=1 \        
    -D CUDA_FAST_MATH=1 \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j && \
    make install && \
    rm -r ~/opencv_build

# -----------------------------------------------------------------------------
# Install stuff of freedoom robotics
# https://docs.freedomrobotics.ai/docs/install
RUN \
    curl -sSf "https://api.freedomrobotics.ai/accounts/INSTALL_ONLY/devices/GENERIC_DEVICE/installscript?mc_token=INSTALL_ONLY_DEVICE_TOKEN&install_elements=no_ssh,no_credentials,webrtc,service_none,ros2" | python3

# -----------------------------------------------------------------------------
# Add development, debugging utils/dependencies here
RUN \
    apt-get update && \
    apt-get install -y \
    htop

# -----------------------------------------------------------------------------
# Add personalized dependencies in here
# RUN \
#     apt-get update && \
#     apt-get install -y \ 
#     vim && \
#     rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Copy configuration and other file
COPY configs/freedom_robotics/ /usr/src/app/freedom_robotics/
COPY balena_configs/balena_start.sh .
COPY data/ /usr/src/app/data
COPY configs/bot_local.launch.py /usr/src/app/configs/
COPY configs/cams_conf.yaml /usr/src/app/configs/
ENV CONF_PATH="/usr/src/app/configs/"

# -----------------------------------------------------------------------------
# Creating ROS2 workspace
RUN mkdir -p /dev_ws && \
    mkdir -p /dev_ws/src
COPY ros2/src/ dev_ws/src/

# Download cv_bridge package
WORKDIR /usr/src/app/dev_ws/src
RUN  \
    git clone https://github.com/ros-perception/vision_opencv.git && \ 
    cd vision_opencv && \ 
    git checkout ros2
RUN  \
    apt-get install -y cmake \ 
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev

# Compiling ROS2 workspace
WORKDIR /usr/src/app/dev_ws
RUN  \
    . /opt/ros/dashing/setup.sh && \ 
    colcon build && \ 
    colcon test && \ 
    /bin/bash -c "source /usr/src/app/dev_ws/install/setup.bash"

# -----------------------------------------------------------------------------
# Start a udev service
ENV UDEV=1
WORKDIR /usr/src/app
CMD [ "bash", "balena_start.sh" ]

# -----------------------------------------------------------------------------


