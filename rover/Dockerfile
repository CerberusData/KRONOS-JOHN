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
    # ROS 2 command line tools use argcomplete to autocompletion
    python3-argcomplete
RUN /bin/bash -c "source /opt/ros/dashing/setup.bash"

# -----------------------------------------------------------------------------
# Add personalized dependencies in here
# RUN \
#     apt-get update && \
#     apt-get install -y \ 
#     vim && \
#     rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
COPY balena_configs/balena_start.sh .
COPY data/ /usr/src/app/data

# -----------------------------------------------------------------------------
# Start a udev service
ENV UDEV=1
WORKDIR /usr/src/app
CMD [ "bash", "balena_start.sh" ]