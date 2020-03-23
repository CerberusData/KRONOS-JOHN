# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# https://www.balena.io/docs/reference/base-images/base-images/#balena-base-images?ref=dockerhub
FROM balenalib/jetson-tx2-ubuntu:bionic
MAINTAINER john@kiwibot.com

WORKDIR /usr/src/app

RUN apt-get update && \
    apt-get install --no-install-recommends -y wget bzip2 && \
    rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND noninteractive

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
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install cuda-toolkit-10-0 cuda-libraries-dev-10-0 -y && \
    dpkg -i libnvinfer5_5.1.6-1+cuda10.0_arm64.deb \
    libnvinfer-dev_5.1.6-1+cuda10.0_arm64.deb && \
    # Remove packages files
    rm -rf *.deb && \
    rm -rf *.tbz2 && \
    dpkg --remove cuda-repo-l4t-10-0-local-10.0.326 && \
    dpkg -P cuda-repo-l4t-10-0-local-10.0.326
    
# -----------------------------------------------------------------------------
# Install Tensorflow and its dependencies
# https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html
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
pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v43 tensorflow

# -----------------------------------------------------------------------------
# Add personalized dependencies in here
# RUN \
#     apt-get update && \
#     apt-get install -y \ 
#     vim && \
#     rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
COPY balena_configs/balena_start.sh .

# Start a udev service
ENV UDEV=1

CMD [ "bash", "balena_start.sh" ]