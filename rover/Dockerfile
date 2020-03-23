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