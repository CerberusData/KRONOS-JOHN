#! /bin/bash

# Download from nvidia packages to build GPU containers for balena
# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# Safe bash script
# set -euo pipefail 

# Use the maximum frequency of the CPU
# nvpmodel -m 0 --verbose

# Turn on the jetson fan
# jetson_clocks.sh

# wget https://s3.amazonaws.com/kiwibot/eyes.mp4 -nc -q
nvgstplayer-1.0 -i /data/eyes.mp4 --svd="omxh264dec" --svs="nvoverlaysink # display-id=0" --loop-forever

# Source ROS2 and execute ROS launch
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc || true

sleep infinity 