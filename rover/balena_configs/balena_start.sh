#! /bin/bash
# -----------------------------------------------------------------------------
# Download from nvidia packages to build GPU containers for balena
# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# -----------------------------------------------------------------------------
# Safe bash script
# set -euo pipefail 
clear

# -----------------------------------------------------------------------------
# Use the maximum frequency of the CPU
# nvpmodel -m 0 #--verbose

# Turn on the jetson fan
# bash /usr/src/app/jetson_clocks.sh

# -----------------------------------------------------------------------------
# Eyer/Face videos
# wget https://s3.amazonaws.com/kiwibot/eyes.mp4 -nc -q
# nvgstplayer-1.0 -i /data/eyes.mp4 --svd="omxh264dec" --svs="nvoverlaysink # display-id=0" --loop-forever

# -----------------------------------------------------------------------------
# Source ROS2 and execute ROS launch
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
echo "source /usr/src/app/dev_ws/install/setup.bash" >> ~/.bashrc
source /opt/ros/dashing/setup.bash
source /usr/src/app/dev_ws/install/setup.bash
ros2 launch /usr/src/app/configs/bot.launch.py &

sleep 10

# -----------------------------------------------------------------------------
# Run Freedoom agent stuff
# Freedom Robotics services
if [ "$FR_AGENT" == "1" ]
then 
    echo "launching freedom agent"
    python3 /usr/src/app/freedom_robotics/inject_freedom.py
    python3 /usr/src/app/freedom_robotics/keep_alive_freedom.py &
else
    echo "No fredoom robotics agent configured"
fi

# -----------------------------------------------------------------------------
sleep infinity 