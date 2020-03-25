#! /bin/bash

# Download from nvidia packages to build GPU containers for balena
# Davidnet (david@kiwibot.com)
# JohnBetaCode (john@kiwibot.com)

# Safe bash script
set -euo pipefail 

# Use the maximum frequency of the CPU
nvpmodel -m 0

# Turn on the jetson fan
/usr/src/app/jetson_clocks.sh

# xinit &
wget https://s3.amazonaws.com/kiwibot/eyes.mp4 -nc -q
# while true ; do gst-launch-1.0 filesrc location=eyes.mp4 !  qtdemux name=demux demux.video_0 ! queue ! h264parse ! omxh264dec !  nvoverlaysink display-id=0 -e ; done

sleep infinity