#!/bin/bash
# http://wiki.ros.org/docker/Tutorials/GUI

set -e

export containerId=$(docker ps -l -q)

echo  "Starting for Docker cointainer ${containerId%}"

xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`