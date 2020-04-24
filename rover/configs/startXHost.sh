#!/bin/bash

set -e

export containerId=$(docker ps -l -q)

echo  "Starting for Docker cointainer ${containerId%}"

xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $containerId`