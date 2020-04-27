#!/bin/bash
# /etc/init.d/startBot

# -----------------------------------------------------------------------------
### BEGIN INIT INFO
# Provides:          Robot
# Required-Start:    $syslog
# Required-Stop:     $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Script to start a Robot at init
# Description:       Script to start Robot at boot time
### END INIT INFO

set -e

# -----------------------------------------------------------------------------
# If you want a command to always run, put it here
# Carry out specific functions when asked to by the system 
case "$1" in
  start)
    echo  "Starting Robot"

      #  ----------------------------------------------------------------------
      #  ROS2 cv_bridge dependency
      if [ -d "${PWD%}/ros2/src/vision_opencv" ] 
      then
          echo "cv_bridge already exits" 
          sleep 2 && clear 
      else
          cd ${PWD%}/ros2/src
          git clone https://github.com/ros-perception/vision_opencv.git
          cd vision_opencv
          git checkout ros2
          cd .. && cd .. && cd ..
          sleep 8 && clear 
      fi

      #  ----------------------------------------------------------------------
      # Delete previous workspaces
      # echo  "ROS2 Removing old shit ... "
      # rm -r ${PWD%}/ros2/install || true
      # rm -r ${PWD%}/ros2/build || true
      # rm -r ${PWD%}/ros2/log || true
      # sleep 2 && clear 
      # export ROS2_DEL_BUILD=0
      # sleep 2 && clear 

      #  ----------------------------------------------------------------------
      #  Build ROS2 packages
      . /opt/ros/dashing/setup.bash
      clear && cd ${PWD%}/ros2    
      echo  "ROS2 Building new stuff ... "
      colcon build
      echo  "ROS2 Build successful ... "
      sleep 2 && clear && cd ..
       
      #  ----------------------------------------------------------------------
      #  Source ROS2 and local enviroment variables
      echo  "ROS2 Sourcing ... "
      source "${PWD%}/ros2/install/setup.bash"
      source "${PWD%}/configs/local_env_vars.sh"

      #  ----------------------------------------------------------------------
      #  ROS2 Launching
      sleep 2 && clear
      echo  "ROS2 launching ... "
      ros2 launch "${PWD%}/configs/bot_local.launch.py"

      #  ----------------------------------------------------------------------
    ;;
  stop)
    echo "Stopping Robot"
    # kill application you want to stop
    for i in $( rosnode list ); do
      ros2 lifecycle set $i shutdown;
    done
    ;;
  *)
    echo "Usage: /etc/init.d/Robot {start|stop}"
    exit 1
    ;;
esac

exit 0
