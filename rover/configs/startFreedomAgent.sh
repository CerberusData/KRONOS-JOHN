set -e

#  Source ROS2 and local enviroment variables
echo  "[INFO]: ROS2 Sourcing ... "
source "${PWD%}/ros2/install/setup.bash"
source "${PWD%}/configs/local_env_vars.sh"

#  Launch freedom agent
# Define enviroment variables in fr_keys:
    # FR_ACCOUNT
    # FR_DEVICE
    # FR_SECRET
    # FR_TOKEN
source "${PWD%}/configs/keys/fr_keys.sh"

# Run Freedoom agent stuff
# Freedom Robotics services
if [ "$FR_AGENT" == "1" ]
then 
    echo "[INFO]: Launching freedom agent"
    python3 ${PWD%}/configs/freedom_robotics/inject_freedom.py &
    python3 ${PWD%}/configs/freedom_robotics/keep_alive_freedom.py 
else
    echo "[WARN]: No fredoom robotics agent configured"
fi