# -----------------------------------------------------------------------------
# NOTE: PLEASE VISIT THE NEXT NOTION TO DECLARE YOUR VARIABLES:
#   https://www.notion.so/kiwi/Defining-Environment-Variables-729fdef098a44b4e8e4c7cfef9a47abe
#
# DO NOT forget group your variables to keep order, format should be:
#   export PACKGE/FUNCTION_VARIABLE-NAME=VALUE  # [type] values: short description
# Example:
#   export GUI_SENSORS_DISTANCE=1 # [int-bolean](1):Enable/(0):Disable - Distance sensors

# -----------------------------------------------------------------------------
# DO NOT DEFINE IN ROBOT - DO NOT DEFINE IN ROBOT - DO NOT DEFINE IN ROBOT
export ROBOBOT_ID=666      # [int] Robot ID
                           # NOTE: Should be undefined in the robot
export LOCAL_LAUNCH=1      # [int-bolean] Disable(1)/Enable(0) - Local launch mode (For pc launch)
                           # NOTE: Should be zero in the robot
export CONF_PATH="${PWD%}/configs"

# -----------------------------------------------------------------------------
# Video streaming variables

# -----------------------------------------------------------------------------
# ROS2 Logging variables 
# https://docs.python.org/3/library/logging.html
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{time}]: {message}" 
                                                # If you would like more or less verbose formatting
export RCUTILS_COLORIZED_OUTPUT=1               # the output is colorized when it’s targeting a terminal. 
                                                # If you would like to force enabling or disabling it
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1   # By default, info and debug log calls aren’t line buffered. 
                                                # You can force it using 

# -----------------------------------------------------------------------------
# Fredoom robotics enviroment variables
export FR_AGENT=0                   # [int-bolean] (1):Enable/(0):Disable freedom robotics agent 
export FR_STREAMING_IDLE_TIME=10    # [int-bolean] (1):Enable/(0):Disable freedom video streaming idle time 
export FR_STREAMING_FACTOR=0.2      # [int-bolean] (1):Enable/(0):Disable freedom video streaming scaling factor when robot is operative 
export FR_STREAMING_IDLE_FACTOR=0.4 # [int-bolean] (1):Enable/(0):Disable freedom video streaming scaling factor when robot is in standby 
export FR_STREAMING_OPTIMIZER=1     # [int-bolean] (1):Enable/(0):Disable freedom video streaming optimizer
