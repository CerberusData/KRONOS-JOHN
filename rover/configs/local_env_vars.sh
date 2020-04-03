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
export KIWIBOT_ID=1206     # [int] Robot ID
                           # NOTE: Should be undefined in the robot
export LOCAL_LAUNCH=1      # [int-bolean] Disable(1)/Enable(0) - Local launch mode (For pc launch)
                           # NOTE: Should be zero in the robot
export CONF_PATH="${PWD%}/configs"

# -----------------------------------------------------------------------------
# Video streaming variables
export VIDEO_HEIGHT=360
export VIDEO_WIDTH=640

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
export FR_AGENT=0