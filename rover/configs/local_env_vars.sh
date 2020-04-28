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
export LOCAL_GUI=0         # [int-bolean] Disable(1)/Enable(0) - Local GUI launch
                           # NOTE: Should be zero in the robot
export CONF_PATH="${PWD%}/configs" # [string] absolute path to config files

# -----------------------------------------------------------------------------
# Nodes launching
export NODE_VIDEO_MAPPING=1     # [int-bolean] (1):Enable/(0):Disable video mapping node launching
export NODE_VIDEO_CALIBRATION=1 # [int-bolean] (1):Enable/(0):Disable video calibration node launching
export NODE_VIDEO_PARTICLE=1    # [int-bolean] (1):Enable/(0):Disable video particle node launching

# -----------------------------------------------------------------------------
# Video processing
export VIDEO_WIDTH=640          # [int][pix] Cameras Video width
export VIDEO_HEIGHT=360         # [int][pix] Cameras Video height
export VISUAL_DEBUGGER=1        # [int-bolean] (1):Enable/(0):Disable visual debugger messages 
export VISUAL_DEBUGGER_TIME=10  # [int][sec] visual debugger message time

# -----------------------------------------------------------------------------
# Stitcher
export STITCHER=0               # [int-bolean](1):Enable/(0):Disable - Video Stitching
export STITCHER_SUP_MODE=0      # [int-bolean](1):Enable/(0):Disable - smooth transitions in stitched image

# -----------------------------------------------------------------------------
# Extrinsic calibration - Mono-vision
export VISION_CAL_SHOW_LOCAL=0          # NOTE: Should be undefined in the robot
                                        # [int-bolean] (1):Enable/(0):Disable show calibration when runing in local

export VISION_CAL_DAYS_OUT=10           # [int][days] Number of days for a calibration out of date
export VISION_CAL_DAYS_REMOVE=20        # [int][days] Number of days to remove a calibration file
export VISION_CAL_UNWARPED_WIDTH=200    # [int][pix] width of unwarped image for monovision
export VISION_CAL_UNWARPED_HEIGHT=360   # [int][pix] height of unwarped image for monovision
export VISION_CAL_TRIES=5               # [int] Number of tries to calibrate a camera
export VISION_CAL_PAT_HOZ=0.6223        # [float][m] Horizontal distance between lines in calibration pattern
export VISION_CAL_PAT_VER=4.0           # [float][m] Vertical length of calibration pattern
export VISION_CAL_PAT_TH_TOP=15         # [int][pix] Number of pixel to project pattern from vanishing point
export VISION_CAL_PAT_TH_BOTTOM=15      # [int][pix] Number of pixel to project pattern from bottom side
export VISION_CAL_PAT_ITE_TRIES=20      # [int] number of tries to change color values to find calibration pattern
export VISION_CAL_PAT_HS=124    # [int][0-255] Hue channel superior value for color filtering in pattern search
export VISION_CAL_PAT_SI=0      # [int][0-255] saturation channel inferior value for color filtering in pattern search
export VISION_CAL_PAT_SS=255    # [int][0-255] saturation channel superior value for color filtering in pattern search
export VISION_CAL_PAT_VI=115    # [int][0-255] value channel inferior value for color filtering in pattern search
export VISION_CAL_PAT_VS=255    # [int][0-255] value channel superior value for color filtering in pattern search
export VISION_CAL_PAT_HI=58     # [int][0-255] Hue channel inferior value for color filtering in pattern search
export VISION_CAL_SHOW_TIME=5   # [int][sec] time to show calibration results on supervisors console

# -----------------------------------------------------------------------------
# ROS2 Logging 
# https://docs.python.org/3/library/logging.html
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}][{time}]: {message}" 
                                                # If you would like more or less verbose formatting
export RCUTILS_COLORIZED_OUTPUT=1               # the output is colorized when it’s targeting a terminal. 
                                                # If you would like to force enabling or disabling it
export RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED=1   # By default, info and debug log calls aren’t line buffered. 
                                                # You can force it using 

# -----------------------------------------------------------------------------
# Fredoom robotics enviroment
export FR_AGENT=1                   # [int-bolean] (1):Enable/(0):Disable freedom robotics agent 
export FR_STREAMING_OPTIMIZER=1     # [int-bolean] (1):Enable/(0):Disable freedom video streaming optimizer
export FR_STREAMING_FACTOR=0.4      # [int-bolean] (1):Enable/(0):Disable freedom video streaming scaling factor when robot is operative
export FR_STREAMING_IDLE_TIME=120   # [int-bolean] (1):Enable/(0):Disable freedom video streaming idle time 
export FR_STREAMING_IDLE_FACTOR=0.2 # [int-bolean] (1):Enable/(0):Disable freedom video streaming scaling factor when robot is in standby 

# -----------------------------------------------------------------------------

