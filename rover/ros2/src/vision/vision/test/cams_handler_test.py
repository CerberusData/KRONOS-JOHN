#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision &Ai Team

"""

# =============================================================================
import inspect
import sys
import os

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
sys.path.insert(0, os.path.dirname(currentdir))

from utils.cam_handler import read_cams_configuration
from utils.cam_handler import CamerasSupervisor
from utils.vision_utils import printlog

# =============================================================================
def main(args=None):

    # ---------------------------------------------------------------------
    LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=1)) 
    VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
    VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640)) 
    CONF_PATH = os.path.abspath(__file__ + "/../../../../../../configs")

    # ---------------------------------------------------------------------
    # Initiate CameraSupervisors Class that handles the threads that reads 
    # the cameras
    cams_config = read_cams_configuration(FILE_NAME="cams_conf_local.yaml" 
        if LOCAL_RUN else "cams_conf.yaml", CONF_PATH=CONF_PATH)
    if cams_config is None : # Validate cameras status
        printlog("No cameras were configured in configuration")
        exit()
    else:
        printlog("cameras configuration loaded")

    # ---------------------------------------------------------------------
    cameras_supervisor = CamerasSupervisor(cams_config=cams_config)

# =============================================================================
if __name__ == '__main__':
    main()
