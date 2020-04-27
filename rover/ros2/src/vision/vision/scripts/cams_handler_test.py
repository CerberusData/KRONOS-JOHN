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
import time
import sys
import cv2
import os

# uncomment if using python2 and ROS1 Kinetic
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
sys.path.insert(0, os.path.dirname(currentdir))

from utils.cam_handler import read_cams_configuration
from utils.cam_handler import CamerasSupervisor
from utils.vision_utils import show_local_gui
from utils.vision_utils import printlog

# =============================================================================
def main(args=None):

    # ---------------------------------------------------------------------
    LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=1)) 
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
    rate = max(list(map(lambda o: int(o.cam_config["FPS"]), 
        cameras_supervisor.camera_handlers.values())))
    # Get cameras status
    # print(cameras_supervisor.get_cameras_status())

    while True:
        start = time.time()

        images_dict = dict(map(lambda o: (o.cam_label, o.image.copy()), 
                cameras_supervisor.camera_handlers.values()))
        show_local_gui(images_dict)

        end = time.time()
        remain = start + 1/float(rate) - end
        if remain > 0.:
            time.sleep(remain)

# =============================================================================
if __name__ == '__main__':
    main()
