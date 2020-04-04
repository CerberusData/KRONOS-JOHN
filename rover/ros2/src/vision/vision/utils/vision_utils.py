#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision &Ai Team

Sources:
https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
https://unix.stackexchange.com/questions/10671/usb-performance-traffic-monitor
https://www.ximea.com/support/wiki/usb3/multiple_cameras_setup
"""

# =============================================================================
import os
import inspect

# =============================================================================
class bcolors:
    LOG = {
        "WARN": ['\033[33m', "WARN"],
        "ERROR": ['\033[91m', "ERROR"],
        "OKGREEN": ['\033[32m', "INFO"],
        "INFO": ['\033[94m', "INFO"],  # ['\033[0m', "INFO"],
        "BOLD": ['\033[1m', "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = '\033[1m'
    ENDC = '\033[0m'
    HEADER = '\033[95m' 
    OKBLUE = '\033[94m'
    GRAY = "\033[90m"
    UNDERLINE = '\033[4m'
def printlog(msg, msg_type="INFO", flush=True):
    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(org, caller, msg_type, msg)
    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=flush)