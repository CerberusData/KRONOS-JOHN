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
import numpy as np
import datetime
import inspect
import cv2
import os

# =============================================================================
# VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUA
class bcolors:
    LOG = {
        "WARN": ['\033[33m', "WARN"],
        "ERROR": ['\033[91m', "ERROR"],
        "OKGREEN": ['\033[32m', "INFO"],
        "INFO": ['\033[0m', "INFO"], # ['\033[94m', "INFO"], 
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
    _str = "[{}][{}][{}]: {}".format(msg_type, org, caller, msg)
    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=flush)
    
def show_local_gui(imgs_dic, win_name="LOCAL_VIDEO", show_time=True):
    """     
        Show a local video with the current cameras, deping on the configuration
        and the camera streamings given by imgs_dic the distribution of images 
        in the window will change.
        Args:
            imgs_dic: `dictionary` dictionary of images with key as the camera 
                label and value as the image streaming of that camera
            win_name: `string` name of window to create local gui window
            show_time: `boolean` show local time on images
        Returns:
    """

    if show_time:
        current_time = datetime.datetime.now().strftime("%c")
        for img in imgs_dic.values():
            cv2.putText(img, str(current_time), (15, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

    for img in imgs_dic.values(): # Draw images margins
        cv2.rectangle(img=img, pt1=(0, 0), pt2=(img.shape[1]-1, img.shape[0]-1), 
            color=(150, 150, 150), thickness=1) 

    if "C" not in imgs_dic.keys(): 
        return
    elif set(imgs_dic.keys()) == set(["C", "B", "LL", "RR", "P"]):
        stream = np.concatenate((np.concatenate((imgs_dic["C"], imgs_dic["B"]), axis=0), 
            np.concatenate((imgs_dic["LL"], imgs_dic["RR"]), axis=0)), axis=1)
        stream[int((stream.shape[0] - imgs_dic["P"].shape[0])*0.5): 
            int((stream.shape[0] - imgs_dic["P"].shape[0])*0.5) + imgs_dic["P"].shape[0],
            int((stream.shape[1] - imgs_dic["P"].shape[1])*0.5): 
            int((stream.shape[1] - imgs_dic["P"].shape[1])*0.5) + imgs_dic["P"].shape[1]] = imgs_dic["P"]
    elif set(imgs_dic.keys()) == set(["C", "LL", "RR", "P"]):
        stream = (np.concatenate((imgs_dic["LL"], imgs_dic["P"], imgs_dic["RR"]), axis=1))
    elif set(imgs_dic.keys()) == set(["C", "LL", "P"]):
        stream = (np.concatenate((imgs_dic["LL"], imgs_dic["P"]), axis=1))
    elif set(imgs_dic.keys()) == set(["C", "RR", "P"]):
            stream = (np.concatenate((imgs_dic["P"], imgs_dic["RR"]), axis=1))
    elif set(imgs_dic.keys()) == set(["P"]):
        stream = imgs_dic["P"]
    elif set(imgs_dic.keys()) == set(["C"]):
        stream = imgs_dic["C"]
    else:
        for key, value in imgs_dic.items():
            cv2.imshow(key, value) 
        key = cv2.waitKey(10) # Show video and capture key
        return
    cv2.imshow(win_name, stream) 
    key = cv2.waitKey(10) # Show video and capture key
    if key==113 or key==81: # (Q) If press q then quit
        exit()
    elif key!=-1: # No key command
        print("Command or key action no found: {}".format(key))

def print_text_list(img, tex_list, color=(0, 0, 255), orig=(10, 25), fontScale=0.7):

    y_jump = 30
    for idx, text in enumerate(tex_list):
        cv2.putText(img=img, text=text, 
            org = (orig[0], int(orig[1] + y_jump*idx)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
            fontScale=fontScale, color=(0, 0, 0), thickness=3, lineType=cv2.LINE_AA)
        cv2.putText(img=img, text=text, 
            org = (orig[0], int(orig[1] + y_jump*idx)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
            fontScale=fontScale, color=color, thickness=1, lineType=cv2.LINE_AA)

# =============================================================================
# MATH OPERATIONS - MATH OPERATIONS - MATH OPERATIONS - MATH OPERATIONS - MATH 
def flat_matrix_for_service(numpy_array):
    """ 
        Flat numpy matrix in vector
    Args:
        numpy_array: `np.array` matrix to flat
    Returns:
        _: `list` vector of matrix flatten
    """

    rows = len(numpy_array)
    cols = len(numpy_array[0])
    nelem = rows*cols
    elems = numpy_array.reshape(1, nelem)[0]

    return list(np.append([float(rows) , float(cols)], elems))

def matrix_from_flat(list_vector):
    """ 
        reshape a list vector to matrix numpy array     
    Args:
        list_vector: `list` list to convert to numpy array (matrix)
    Returns:
        _: `numpy.array` matrix of list reshaped
    """

    rows = list_vector[0]
    cols = list_vector[1]
    return np.array(list_vector[2:]).reshape(rows, cols)

# =============================================================================