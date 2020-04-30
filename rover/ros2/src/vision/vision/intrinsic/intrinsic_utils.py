# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np
import yaml
import os

from vision.utils.vision_utils import printlog

# =============================================================================
def read_intrinsic_params(CONF_PATH, FILE_NAME):
    """ 
        Loads intrinsic camera parameters from file  
    Args:
        file_path: `string` absolute path to yaml file
    Returns:
        file_path: `dict` intrinsic camera configuration
            dictionary
    """

    try:
        abs_path = os.path.join(CONF_PATH, FILE_NAME)
        
        if os.path.isfile(abs_path):
            with open(abs_path, 'r') as stream:
                data_loaded = yaml.safe_load(stream)
        else:
            return None
        
        for key in [
            "camera_matrix", 
            "distortion_coefficients",
            "rectification_matrix",
            "projection_matrix"]:

            if key not in data_loaded:
                printlog(
                    msg="Intrinsic file {}, invalid".format(
                    FILE_NAME), msg_type="ERROR")
                return None

            data_loaded[key] = \
                np.array(data_loaded[key]["data"]).reshape(
                    data_loaded[key]["rows"], 
                    data_loaded[key]["cols"])

    except Exception as e:
        printlog(
            msg="loading instrinsic configuration file {}, {}".format(
            FILE_NAME, e), msg_type="ERROR")
        return None

    printlog(msg="{} instrinsic configuration loaded".format(
        FILE_NAME), msg_type="OKGREEN")

    return data_loaded

# =============================================================================
if __name__ == '__main__':
    
    CONF_PATH = os.path.abspath(__file__ + "/../../../../../../configs")
    VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
    VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
    FILE_NAME = "Intrinsic_{}_{}.yaml".format(VIDEO_WIDTH, VIDEO_HEIGHT)

    intrisic_params = read_intrinsic_params(
        CONF_PATH=CONF_PATH, 
        FILE_NAME=FILE_NAME)

# =============================================================================