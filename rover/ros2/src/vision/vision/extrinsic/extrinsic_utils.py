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

# =============================================================================
def read_extrinsic_params(CONF_PATH, FILE_NAME):
    """ 
        Loads extrinsic camera parameters from file  
    Args:
        file_path: `string` absolute path to yaml file
    Returns:
        file_path: `dict` extrinsic camera configuration
            dictionary
    """

    abs_path = os.path.join(CONF_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
    else:
        return None

    return data_loaded

# =============================================================================
if __name__ == '__main__':
    
    CONF_PATH = os.path.abspath(__file__ + "/../../../../../../configs")
    VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
    VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
    CAM_LABEL = "C"
    FILE_NAME = "Extrinsic_{}_{}_{}.yaml".format(
        VIDEO_WIDTH, VIDEO_HEIGHT, CAM_LABEL)

    extrinsic_params = read_extrinsic_params(
        CONF_PATH=CONF_PATH, 
        FILE_NAME=FILE_NAME)
    
    print(extrinsic_params)

# =============================================================================