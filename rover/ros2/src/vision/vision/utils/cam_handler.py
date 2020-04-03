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
import yaml
import rclpy

# =============================================================================
def read_cams_configuration(
    FILE_PATH=os.path.dirname(os.path.abspath(__file__)), 
    FILE_NAME="cams_config.yaml"):
    """ Reads the camera labels, ports and other settings from file
    Args:
        FILE_PATH: `string` absolute path to configuration of cameras
        FILE_NAME: `string` name of cameras configuration file 
    Returns:
        _: `dictionary` key: camera labels, values: dictionary with camera 
            properties and settings, see yaml file for more details
    """    

    abs_path = os.path.join(FILE_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        return None

# =============================================================================