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
import cv2
import os

from vision.utils.vision_utils import printlog

class IntrinsicClass():

    def __init__(self, FILE_NAME=None, FILE_PATH=None):
        """ 
            Class constructor
        Args:
            FILE_NAME: 'string' intrinsic file name
            FILE_PATH: 'string' intrinsic file path
        Returns:
        """

        self.file_name = "Intrinsic_{}_{}.yaml".format(
            int(os.getenv(key="VIDEO_WIDTH", default=640)), 
            int(os.getenv(key="VIDEO_HEIGHT", default=360))) if FILE_NAME is None else FILE_NAME
        self.file_path = str(os.getenv(key="CONF_PATH", default=os.path.dirname(
            os.path.abspath(__file__)))) if FILE_PATH is None else FILE_PATH

        self.image_width = None
        self.image_height = None
        self.mtx = None
        self.distortion_model = None
        self.distortion_coefficients = None
        self.rectification_matrix = None
        self.projection_matrix = None
        self.map1 = None
        self.map2 = None

        self.load()

    def load(self):
        """ 
            Loads intrinsic camera parameters from file  
        Args:
        Returns:
        """

        try:
            abs_path = os.path.join( self.file_path, self.file_name)
            
            if os.path.isfile(abs_path):
                with open(abs_path, 'r') as stream:
                    data = yaml.safe_load(stream)
            else:
                printlog(
                    msg="No instrinsic configuration file {}".format(
                        self.file_name), msg_type="ERROR")
                return
            
            for key in [
                "camera_matrix", 
                "distortion_coefficients",
                "rectification_matrix",
                "projection_matrix"]:

                if key not in data:
                    printlog(
                        msg="Intrinsic file {}, invalid".format(
                        FILE_NAME), msg_type="ERROR")
                    raise Exception('invalid file format')

                data[key] = \
                    np.array(data[key]["data"]).reshape(
                        data[key]["rows"], 
                        data[key]["cols"])

            self.image_width = data["image_width"]
            self.image_height = data["image_height"]
            self.mtx = data["camera_matrix"]
            self.distortion_model = data["distortion_model"]
            self.distortion_coefficients = data["distortion_coefficients"]
            self.rectification_matrix = data["rectification_matrix"]
            self.projection_matrix = data["projection_matrix"]

            map1, map2 = cv2.initUndistortRectifyMap(
                        cameraMatrix=self.mtx, 
                        distCoeffs=self.distortion_coefficients, 
                        R=np.array([]), 
                        newCameraMatrix=self.mtx, 
                        size=(
                            data["image_width"], 
                            data["image_height"]), 
                        m1type=cv2.CV_8UC1)
            self.map1 = map1
            self.map2 = map2

            printlog(msg="{} instrinsic configuration loaded".format(
                self.file_name), msg_type="OKGREEN")

        except Exception as e:

            self.image_width = None
            self.image_height = None
            self.mtx = None
            self.distortion_model = None
            self.distortion_coefficients = None
            self.rectification_matrix = None
            self.projection_matrix = None
            self.map1 = None
            self.map2 = None
            
            printlog(
                msg="instrinsic file {} error, {}".format(
                self.file_name, e), msg_type="ERROR")

# =============================================================================
if __name__ == '__main__':
    
    CONF_PATH = os.path.abspath(__file__ + "/../../../../../../configs")
    VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
    VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
    FILE_NAME = "Intrinsic_{}_{}.yaml".format(VIDEO_WIDTH, VIDEO_HEIGHT)

    intrisic_params = IntrinsicClass(FILE_NAME=FILE_NAME, FILE_PATH=CONF_PATH)

# =============================================================================