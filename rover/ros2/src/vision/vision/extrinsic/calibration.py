# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np

import traceback
import rospy
import time
import copy
import yaml
import cv2
import os

from web_client.msg import Messages
from std_msgs.msg import String
from std_msgs.msg import Bool

from extended_rospylogs import Debugger
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from python_utils.utils import load_intrinsic_params
from calibration_utils import return_pixel_relation
from calibration_utils import create_ruler_mask
from calibration_utils import find_projection
from calibration_utils import get_rot_matrix
from vision.utils import overlay_image

from easy_memmap import EasyMemmap, MultiImagesMemmap

# =============================================================================
LOCAL_LAUNCH = int(os.getenv(key="LOCAL_LAUNCH", default=0))

# =============================================================================
class Calibrator(Debugger):

    def __init__(self, queue=2):

        # inherit from debuger
        super(Calibrator,self).__init__()

        # Create subscribers
        self._calibrate_sus = rospy.Subscriber("video_mapping/calibrate", String, 
            self._calibrate_cb, queue_size=queue)
        
        # Create publishers
        self._calibrate_done_pub = rospy.Publisher('video_mapping/calibrate/done', Bool, queue_size=queue)
        self._update_pub = rospy.Publisher("web_client/configure", Bool, queue_size=queue)
        self._info_web_pub = rospy.Publisher('web_client/message', Messages, queue_size=queue)        
        self.info_msg = Messages() # Variable for calibration messages

        # Pattern calibration variables
        self._SHOW_TIME = int(os.getenv(key="VISION_CALIBRATION_SHOW_TIME", default=10))
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
        self._VIDEO_SIZE = (self._VIDEO_WIDTH, self._VIDEO_HEIGHT)
        self._UNWARPED_WIDTH = int(os.getenv(key="VISION_MONO_UNWARPED_WIDTH", default=200))
        self._UNWARPED_HEIGHT = int(os.getenv(key="VISION_MONO_UNWARPED_HEIGHT", default=360))
        self._UNWARPED_SIZE = (self._UNWARPED_WIDTH, self._UNWARPED_HEIGHT)
        self._PATTERN_VER = float(os.getenv(key="VISION_CALIBRATION_PATTERN_VER", default=1.0))
        self._PATTERN_HOZ = float(os.getenv(key="VISION_CALIBRATION_PATTERN_HOZ", default=1.0))
        self._PATTERN_THRESH_TOP = int(os.getenv(key="VISION_CALIBRATION_PATTERN_THRESH_TOP", default=15))
        self._PATTERN_THRESH_BOTTOM = int(os.getenv(key="VISION_CALIBRATION_PATTERN_THRESH_BOTTOM", default=0))
        self._PATTERN_ITERATION_TRIES = int(os.getenv(key="VISION_CALIBRATION_PATTERN_ITERATION_TRIES", default=20))
        self._HSVI = {
            "H": int(os.getenv(key="VISION_CALIBRATION_PATTERN_HI", default=75)),
            "S": int(os.getenv(key="VISION_CALIBRATION_PATTERN_SI", default=70)),
            "V": int(os.getenv(key="VISION_CALIBRATION_PATTERN_VI", default=130))}
        self._HSVS = {
            "H": int(os.getenv(key="VISION_CALIBRATION_PATTERN_HS", default=115)),
            "S": int(os.getenv(key="VISION_CALIBRATION_PATTERN_SS", default=255)),
            "V": int(os.getenv(key="VISION_CALIBRATION_PATTERN_VS", default=255))}
        
        # Memmap to catch supervisors image to be streamed
        self.video_map = None

        # Load cameras intrinsic parameters
        intrinsic_path = os.path.join(
            os.getenv(key="VISION_INTRINSIC_PATH", default="/usr/src/app/"), 
            "Intrinsic_{}_{}.yaml".format(self._VIDEO_WIDTH, self._VIDEO_HEIGHT))
        self.mtx, _, self.dist, _, _ = load_intrinsic_params(
            file_path=intrinsic_path)
        if self.mtx is None or self.dist is None: # If there's no calibration to load
            self.debugger(DEBUG_LEVEL_0, "No intrinsic calibration file found, for" 
                "video size {}x{}".format(self._VIDEO_WIDTH, self._VIDEO_HEIGHT), 
                log_type="err")

        self.show_calibration_result = False # Enable/Disable calibration drawings
        self.calibration_image = None # Result image for calibrations
        self.calibration_folder = os.getenv(key="VISION_EXTRINSIC_PATH", 
            default="/data")

        #wait for publishers
        time.sleep(1) 

        # Read flag image to overlay over result calibration image
        self._flag_image = cv2.imread(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 
            "figures/flag_4m.png"), cv2.IMREAD_UNCHANGED)

        self.calibrating_stat = False

    def calibrate_distances(self, img_src, camera_label):

        # If runing on local launch load a default picture to test calibration
        if LOCAL_LAUNCH: 
            img_src = cv2.imread(os.path.join(
                os.path.dirname(os.path.abspath(__file__)), 
                "calibration_default.jpg"))
            
        # Perform extrinsic calibration 
        ext_cal = find_projection(img_src=img_src.copy(), 
            mtx=self.mtx, dist=self.dist, 
            PATTERN_THRESH_TOP=self._PATTERN_THRESH_TOP, 
            PATTERN_THRESH_BOTTOM=self._PATTERN_THRESH_BOTTOM,
            PATTERN_ITERATION_TRIES=self._PATTERN_ITERATION_TRIES,
            HSVI=self._HSVI, HSVS=self._HSVS)

        if ext_cal is not None:
            try: 
                # Calculate rotation matrix from surface
                M = get_rot_matrix(p1=ext_cal["p1"], p2=ext_cal["p2"], p3=ext_cal["p3"], 
                    p4=ext_cal["p4"], UNWARPED_SIZE=self._UNWARPED_SIZE) 

                # Calculate the pixel relation in 'X' and 'Y' from surface 
                # projection and Rotation Matrix
                surf_relation = return_pixel_relation(
                    img_src=img_src.copy(), M=M, mtx=self.mtx, dist=self.dist, 
                    p1=ext_cal["p1"], p2=ext_cal["p2"], p3=ext_cal["p3"], p4=ext_cal["p4"], 
                    Left_Line=ext_cal["Left_Line"], Right_Line=ext_cal["Right_Line"], 
                    Hy=ext_cal["Hy"], Dx_lines=self._PATTERN_HOZ,  Dy_lines=self._PATTERN_VER, 
                    UNWARPED_SIZE=self._UNWARPED_SIZE, HSVI=self._HSVI, HSVS=self._HSVS,
                    PATTERN_ITERATION_TRIES=self._PATTERN_ITERATION_TRIES)
                if not surf_relation:
                    raise RuntimeError("No pattern found correctly")
                    return False

                # Save mono vision camera measurement parameters in file
                mono_file_name = 'Extrinsic_{}_{}_{}.yaml'.format(
                    self._VIDEO_SIZE[0], self._VIDEO_SIZE[1], camera_label)
                mono_abs_path = os.path.join(self.calibration_folder, mono_file_name)
                mono_vision_params={
                    "M": M.tolist(),
                    "vp": ext_cal["vp"].tolist(), 
                    "p1": ext_cal["p1"], 
                    "p2": ext_cal["p2"], 
                    "p3": ext_cal["p3"], 
                    "p4": ext_cal["p4"], 
                    "dead_view": surf_relation["dead_view_distance"],
                    "ppmx": surf_relation["ppmx"], 
                    "ppmy": surf_relation["ppmy"], 
                    "unwarped_size": self._UNWARPED_SIZE,
                    "image_size": self._VIDEO_SIZE
                    }
                self.save_params_and_test(mono_abs_path=mono_abs_path, 
                    data=mono_vision_params)
        
                # Report successfully calibration
                self.debugger_pilots_console("Calibracion para la camara {} exitosa!".format(camera_label))
                self._calibrate_done_pub.publish(True)

                # Set calibration image and show it to pilots console
                mask_pro, mask_src = create_ruler_mask(flag_img=self._flag_image, 
                    mono_params=mono_vision_params)
                self.calibration_image = overlay_image(l_img=img_src.copy(), 
                    s_img=mask_src, pos=(0, 0), transparency=1)
                calibration_image = self.calibration_image.copy()

                # overlay with rules in wrapped perspective
                cal_img_pro = cv2.warpPerspective(src=self.calibration_image, 
                    M=M, dsize=self._UNWARPED_SIZE)
                ruler_projected_orig = overlay_image(l_img=cal_img_pro, 
                    s_img=mask_pro, pos=(0, 0), transparency=1)
                ruler_projected = cv2.resize(src=cv2.rotate(src=ruler_projected_orig, 
                    rotateCode=cv2.ROTATE_90_CLOCKWISE), dsize=self._VIDEO_SIZE)

                # Show calibration results
                self.show_calibration_result = True
                self.calibration_image = ruler_projected
                time.sleep(self._SHOW_TIME)
                self.calibration_image = calibration_image
                time.sleep(self._SHOW_TIME)
                self.show_calibration_result = False
                return True

            except Exception as e:
                self.debugger_pilots_console(
                    "No se pudo encontrar el patron de calibracion! ({})".format(e),
                    log_type="err")
                traceback.print_exc()
                return False

        else:
            self.debugger_pilots_console("No se pudo encontrar el patron de calibracion!", 
                log_type="err")
            return False
    
    def save_params_and_test(self, mono_abs_path, data):
        """ Saves and check file with mono vision calibration
        Args:
            file_name: `String` name to save mono vision parameters file
            data: `Dictionary` to save in file
        Returns:
        """    
        try: # Save the calibration data in file
            with open(mono_abs_path, 'w') as outfile: 
                yaml.dump(data, outfile, default_flow_style=False)
            print("[INFO][EXTRINSIC]: Camera extrinsic calibration saved {}".format(mono_abs_path)) 
        except IOError as e: # Report any error saving de data
            print("[ERROR][EXTRINSIC]: Problem saving camera extrinsic calibration: {}".format(e))

    def _calibrate_cb(self, data):
        """ Callback function to calibration a camera
        Args:
            data: `String` ross message with camera label to calibrate
        Returns:
        """    
        self.calibrating_stat = True

        if self.video_map is None:
            self.video_map = MultiImagesMemmap(mode="r", name="main_stream", 
                memmap_path=os.getenv("VIDEO_MEMMAP_PATH", "/tmp"))
            self.video_map.wait_until_available()

        camera2calibrate = data.data
        calibration_tries = int(os.getenv(key="VISION_CALIBRATION_TRIES", default=5))
        if camera2calibrate not in self.video_map.labels_dict: # Check if cam_label is images dictionary
            self.debugger( DEBUG_LEVEL_0, "{} is not a camera label available: {}".format(
                camera2calibrate, self.video_map.labels_dict), log_type="err")
        else:
            if self.mtx is None:
                self.debugger(DEBUG_LEVEL_0, "No distortion file found, cannot \
                    calibrate cameras", log_type="err")
            else:
                for i in range(calibration_tries):                    
                    if self.calibrate_distances(
                        img_src=self.video_map.read(camera2calibrate), 
                        camera_label=camera2calibrate): 
                        break # Break if calibration succeed
                    else:
                        if i != calibration_tries-1:
                            self.debugger(DEBUG_LEVEL_0, "Retrying camera {} \
                                calibration # {}".format(camera2calibrate, i+1), 
                                log_type="warn")
                            time.sleep(1) # Wait t seconds to try again camera calibration
        
        self.calibrating_stat = False

    def debugger_pilots_console(self, msg, log_type="info"):
        """     
            check if calibration for camera label 'camera' already exits
        Args:
            msg: `string` message to print o set in debugger
            log_type: `string` type of message 
        Returns:
        """
        
        if log_type == "info":
            self.info_msg.type = Messages.INFO
        elif log_type == "warn":
            self.info_msg.type = Messages.WARNING
        elif log_type == "err":
            self.info_msg.type = Messages.ERROR
        self.debugger(DEBUG_LEVEL_0, msg, log_type = log_type)
        self.info_msg.data = msg
        self._info_web_pub.publish(self.info_msg)

# =============================================================================