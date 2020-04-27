#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import rclpy
import time
import cv2
import os

from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vision.intrinsic.intrinsic_utils import read_intrinsic_params

from vision.extrinsic.extrinsic_utils import read_extrinsic_params
from vision.extrinsic.extrinsic_utils import find_projection
from vision.extrinsic.extrinsic_utils import get_rot_matrix
from vision.extrinsic.extrinsic_utils import pixel_relation

from vision.utils.vision_utils import flat_matrix_for_service
from vision.utils.vision_utils import printlog

from usr_msgs.msg import Intrinsic
from usr_msgs.msg import CamerasStatus
from usr_msgs.msg import VisualMessage
from std_msgs.msg import String

# =============================================================================
class CalibratorPublishers(Node):

    def __init__(self):

        # ---------------------------------------------------------------------
        super().__init__('CalibratorPublishers')

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # Constant/global variables
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
        self._INT_FILE_NAME = "Intrinsic_{}_{}.yaml".format(
            self._VIDEO_WIDTH, 
            self._VIDEO_HEIGHT)

        self._VISION_CAL_UNWARPED_WIDTH = int(os.getenv(key="VISION_CAL_UNWARPED_WIDTH", default=200))
        self._VISION_CAL_UNWARPED_HEIGHT = int(os.getenv(key="VISION_CAL_UNWARPED_HEIGHT", default=360))
        self._VISION_CAL_UNWARPED_SIZE = (
            self._VISION_CAL_UNWARPED_WIDTH, 
            self._VISION_CAL_UNWARPED_HEIGHT)
        self._VISION_CAL_PAT_VER = float(os.getenv(key="VISION_CAL_PAT_VER", default=1.0))
        self._VISION_CAL_PAT_HOZ = float(os.getenv(key="VISION_CAL_PAT_HOZ", default=1.0))
        self._VISION_CAL_PAT_TH_TOP = int(os.getenv(key="VISION_CAL_PAT_TH_TOP", default=15))
        self._VISION_CAL_PAT_TH_BOTTOM = int(os.getenv(key="VISION_CAL_PAT_TH_BOTTOM", default=15))
        self._VISION_CAL_SHOW_TIME = int(os.getenv(key="VISION_CAL_SHOW_TIME", default=5))
        self._VISION_CAL_TRIES = int(os.getenv(key="VISION_CAL_TRIES", default=20))
        self._VISION_CAL_PAT_ITE_TRIES = int(os.getenv(key="VISION_CAL_PAT_ITE_TRIES", default=20))
        self._VISION_CAL_DAYS_REMOVE = int(os.getenv(key="VISION_CAL_DAYS_REMOVE", default=20))
        self._VISION_CAL_DAYS_OUT = int(os.getenv(key="VISION_CAL_DAYS_OUT", default=20))
        self._VISION_CAL_SHOW_LOCAL = int(os.getenv(key="VISION_CAL_SHOW_LOCAL", default=0))
        self._HSVI = {
            "H": int(os.getenv(key="VISION_CAL_PAT_HI", default=58)),
            "S": int(os.getenv(key="VISION_CAL_PAT_SI", default=0)),
            "V": int(os.getenv(key="VISION_CAL_PAT_VI", default=115))}
        self._HSVS = {
            "H": int(os.getenv(key="VISION_CAL_PAT_HS", default=124)),
            "S": int(os.getenv(key="VISION_CAL_PAT_SS", default=255)),
            "V": int(os.getenv(key="VISION_CAL_PAT_VS", default=255))}
        self._UNWARPED_SIZE = (
            int(os.getenv(key="VISION_CAL_UNWARPED_WIDTH", default=200)),
            int(os.getenv(key="VISION_CAL_UNWARPED_HEIGHT", default=360)))

        # ---------------------------------------------------------------------
        # Variables
        self._cams_status = None
        self.intrinsic = None
        self.extrinsic = {}
        self._flag_image = cv2.imread(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 
            "extrinsic/figures/flag_4m.png"), cv2.IMREAD_UNCHANGED)
        self.calibrating_stat = False
        self.show_calibration_result = False # Enable/Disable calibration drawings
        self.calibration_image = None # Result image for calibrations

        # ---------------------------------------------------------------------
        # Read intrinsic parameters from file
        self.load_intrinsic()
        
        # ---------------------------------------------------------------------
        # Topics publishers
        self.pb_visual_debugger= self.create_publisher(
            VisualMessage, 'video_streaming/visual_debugger', 5)
        self.visual_debugger_msg = VisualMessage()

        # ---------------------------------------------------------------------
        # Topics Subscribers
        self.sub_cams_status = self.create_subscription(
            msg_type=CamerasStatus, topic='video_streaming/cams_status', 
            callback=self.cb_cams_status, qos_profile=5, 
            callback_group=self.callback_group)
        self.sub_cams_status = self.create_subscription(
            msg_type=String, topic='video_calibrator/calibrate', 
            callback=self.cb_calibrate, qos_profile=5,
            callback_group=self.callback_group)

        # ---------------------------------------------------------------------

    def cb_calibrate(self, msg):
        """ Callback function to calibration a camera
        Args:
            data: `String` ross message with camera label to calibrate
        Returns:
        """  
        
        # ---------------------------------------------------------------------
        cam_label = msg.data
        self.pub_log(msg="Calibrating camera {} ...".format(cam_label))    
        self.calibrating_stat = True

        # ---------------------------------------------------------------------
        # Check if camera label exits
        if cam_label not in self.extrinsic.keys():
            printlog(msg="{} is not a camera label available in extrinsics {}".format(
                cam_label, self.extrinsic.keys()), msg_type="ERROR")
            return 

        # If runing on local launch load a default picture to test calibration
        if self._LOCAL_RUN: 
            cam_img = cv2.imread(
                os.path.join(os.path.dirname(os.path.realpath(__file__)), 
                "extrinsic/calibration_default.jpg"), 
                int(cv2.IMREAD_UNCHANGED))
        else:
            # TODO: Get image from topic, for now with simulation image
            cam_img = None
            
        try:
            # If exits continue calibration process
            if self.intrinsic["camera_matrix"] is not None:
                for i in range(self._VISION_CAL_TRIES):                    
                    if self.calibrate_distances(
                        img_src=cam_img, camera_label=cam_label): 
                        break 
                    printlog(msg="Retrying camera {} calibration # {}".format(
                        cam_label, i+1), msg_type="WARN")
                    time.sleep(1) # Wait t seconds to try again camera calibration
                    if i == self._VISION_CAL_TRIES - 1:
                        printlog(msg="Camera {} no calibrated".format(
                                cam_label), msg_type="ERROR")
            else:
                printlog(msg="No intrinsic calibration to perform extrinsic", 
                    msg_type="ERROR")

        except OSError as err:
            printlog(msg="OS error: {0}".format(err), msg_type="ERROR")
        except ValueError:
            printlog(msg="ValueError raised", msg_type="ERROR")
        except Exception as e:
            printlog(msg="Exception error: {}".format(e), 
                msg_type="ERROR")
        except:
            printlog(msg="Unexpected error: {}".format(
                sys.exc_info()[0]), msg_type="ERROR")
            raise

        self.calibrating_stat = False

    def cb_intrinsic_params(self):

        intrinsic_conf = Intrinsic()

        intrinsic_conf.image_width = 0 if self.intrinsic is None else self.intrinsic["image_width"]
        intrinsic_conf.image_height = 0 if self.intrinsic is None else self.intrinsic["image_height"]
        intrinsic_conf.camera_matrix = [] if self.intrinsic is None else flat_matrix_for_service(self.intrinsic["camera_matrix"])
        intrinsic_conf.distortion_coefficients = [] if self.intrinsic is None else flat_matrix_for_service(self.intrinsic["distortion_coefficients"])
        intrinsic_conf.rectification_matrix = [] if self.intrinsic is None else flat_matrix_for_service(self.intrinsic["rectification_matrix"])
        intrinsic_conf.projection_matrix = [] if self.intrinsic is None else flat_matrix_for_service(self.intrinsic["projection_matrix"])

        return response

    def cb_cams_status(self, msg):

        if self._cams_status is None:
            self._cams_status = {
                cam_stat.split(":")[0] : int(cam_stat.split(":")[1])
                for cam_stat in msg.cams_status}
            self.get_logger().info("Got cameras status")
            self.load_extrinsic()

    def load_intrinsic(self):

        self.intrinsic = read_intrinsic_params(
            CONF_PATH=self._CONF_PATH, 
            FILE_NAME=self._INT_FILE_NAME)
        if self.intrinsic is None:
            self.get_logger().error(
                "loading instrinsic configuration file {}".format(
                self._INT_FILE_NAME))
        else:
            self.get_logger().info(
                "{} instrinsic configuration loaded".format(
                self._INT_FILE_NAME))

    def load_extrinsic(self):

        for cam_key, cam_status in self._cams_status.items():
            if not cam_status and not self._LOCAL_RUN:
                self.get_logger().warning(
                    "{} camera no response, no extrinsic configuration loaded".format(cam_key))
                continue
            cam_extrinsic = read_extrinsic_params(
                    CONF_PATH=self._CONF_PATH, 
                    FILE_NAME="Extrinsic_{}_{}_{}.yaml".format(
                        self._VIDEO_WIDTH, self._VIDEO_HEIGHT, cam_key))
            if cam_extrinsic is None:
                self.get_logger().warning(
                    "No extrinsic configuration file for {} camera".format(cam_key))
                self.extrinsic[cam_key] = None
                continue
            self.get_logger().info(
                "Extrinsic configuration loaded for {} camera".format(cam_key))
            self.extrinsic[cam_key] = cam_extrinsic

    def pub_log(self, msg, msg_type="INFO", prompt=True):
        
        self.visual_debugger_msg.data = msg 
        self.visual_debugger_msg.type = msg_type
        self.pb_visual_debugger.publish(self.visual_debugger_msg)
        if prompt:
            printlog(msg=self.visual_debugger_msg.data, msg_type=msg_type)
        
    def calibrate_distances(self, img_src, camera_label):

        # Perform extrinsic calibration 
        ext_cal = find_projection(img_src=img_src.copy(), 
            mtx=self.intrinsic["camera_matrix"], 
            dist=self.intrinsic["distortion_coefficients"], 
            PATTERN_THRESH_TOP=self._VISION_CAL_PAT_TH_TOP, 
            PATTERN_THRESH_BOTTOM=self._VISION_CAL_PAT_TH_BOTTOM,
            PATTERN_ITERATION_TRIES=self._VISION_CAL_PAT_ITE_TRIES,
            HSVI=self._HSVI, HSVS=self._HSVS,
            LOCAL_CALIBRATION=self._LOCAL_RUN and self._VISION_CAL_SHOW_LOCAL)
        
        if ext_cal is not None:
            try: 
                # Calculate rotation matrix from surface
                M = get_rot_matrix(p1=ext_cal["p1"], p2=ext_cal["p2"], p3=ext_cal["p3"], 
                    p4=ext_cal["p4"], UNWARPED_SIZE=self._UNWARPED_SIZE) 

                # Calculate the pixel relation in 'X' and 'Y' from surface 
                # projection and Rotation Matrix
                surf_relation = pixel_relation(
                    img_src=img_src.copy(), 
                    M=M, 
                    mtx=self.intrinsic["camera_matrix"], 
                    dist=self.intrinsic["distortion_coefficients"], 
                    p1=ext_cal["p1"], p2=ext_cal["p2"], p3=ext_cal["p3"], p4=ext_cal["p4"], 
                    Left_Line=ext_cal["Left_Line"], Right_Line=ext_cal["Right_Line"], 
                    Hy=ext_cal["Hy"], 
                    Dx_lines=self._VISION_CAL_PAT_HOZ,  
                    Dy_lines=self._VISION_CAL_PAT_VER, 
                    UNWARPED_SIZE=self._UNWARPED_SIZE, 
                    HSVI=self._HSVI, HSVS=self._HSVS,
                    PATTERN_ITERATION_TRIES=self._VISION_CAL_PAT_ITE_TRIES)
                if not surf_relation:
                    raise RuntimeError("No pattern found correctly")
                    return False

                # Save mono vision camera measurement parameters in file
                extrinsic_file_name = 'Extrinsic_{}_{}_{}.yaml'.format(
                    self._VIDEO_WIDTH, self._VIDEO_HEIGHT, camera_label)
                extrinsic_abs_path = os.path.join(
                    self._CONF_PATH, extrinsic_file_name)
                extrinsic_vision_params={
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
                printlog(msg="Calibracion para la camara {} exitosa!".format(
                    camera_label), msg_type="OKGREEN")

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
                # self.show_calibration_result = True
                # self.calibration_image = ruler_projected
                # time.sleep(self._SHOW_TIME)
                # self.calibration_image = calibration_image
                # time.sleep(self._SHOW_TIME)
                # self.show_calibration_result = False
                return True

            except Exception as e:
                printlog(msg="No se pudo encontrar el patron de calibracion! ({})".format(
                    e), msg_type="ERROR")
                return False
        else:
            printlog(msg="No se pudo encontrar el patron de calibracion!".format(
                e), msg_type="ERROR")
            return False

# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the 
    # executor is shutdown.
    calibrator_publishers = CalibratorPublishers()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    # Execute work and block until the context associated with the 
    # executor is shutdown. Callbacks will be executed by the provided 
    # executor.
    rclpy.spin(calibrator_publishers, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibrator_publishers.destroy_node()
    rclpy.shutdown()

# =============================================================================
if __name__ == '__main__':
    main()

# =============================================================================