#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np
import rclpy
import time
import yaml
import cv2
import os

from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vision.intrinsic.intrinsic_utils import IntrinsicClass
from vision.extrinsic.extrinsic_utils import read_extrinsic_params
from vision.extrinsic.extrinsic_utils import find_projection
from vision.extrinsic.extrinsic_utils import get_rot_matrix
from vision.extrinsic.extrinsic_utils import pixel_relation
from vision.extrinsic.extrinsic_utils import create_ruler_mask

from vision.utils.vision_utils import flat_matrix_for_service
from vision.utils.vision_utils import overlay_image
from vision.utils.vision_utils import printlog

from usr_msgs.msg import CamerasStatus
from usr_msgs.msg import VisualMessage
from usr_msgs.msg import Extrinsic
from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
        self._LOCAL_GUI = int(os.getenv(key="LOCAL_GUI", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
        self._VIDEO_SIZE = (self._VIDEO_WIDTH, self._VIDEO_HEIGHT)
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
        self.cam_img = None
        self.extrinsic = {}
        self._flag_image = cv2.imread(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 
            "extrinsic/figures/flag_4m.png"), cv2.IMREAD_UNCHANGED)
        self.calibrating_stat = False
        self.calibration_image = None # Result image for calibrations

        # ---------------------------------------------------------------------
        # Read intrinsic parameters from file
        self.intrinsic = IntrinsicClass()

        # ---------------------------------------------------------------------
        # Topics publishers
        self.pb_visual_debugger_msg = VisualMessage()
        self.pb_visual_debugger = self.create_publisher(
            VisualMessage, 'video_streaming/visual_debugger', 5)

        self.img_bridge = CvBridge()
        self.pb_calibrator_result = self.create_publisher(
            Image, 'video_calibrator/extrinsic_img_result', 2)

        self.pb_calibrator_extrinsic = self.create_publisher(
            Extrinsic, 'video_calibrator/extrinsic_parameters', 2)

        # ---------------------------------------------------------------------
        # Topics Subscribers
        self.sub_cams_status = self.create_subscription(
            msg_type=CamerasStatus, topic='video_streaming/cams_status', 
            callback=self.cb_cams_status, qos_profile=5, 
            callback_group=self.callback_group)

        self.sub_calibration = self.create_subscription(
            msg_type=String, topic='video_calibrator/calibrate', 
            callback=self.cb_calibrate, qos_profile=5,
            callback_group=self.callback_group)

        self.sub_img_to_calibrate = self.create_subscription(
            msg_type=Image, topic='video_calibrator/img_to_calibrate', 
            callback=self.cb_img_to_calibrate, qos_profile=5,
            callback_group=self.callback_group)

        # ---------------------------------------------------------------------

    def cb_img_to_calibrate(self, msg):
        """ Callback function to update camera image to calibrate
        Args:
            data: `cv2.math` image to calibrate
        Returns:
        """  

        try:
            self.cam_img = self.img_bridge.imgmsg_to_cv2(
                img_msg=msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            printlog(msg="erro while getting data from video_calibrator/img_to_calibrate, {}".format(
                e), msg_type="ERROR")

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
            time.sleep(10)
            return 

        # If runing on local launch load a default picture to test calibration
        if self._LOCAL_RUN: 
            self.cam_img = cv2.imread(
                os.path.join(os.path.dirname(os.path.realpath(__file__)), 
                "extrinsic/calibration_default.jpg"), 
                int(cv2.IMREAD_UNCHANGED))
        else:
            # wait image from topic, for now with simulation image
            timeout = time.time() + 10   # 10 seconds from now
            while True:
                if self.cam_img is not None or time.time() > timeout:
                    break
            if self.cam_img is None:
                printlog(msg="Timeout getting image from topic, camera no calibrated",
                     msg_type="ERROR")
                self.calibrating_stat = False
                return False

            if self.intrinsic.mtx is not None and not self._LOCAL_RUN:

                # Resize image if size diferento to intrinsic calibration size
                if (self.cam_img.shape[1] != self.intrinsic.image_width
                    and self.cam_img.shape[0] != self.intrinsic.image_height):
                    self.cam_img = c2.resize(self.cam_img, 
                        (self.intrinsic.image_width, 
                        self.intrinsic.image_height))

                # Undistord image if intrinsic calibration
                self.cam_img = cv2.remap(self.cam_img, 
                    self.intrinsic.map1, self.intrinsic.map2,
                    cv2.INTER_LINEAR) 
            
        try:
            # If exits continue calibration process
            if self.intrinsic.mtx is not None:
                self.calibrate_distances(
                    img_src=self.cam_img, 
                    camera_label=cam_label)
            else:
                self.pub_log(msg="No intrinsic calibration to perform extrinsic", 
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
        self.cam_img = None

    def cb_cams_status(self, msg):
        """ Callback function to get cameras status
        Args:
            msg: `String` cameras status
        Returns:
        """  

        if self._cams_status is None:

            self._cams_status = {
                cam_stat.split(":")[0] : int(cam_stat.split(":")[1])
                for cam_stat in msg.cams_status}

            self.get_logger().info("Got cameras status")
            self.load_extrinsic()

    def publish_extrinsic(self, cam_label, extrinsic_data):

        try: 
            extrinsic_msg = Extrinsic()
            
            extrinsic_msg.cam_label = cam_label
            extrinsic_msg.projection_matrix = flat_matrix_for_service(
                numpy_array=extrinsic_data["M"])
            extrinsic_msg.vp = extrinsic_data["vp"]
            extrinsic_msg.p1 = extrinsic_data["p1"]
            extrinsic_msg.p2 = extrinsic_data["p2"]
            extrinsic_msg.p3 = extrinsic_data["p3"]
            extrinsic_msg.p4 = extrinsic_data["p4"]
            extrinsic_msg.dead_view = extrinsic_data["dead_view"]
            extrinsic_msg.ppmx = extrinsic_data["ppmx"]
            extrinsic_msg.ppmy = extrinsic_data["ppmy"]
            extrinsic_msg.unwarped_size = extrinsic_data["unwarped_size"]
            extrinsic_msg.image_size = extrinsic_data["image_size"]

            self.pb_calibrator_extrinsic.publish(extrinsic_msg)
        
        except Exception as e:
            printlog(msg="Error publishing extrinsic calibration"
                "trought topic, {}".format(e), msg_type="ERROR")
            return False

    def load_extrinsic(self):
        """     
            load extrinsic parameters from file
            Args:
            Returns:
        """

        for cam_key, cam_status in self._cams_status.items():
            
            self.extrinsic[cam_key] = None

            if not cam_status and not self._LOCAL_RUN:
                self.get_logger().warning(
                    "{} camera no response, no extrinsic configuration loaded".format(
                        cam_key))
                continue

            cam_extrinsic = read_extrinsic_params(
                    CONF_PATH=self._CONF_PATH, 
                    FILE_NAME="Extrinsic_{}_{}_{}.yaml".format(
                        self._VIDEO_WIDTH, self._VIDEO_HEIGHT, cam_key))
            
            if cam_extrinsic is None:
                self.get_logger().warning(
                    "No extrinsic configuration file for {} camera".format(
                        cam_key))
                self.extrinsic[cam_key] = None
                continue
            
            self.get_logger().info(
                "Extrinsic configuration loaded for {} camera".format(cam_key))
            self.extrinsic[cam_key] = cam_extrinsic

    def pub_log(self, msg, msg_type="INFO", prompt=True):
        """
            Print log message tracking file and function caller 
        Args:
            msg: `string` message to print
            msg_type: `string` message type
            prompt: `boolean` sure that any output is buffered and go to the 
                destination.
        Returns:
        """

        self.pb_visual_debugger_msg.data = msg 
        self.pb_visual_debugger_msg.type = msg_type
        self.pb_visual_debugger.publish(self.pb_visual_debugger_msg)

        if prompt:
            printlog(msg=self.pb_visual_debugger_msg.data, msg_type=msg_type)
        
    def calibrate_distances(self, img_src, camera_label):
        """
            Print log message tracking file and function caller 
        Args:
            img_src: `cv2.math` image to calibrate
            camera_label: `string` camera label
        Returns:
        """

        # Perform extrinsic calibration 
        ext_cal = find_projection(img_src=img_src.copy(), 
            mtx=self.intrinsic.mtx, 
            dist=self.intrinsic.distortion_coefficients, 
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
                    mtx=self.intrinsic.mtx, 
                    dist=self.intrinsic.distortion_coefficients, 
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
                    "vp": [int(ext_cal["vp"][0]), 
                           int(ext_cal["vp"][1])], 
                    "p1": list(ext_cal["p1"]), 
                    "p2": list(ext_cal["p2"]), 
                    "p3": list(ext_cal["p3"]), 
                    "p4": list(ext_cal["p4"]), 
                    "dead_view": surf_relation["dead_view_distance"],
                    "ppmx": surf_relation["ppmx"],
                    "ppmy": surf_relation["ppmy"], 
                    "unwarped_size": list(self._UNWARPED_SIZE),
                    "image_size": list(self._VIDEO_SIZE)
                    }

                self.save_extrinsic(file_path=extrinsic_abs_path, 
                    data=extrinsic_vision_params)
        
                # Set calibration image and show it to pilots console
                mask_pro, mask_src = create_ruler_mask(flag_img=self._flag_image, 
                    extrinsic_params=extrinsic_vision_params)
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

                if self._LOCAL_RUN:
                    cv2.imwrite(os.path.join(
                        os.path.dirname(os.path.realpath(__file__)), "extrinsic",
                        "extrinsic_cal_projection.jpg"), ruler_projected)
                    cv2.imwrite(os.path.join(
                        os.path.dirname(os.path.realpath(__file__)), "extrinsic",
                        "extrinsic_cal_source.jpg"), calibration_image)

                # Send supervisor's image message to topic
                try:
                    img_ext_result = self.img_bridge.cv2_to_imgmsg(
                        cvim=ruler_projected, encoding="bgr8")
                    # img_msg.header.stamp = time.time()
                    self.pb_calibrator_result.publish(img_ext_result)
                   
                    time.sleep(self._VISION_CAL_SHOW_TIME)

                    img_ext_result = self.img_bridge.cv2_to_imgmsg(
                        cvim=calibration_image, encoding="bgr8")
                    # img_msg.header.stamp = time.time()
                    self.pb_calibrator_result.publish(img_ext_result)

                    time.sleep(self._VISION_CAL_SHOW_TIME)

                except CvBridgeError as e:
                    self.get_logger().error(
                        "publishing extrinsic result images through topic, {}".format(e))

                # Publish extrinsic result trought topic
                self.publish_extrinsic(cam_label=camera_label, 
                    extrinsic_data=extrinsic_vision_params)

                # Report successfully calibration
                self.pub_log(msg="Calibracion para la camara {} exitosa!".format(
                    camera_label), msg_type="OKGREEN")

                return True

            except Exception as e:
                printlog(msg="No se pudo encontrar el patron de calibracion! ({})".format(
                    e), msg_type="ERROR")
                return False

        else:
            self.pub_log(msg="No se pudo encontrar el patron de calibracion!", 
                msg_type="ERROR")
            return False

    def save_extrinsic(self, file_path, data):
        """ Saves file with extrinsic calibration
        Args:
            file_path: `String` name to save mono vision parameters file
            data: `Dictionary` with extrinsic calibration to save in file
        Returns:
        """

        try: # Save the calibration data in file

            with open(file_path, 'w') as outfile: 
                yaml.dump(data, outfile, default_flow_style=False)
            printlog(msg="Camera extrinsic calibration saved {}".format(
                file_path), msg_type="INFO")

        except IOError as e: # Report any error saving de data
            printlog(msg="Problem saving camera extrinsic calibration: {}".format(e), 
                msg_type="ERROR")

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