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
import cv2
import os

from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vision.intrinsic.intrinsic_utils import read_intrinsic_params
from vision.extrinsic.extrinsic_utils import read_extrinsic_params
from vision.utils.vision_utils import flat_matrix_for_service

from usr_msg.msg import Intrinsic
from usr_msg.msg import CamerasStatus
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
        self._VISION_CAL_PAT_TH_BOTTOM = int(os.getenv(key="VISION_CAL_PAT_TH_BOTTOM", default=0))
        self._VISION_CAL_SHOW_TIME = int(os.getenv(key="VISION_CAL_SHOW_TIME", default=0))
        self._VISION_CAL_TRIES = int(os.getenv(key="VISION_CAL_TRIES", default=20))
        self._VISION_CAL_PAT_ITE_TRIES = int(os.getenv(key="VISION_CAL_PAT_ITE_TRIES", default=20))
        self._VISION_CAL_DAYS_REMOVE = int(os.getenv(key="VISION_CAL_DAYS_REMOVE", default=20))
        self._VISION_CAL_DAYS_OUT = int(os.getenv(key="VISION_CAL_DAYS_OUT", default=20))
        self._HSVI = {
            "H": int(os.getenv(key="VISION_CAL_PAT_HI", default=58)),
            "S": int(os.getenv(key="VISION_CAL_PAT_SI", default=0)),
            "V": int(os.getenv(key="VISION_CAL_PAT_VI", default=115))}
        self._HSVS = {
            "H": int(os.getenv(key="VISION_CAL_PAT_HS", default=115)),
            "S": int(os.getenv(key="VISION_CAL_PAT_SS", default=255)),
            "V": int(os.getenv(key="VISION_CAL_PAT_VS", default=255))}

        # ---------------------------------------------------------------------
        # Variables
        self._cams_status = None
        self.intrinsic = None
        self.extrinsic = {}
        self._flag_image = cv2.imread(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 
            "figures/flag_4m.png"), cv2.IMREAD_UNCHANGED)
        self.calibrating_stat = False
        self.show_calibration_result = False # Enable/Disable calibration drawings
        self.calibration_image = None # Result image for calibrations

        # ---------------------------------------------------------------------
        # Read intrinsic parameters from file
        self.load_intrinsic()
        
        # ---------------------------------------------------------------------
        # Topics publishers
        self.pb_visual_debugger= self.create_publisher(
            String, 'video_streaming/visual_debugger', 5)
        self.visual_debugger_msg = String()

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
        
        cam_label = msg.data

        self.visual_debugger_msg.data = "calibrating camera {}".format(cam_label)
        self.pb_visual_debugger.publish(self.visual_debugger_msg)

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