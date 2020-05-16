#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import time
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from usr_msgs.msg import VisualMessage
from usr_msgs.msg import Extrinsic as Extrinsic_msg
from std_msgs.msg import Bool

from vision.utils.vision_utils import print_text_list
from vision.utils.vision_utils import printlog

from vision.intrinsic.intrinsic_utils import IntrinsicClass
from vision.extrinsic.extrinsic_utils import Extrinsic

# =============================================================================
class VisualDebuggerSubscriber():

    def __init__(self, parent_node):

        # Timer with time to show message
        self._VISUAL_DEBUGGER_TIME = int(
            os.getenv(key="VISUAL_DEBUGGER_TIME", default=10))

        # Message to show in console
        self.msg = "" 
        # Type of message "info, err, warn"
        self.type = "INFO" 
        
        # Subscribers
        self._sub_visual_debugger = parent_node.create_subscription(
            msg_type=VisualMessage, topic='video_streaming/visual_debugger', 
            callback=self.cb_visual_debugger, qos_profile=5,
            callback_group=parent_node.callback_group
            )

    def cb_visual_debugger(self, msg):
        """ Draws the visual debugger message
        Args:
            msg: `VisualMessage` message for visual debugger
                data: `string` message content
                type: `string` message type
        Returns:
        """

        self.msg = msg.data
        self.type = msg.type
        
        time.sleep(self._VISUAL_DEBUGGER_TIME)
        
        self.msg = ""
        self.type = "INFO"

    def draw(self, img):
        """ Draw the visual debugger message
        Args:
            img: `cv2.math` image to draw visual
                debugger message
        Returns:
        """

        if not self.msg:
            return

        color = (255, 255, 255)
        if (self.type == "ERROR" or
            self.type == "ERR"):
            color = (0, 0, 255)
        elif (self.type == "WARNING" or
            self.type == "WARN"):
            color = (0, 255, 255)
        elif self.type == "OKGREEN":
            color = (0, 255, 0)

        print_text_list(
            img=img, tex_list=[self.msg], 
            color=color, orig=(10, int(img.shape[0]*0.95)), 
            fontScale=0.7)

class ExtrinsicSubscriber():

    def __init__(self, parent_node, cam_labels=["C"]):

        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))

        # Subscribers
        self._sub_extrinsic_params = parent_node.create_subscription(
            msg_type=Extrinsic_msg, topic='video_calibrator/extrinsic_parameters', 
            callback=self.cb_extrinsic_params, qos_profile=2,
            callback_group=parent_node.callback_group
            )

        # Read intrinsic parameters from file
        self.intrinsic = IntrinsicClass()

        # extrinsic parameters dictionary
        self.extrinsic = Extrinsic(
            dist=self.intrinsic.distortion_coefficients,
            mtx=self.intrinsic.mtx,
            cam_labels=cam_labels)

    def cb_extrinsic_params(self, msg):
        """ Re-assing extrinsic calibration 
        Args:
            msg: `VisualMessage` message for visual debugger
                data: `string` message content
                type: `string` message type
        Returns:
        """

        try: 
            if msg.cam_label in self.extrinsic.cams.keys():
                self.extrinsic.load(
                    mtx=self.intrinsic.mtx, 
                    dist=self.intrinsic.distortion_coefficients,
                    FILE_NAME="Extrinsic_{}_{}_{}.yaml".format(
                        self._VIDEO_WIDTH, 
                        self._VIDEO_HEIGHT, 
                        msg.cam_label))

        except Exception as e:
            printlog(msg="Error getting extrinsic calibration"
                "from topic, {}".format(e), msg_type="ERROR")
            return False
        printlog(msg="Extrinsic parameters for CAM{} updated".format(
            msg.cam_label), msg_type="INFO")

class WebclientControl():

    def __init__(self, parent_node):

        self.control_tilt = 0
        self.control_pan = 0
        self.control_throttle = 0

class WaypointSuscriber():

    def __init__(self):
        
        pass

class Robot():

    def __init__(self, parent_node):

        self.stream_stitch = False
        self.stream_rear_cam = False

        # Subscribers
        self._sub_extrinsic_params = parent_node.create_subscription(
            msg_type=Bool, topic='video_streaming/stitch', 
            callback=self.cb_video_streaming_stitch, qos_profile=1,
            callback_group=parent_node.callback_group)

        self._sub_extrinsic_params = parent_node.create_subscription(
            msg_type=Bool, topic='video_streaming/rear_cam', 
            callback=self.cb_video_streaming_rear_cam, qos_profile=1,
            callback_group=parent_node.callback_group)

    def cb_video_streaming_rear_cam(self, data):
        self.stream_rear_cam = not self.stream_rear_cam

    def cb_video_streaming_stitch(self, data):
        self.stream_stitch = not self.stream_stitch


# =============================================================================