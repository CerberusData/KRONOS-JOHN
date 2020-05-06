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
from usr_msgs.msg import Extrinsic

from vision.utils.vision_utils import print_text_list
from vision.utils.vision_utils import printlog

from vision.extrinsic.extrinsic_utils import ExtrinsicClass

# =============================================================================
class VisualDebuggerSubscriber():

    def __init__(self, parent_node):

        # Timer with time to show message
        self._VISUAL_DEBUGGER_TIME = int(
            os.getenv(key="VISUAL_DEBUGGER_TIME", default=10))

        # Message to show in console
        self.visual_debugger_msg = "" 
        # Type of message "info, err, warn"
        self.visual_debugger_type = "INFO" 
        
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

        self.visual_debugger_msg = msg.data
        self.visual_debugger_type = msg.type
        
        time.sleep(self._VISUAL_DEBUGGER_TIME)
        
        self.visual_debugger_msg = ""
        self.visual_debugger_type = "INFO"

    def draw_visual_debugger(self, img):
        """ Draws the visual debugger message
        Args:
            img: `cv2.math` image to draw visual
                debugger message
        Returns:
        """

        if not self.visual_debugger_msg:
            return img

        color = (255, 255, 255)

        if (self.visual_debugger_type == "ERROR" or
            self.visual_debugger_type == "ERR"):
            color = (0, 0, 255)
        elif (self.visual_debugger_type == "WARNING" or
            self.visual_debugger_type == "WARN"):
            color = (0, 255, 255)
        elif self.visual_debugger_type == "OKGREEN":
            color = (0, 255, 0)

        return print_text_list(
            img=img, tex_list=[self.visual_debugger_msg], 
            color=color, orig=(10, 50), 
            fontScale=0.7)

class ExtrinsicSubscriber():

    def __init__(self, parent_node):

        # Subscribers
        self._sub_extrinsic_params = parent_node.create_subscription(
            msg_type=Extrinsic, topic='video_calibrator/extrinsic_parameters', 
            callback=self.cb_extrinsic_params, qos_profile=2,
            callback_group=parent_node.callback_group
            )

        self.extrinsic = ExtrinsicClass()

    def cb_extrinsic_params(self, msg):
        """ Re-assing extrinsic calibration 
        Args:
            msg: `VisualMessage` message for visual debugger
                data: `string` message content
                type: `string` message type
        Returns:
        """

        try: 
            self.extrinsic.M = msg.projection_matrix
            self.extrinsic.p1 = msg.p1
            self.extrinsic.p2 = msg.p2
            self.extrinsic.p3 = msg.p3
            self.extrinsic.p4 = msg.p4
            self.extrinsic.vp = msg.vp
            self.extrinsic.dead_view = msg.dead_view
            self.extrinsic.ppmx = msg.ppmx
            self.extrinsic.ppmy = msg.ppmy
            self.extrinsic.warped_size = msg.unwarped_size
            self.extrinsic.image_size = msg.image_size

        except Exception as e:
            printlog(msg="Error getting extrinsic calibration"
                "from topic, {}".format(e), msg_type="ERROR")
            return False

        printlog(msg="Extrincid parameters updated", msg_type="INFO")

# =============================================================================