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

from vision.utils.vision_utils import print_text_list
from vision.utils.vision_utils import printlog

# =============================================================================
class VisualDebugger():

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

# class Extrinsic():

#     def __init__(self, parent_node):

#         pass

# class Intrinsic():

#     def __init__(self, parent_node):

#         # Subscribers
#         self._sub_visual_debugger = parent_node.create_subscription(
#             msg_type=VisualMessage, topic='video_streaming/visual_debugger', 
#             callback=self.cb_visual_debugger, qos_profile=5,
#             callback_group=parent_node.callback_group
#             )

# =============================================================================