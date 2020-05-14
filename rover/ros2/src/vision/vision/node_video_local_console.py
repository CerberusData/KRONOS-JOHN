#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import cv2

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# =============================================================================
class LocalConsoleNode(Node, Thread):

    def __init__(self):
        
        self._win_name = "LOCAL_CONSOLE"

        # ---------------------------------------------------------------------  
        # Subscribers
        self.calibrator_img = np.zeros((300, 300, 3), np.uint8)
        self.sub_calibrator_img_res = self.create_subscription(
            msg_type=Image, topic='streaming/cam_central', 
            callback=self.cb_streaming_img, qos_profile=5, 
            callback_group=self.callback_group)

    def cb_streaming_img(self, data):
        pass
    
    def cb_mouse_event(self, data):
        pass

    def cb_key_event(self, key):
        pass

# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the 
    # executor is shutdown.
    mapping_node = LocalConsoleNode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()
  
    # Execute work and block until the context associated with the 
    # executor is shutdown. Callbacks will be executed by the provided 
    # executor.
    rclpy.spin(mapping_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mapping_node.destroy_node()
    rclpy.shutdown()

# =============================================================================
if __name__ == '__main__':
    main()

# =============================================================================