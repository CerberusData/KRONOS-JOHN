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
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from threading import Thread, Event

from vision.utils.vision_utils import printlog

# =============================================================================
class DataCaptureNode(Node, Thread):
    def __init__(self):
        """     
            Object class constructor
        Args:
        Returns:
        """

        # ---------------------------------------------------------------------
        super().__init__("DataCaptureNode")

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        Thread.__init__(self)
        self.rate = 20
        self.recording

        # ---------------------------------------------------------------------
        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.tick = time.time()
        # self.daemon = True
        self.start()

    def run(self):
        """ Cycle of threads execution
        Args:
        Returns:
        """

        while True:
            self.tick = time.time()
            try:

                pass

                # -------------------------------------------------------------
                # Operate times for next frame iteration
                tock = time.time() - self.tick
                twait = 1.0 / self.rate - tock
                if twait <= 0.0:
                    continue
                time.sleep(twait)
                # print("fps:", 1./(time.time() - self.tick), flush=True)

            except Exception as e:
                printlog(msg=e, msg_type="ERROR")

# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    capture_node = DataCaptureNode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(capture_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    capture_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
