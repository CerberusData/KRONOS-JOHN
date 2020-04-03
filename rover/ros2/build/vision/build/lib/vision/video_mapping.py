#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import os

import rclpy
from rclpy.node import Node

from vision.utils.cam_handler import read_cams_configuration

from rclpy.logging import get_logger

# =============================================================================
class VideoPublishers(Node):

    def __init__(self):
        super().__init__('VideoPublishers')

        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640)) 
        self._FR_AGENT = int(os.getenv(key="FR_AGENT", default=0)) 

        self.get_logger().info('HOLA HP:')

        # ---------------------------------------------------------------------
        # Initiate CameraSupervisors Class that handles the threads that reads 
        # the cameras
        self.cams_config = read_cams_configuration(FILE_NAME="cams_conf_local.yaml" 
            if self._LOCAL_RUN else "cams_conf.yaml")
        #if self.cams_config is None : # Validate cameras status
        #    pass
            #"No cameras were configured in configuration file, video mapping node stopped", 
            #log_type="err")
        
        print(self.cams_config)


        # Start cameras handler with configuration
        # cameras_supervisor = CamerasSupervisor(cams_config=cams_config)
    

        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = ""
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the executor 
    # is shutdown.

    video_publishers = VideoPublishers()
    rclpy.spin(video_publishers)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
