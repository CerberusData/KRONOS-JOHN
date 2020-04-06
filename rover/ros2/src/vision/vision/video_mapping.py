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
from rclpy.logging import get_logger

from vision.utils.cam_handler import read_cams_configuration
from vision.utils.cam_handler import CamerasSupervisor

from usr_srv.srv import CamerasStatus
from cv_bridge import CvBridge, CvBridgeError

# =============================================================================
class VideoPublishers(Node):

    def __init__(self):
        super().__init__('VideoPublishers')

        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640)) 
        self._FR_AGENT = int(os.getenv(key="FR_AGENT", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))

        # ---------------------------------------------------------------------
        # Initiate CameraSupervisors Class that handles the threads that reads 
        # the cameras
        self.cams_config = read_cams_configuration(FILE_NAME="cams_conf_local.yaml" 
            if self._LOCAL_RUN else "cams_conf.yaml", CONF_PATH=self._CONF_PATH)
        if self.cams_config is None : # Validate cameras status
            self.get_logger().error("No cameras were configured in configuration")
            exit()
        else:
            self.get_logger().info("cameras configuration loaded")
        
        # Start cameras handler with configuration
        self.cameras_supervisor = CamerasSupervisor(cams_config=self.cams_config)
        
        # ---------------------------------------------------------------------
        # Services
        self.srv_cams_status = self.create_service(srv_type=CamerasStatus, 
            srv_name="streaming/cam_status", callback=self.cb_cams_status)
        
        # ---------------------------------------------------------------------
        # Publishers
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def cb_cams_status(self, request, response):
        response.cameras_status = ["{}:{}".format(cam_key, int(cam_status)
            ) for cam_key, cam_status in self.cameras_supervisor.get_cameras_status()]
        return response

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

# =============================================================================
if __name__ == '__main__':
    main()
