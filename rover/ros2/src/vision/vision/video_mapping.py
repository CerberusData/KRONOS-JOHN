#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
from functools import partial 
import time
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from vision.utils.cam_handler import read_cams_configuration
from vision.utils.cam_handler import CamerasSupervisor
from vision.utils.vision_utils import show_local_gui

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from usr_srv.srv import CamerasStatus

# =============================================================================
class VideoPublishers(Node):

    def __init__(self):
        super().__init__('VideoPublishers')

        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
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
        cams_status = self.cameras_supervisor.get_cameras_status()
        self.img_bridge = CvBridge()
        
        # ---------------------------------------------------------------------
        # Services
        self.srv_cams_status = self.create_service(srv_type=CamerasStatus, 
            srv_name="streaming/cam_status", callback=self.cb_cams_status)
        
        # ---------------------------------------------------------------------
        # Publishers
        self.cam_publishers = {}
        if self._FR_AGENT or self._LOCAL_RUN:
            self.cam_publishers = {cam_label:self.create_publisher(Image, 
                'streaming/cam_{}'.format(cam_label), 5) 
                for cam_label in self.cams_config.keys()
                if cams_status[cam_label] or self._LOCAL_RUN}        

        # ---------------------------------------------------------------------
        # Timers
        self.cam_timers = {cam_label:self.create_timer(0.5, partial(
            self.timer_callback, cam_label))
            for cam_label in self.cams_config.keys()
            if cams_status[cam_label] or self._LOCAL_RUN}

        # ---------------------------------------------------------------------  
        # Local gui
        self.local_gui = local_gui()

    def cb_cams_status(self, request, response):

        response.cameras_status = ["{}:{}".format(cam_key, int(cam_status)
            ) for cam_key, cam_status in self.cameras_supervisor.get_cameras_status()]
        return response

    def timer_callback(self, cam_label):

        img = self.cameras_supervisor.camera_handlers[cam_label].image
        if img is not None:
            # Send supervisor's image message to topic
            try:
                img_msg = self.img_bridge.cv2_to_imgmsg(
                    cvim=img, 
                    encoding="bgr8")
                # img_msg.header.stamp = time.time()
                self.cam_publishers[cam_label].publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error("publishing CAM{} image in topic, {}".format(
                    cam_label, e))

class local_gui():

    def __init__(self):

        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 



# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the 
    # executor is shutdown.
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
