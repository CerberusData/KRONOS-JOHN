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
import os

from rclpy.node import Node
from rclpy.logging import get_logger

from vision.intrinsic.intrinsic_utils import read_intrinsic_params
from vision.utils.vision_utils import flat_matrix_for_service
from usr_srv.srv import Intrinsic

# =============================================================================
class CalibratorPublishers(Node):

    def __init__(self):
        super().__init__('CalibratorPublishers')

        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
        
        # ---------------------------------------------------------------------
        # Read intrinsic parameters from file
        self._FILE_NAME = "Intrinsic_{}_{}.yaml".format(
            self._VIDEO_WIDTH, 
            self._VIDEO_HEIGHT)
        self.params = read_intrinsic_params(
            CONF_PATH=self._CONF_PATH, 
            FILE_NAME=self._FILE_NAME)
        if self.params is None:
            self.get_logger().error(
                "loading instrinsic configuration file {}".format(
                self._FILE_NAME))
        else:
            self.get_logger().info(
                "{} instrinsic configuration loaded".format(
                self._FILE_NAME))

        # ---------------------------------------------------------------------
        # Services
        self.srv_intrinsic_params = self.create_service(srv_type=Intrinsic, 
            srv_name="calibrator/intrinsic_params", callback=self.cb_intrinsic_params)
        
    def cb_intrinsic_params(self, request, response):

        response.image_width = 0 if self.params is None else self.params["image_width"]
        response.image_height = 0 if self.params is None else self.params["image_height"]
        response.camera_matrix = [] if self.params is None else flat_matrix_for_service(self.params["camera_matrix"])
        response.distortion_coefficients = [] if self.params is None else flat_matrix_for_service(self.params["distortion_coefficients"])
        response.rectification_matrix = [] if self.params is None else flat_matrix_for_service(self.params["rectification_matrix"])
        response.projection_matrix = [] if self.params is None else flat_matrix_for_service(self.params["projection_matrix"])

        return response

# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the 
    # executor is shutdown.
    calibrator_publishers = CalibratorPublishers()
    rclpy.spin(calibrator_publishers)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    calibrator_publishers.destroy_node()
    rclpy.shutdown()

# =============================================================================
if __name__ == '__main__':
    main()

# =============================================================================