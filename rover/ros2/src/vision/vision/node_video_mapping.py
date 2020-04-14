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
from vision.utils.vision_utils import matrix_from_flat

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from usr_srv.srv import CamerasStatus
from usr_srv.srv import Intrinsic

from vision.stitcher.stitcher import Stitcher

# =============================================================================
class MappingNode(Node):

    def __init__(self):
        super().__init__('MappingNode')

        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))

        self._FR_AGENT = int(os.getenv(key="FR_AGENT", default=0))

        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))

        self._STITCHER = int(os.getenv(key="STITCHER", default=0))
        self._STITCHER_SUP_MODE = int(os.getenv(key="STITCHER_SUP_MODE", default=0))

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
        self.img_optimizer = streaming_optimizer()

        # Stitcher object
        self.stitcher = Stitcher(
            abs_path=os.path.join(self._CONF_PATH, "stitcher_config.npz"),
            super_stitcher=self._STITCHER_SUP_MODE) if self._STITCHER else None

        # ---------------------------------------------------------------------
        # Services
        self.srv_cams_status = self.create_service(srv_type=CamerasStatus, 
            srv_name="video_mapping/cams_status", callback=self.cb_cams_status)
        
        # ---------------------------------------------------------------------
        # Publishers
        self.cam_publishers = {}
        if self._FR_AGENT or self._LOCAL_RUN:
            self.cam_publishers = {cam_label:self.create_publisher(Image, 
                'streaming/cam_{}'.format(cam_label), 5) 
                for cam_label in self.cams_config.keys()
                if cams_status[cam_label] or self._LOCAL_RUN}
                
        # ---------------------------------------------------------------------
        # Subscribers

        # ---------------------------------------------------------------------
        # Timers
        self.cam_timers = {}
        self.cam_timers = {cam_label:self.create_timer(
            timer_period_sec=1./float(self.cams_config[cam_label]["FPS"]), 
            callback=partial(self.cb_cam_img_pub, cam_label))
            for cam_label in self.cams_config.keys()
            if cams_status[cam_label] or self._LOCAL_RUN}

        # ---------------------------------------------------------------------  
        # Local gui
        if self._LOCAL_RUN:
            self.gui_rate = 1./float(max(list(map(lambda o: int(o.cam_config["FPS"]), 
                self.cameras_supervisor.camera_handlers.values()))))
            self.gui_timer = self.create_timer(
                timer_period_sec=self.gui_rate, 
                callback=self.cb_draw_local_gui)

        # ----------------------------------------------------------------------
        # Intrisic Calibration
        self.srv_cli_intrinsic = self.create_client(
            srv_type=Intrinsic, 
            srv_name='calibrator/intrinsic_params')
        while not self.srv_cli_intrinsic.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('service not available, waiting again...')
        self.srv_intrinsic_req = Intrinsic.Request()

    def send_request(self):
        self.future = self.srv_cli_intrinsic.call_async(
            self.srv_intrinsic_req)

    def cb_cams_status(self, request, response):

        response.cameras_status = ["{}:{}".format(cam_key, int(cam_status)
            ) for cam_key, cam_status in self.cameras_supervisor.get_cameras_status()]
        return response

    def cb_cam_img_pub(self, cam_label):

        img = self.cameras_supervisor.camera_handlers[cam_label].image
        if img is not None:
            img = self.img_optimizer.optimize(
                img=img, cam_label=cam_label)

            # Send supervisor's image message to topic
            try:
                img_msg = self.img_bridge.cv2_to_imgmsg(
                    cvim=img, encoding="bgr8")
                # img_msg.header.stamp = time.time()
                self.cam_publishers[cam_label].publish(img_msg)

            except CvBridgeError as e:
                self.get_logger().error("publishing CAM{} image in topic, {}".format(
                    cam_label, e))

    def cb_draw_local_gui(self):

        imgs_dic = dict(map(lambda o: (o.cam_label, o.image.copy()), 
                self.cameras_supervisor.camera_handlers.values()))
        if not self.stitcher is None: 
            imgs_dic["S"] = self.img_stitch(imgs_dic) 
        show_local_gui(
            imgs_dic=imgs_dic, 
            win_name="LOCAL_VIDEO_STREAMING")
        
    def img_stitch(self, imgs_dic):

        cam_keys = self.cameras_supervisor.camera_handlers.keys()
        imgs = [
            None if not "RR" in cam_keys else imgs_dic["RR"].copy(),
            None if not "C" in cam_keys else imgs_dic["C"].copy(),
            None if not "LL" in cam_keys else imgs_dic["LL"].copy()]
        for idx, img in enumerate(imgs):
            if img is not None:
                if (img.shape[0]!=self._VIDEO_HEIGHT or 
                    img.shape[1]!=self._VIDEO_WIDTH):
                    imgs[idx] = cv2.resize(img, (self._VIDEO_WIDTH, self._VIDEO_HEIGHT), 
                        int(cv2.INTER_NEAREST))

        return self.stitcher.image_resize(
            image= self.stitcher.stitch(images=imgs), 
            width=self._VIDEO_WIDTH, 
            height=self._VIDEO_HEIGHT)

class streaming_optimizer(object):

    def __init__(self):
        """     
            Creates video optimizer object for third party services to get the 
            parameters with images quality are reduced to be sent.
        Args:
        Returns:
        """
        # Read local variables of video streaming configuration
        self.IDLE_TIME = int(os.environ.get("FR_STREAMING_IDLE_TIME", 120))
        self.STREAMING_FACTOR = float(os.getenv("FR_STREAMING_FACTOR", 0.4)) 
        self.STREAMING_IDLE_FACTOR = float(os.getenv("FR_STREAMING_IDLE_FACTOR", 0.2))  
        
        self.inactive_timer = 0
        self._time_tick = time.time()

    def optimize(self, img, cam_label=None):
        """     
            reduces the images quality to be sent.
        Args:
            img: `cv2.math` image to reduce quality
        Returns:
        """

        if self.inactive_timer < self.IDLE_TIME + 1:
            self.inactive_timer = time.time() - self._time_tick

        if cam_label is not None:  
            hfc = 0.2 if img.shape[1] < 500 else 0.4
            org = (int(img.shape[1]*hfc), int(img.shape[0]*0.90))      
            img = cv2.putText(img=img, text="CAMERA_{}".format(cam_label), org = org, 
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, 
                color=(0, 0, 0), thickness = 4, lineType = cv2.LINE_AA)
            img = cv2.putText(img=img, text="CAMERA_{}".format(cam_label), org = org, 
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, 
                color=(0, 255, 255), thickness = 1, lineType = cv2.LINE_AA)

        elif self.inactive_timer>=self.IDLE_TIME:
            img = cv2.resize(src=cv2.cvtColor(src=img, code=cv2.COLOR_BGR2GRAY), 
                dsize=(int(img.shape[1] * self.STREAMING_IDLE_FACTOR), 
                       int(img.shape[0] * self.STREAMING_IDLE_FACTOR)))
            img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)

        return img

    # TODO(JOHN): Integrate robot actions to reset idle time 
    def cb_actuator_action(self):
        """     
            Reset inactive timer when a action coming from actuator reference
            control is received
        Args:
        Returns:
        """
        self._time_tick = time.time()

# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the 
    # executor is shutdown.
    mapping_node = MappingNode()
    mapping_node.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(mapping_node)

        if mapping_node.future.done():
            try:
                response = mapping_node.future.result()
            except Exception as e:
                mapping_node.get_logger().info(
                    'Service call failed %r' % (e,))
            # print(response, flush=True)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mapping_node.destroy_node()
    rclpy.shutdown()

# =============================================================================
if __name__ == '__main__':
    main()

# =============================================================================