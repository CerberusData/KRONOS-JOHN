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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vision.utils.cam_handler import read_cams_configuration
from vision.utils.cam_handler import CamerasSupervisor
from vision.utils.vision_utils import show_local_gui
from vision.utils.vision_utils import matrix_from_flat

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision.stitcher.stitcher import Stitcher

from usr_msgs.msg import CamerasStatus
from usr_msgs.msg import VisualMessage

from python_utils.pysubscribers import VisualDebugger

# =============================================================================
class MappingNode(Node, 
    VisualDebugger):

    def __init__(self):
        """ 
            VisualDebugger:
                Methods:
                    cb_visual_debugger [None]: callback function for subsciptor
                    draw_visual_debugger [cv2.math]: Draws the visual debugger message
                Arguments:
                    VISUAL_DEBUGGER_TIME [int]: Timer with time to show message
                    visual_debugger_msg  [string]:Message to show in console
                    visual_debugger_type [string]: Type of message "INFO, ERR, WARN"
                   _sub_visual_debugger [subscriptor]: 
        """

        # ---------------------------------------------------------------------
        super().__init__('MappingNode')

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        VisualDebugger.__init__(self, parent_node=self)
        
        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))

        self._FR_AGENT = int(os.getenv(key="FR_AGENT", default=0))
        self._FR_STREAMING_OPTIMIZER = int(os.getenv(key="FR_STREAMING_OPTIMIZER", default=0))

        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))

        self._STITCHER = int(os.getenv(key="STITCHER", default=0))
        self._STITCHER_SUP_MODE = int(os.getenv(key="STITCHER_SUP_MODE", default=0))

        self._VISUAL_DEBUGGER = int(os.getenv(key="VISUAL_DEBUGGER", default=1))
        self._VISUAL_DEBUGGER_TIME = int(os.getenv(key="VISUAL_DEBUGGER_TIME", default=5))
        
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
        self.img_optimizer = streaming_optimizer() 
        self.img_bridge = CvBridge()
        
        # Stitcher object
        self.stitcher = Stitcher(
            abs_path=os.path.join(self._CONF_PATH, "stitcher_config.npz"),
            super_stitcher=self._STITCHER_SUP_MODE) if self._STITCHER else None

        # ---------------------------------------------------------------------
        # Services
        
        # ---------------------------------------------------------------------
        # Publishers

        # Cameras streaming images
        self.cam_publishers = {}
        if self._FR_AGENT or self._LOCAL_RUN:
            self.cam_publishers = {cam_label:self.create_publisher(Image, 
                'streaming/cam_{}'.format(cam_label), 5) 
                for cam_label in self.cams_config.keys()
                if cams_status[cam_label] or self._LOCAL_RUN}
                
        # Cameras status
        self.pb_cams_status = self.create_publisher(
            CamerasStatus, 'video_streaming/cams_status', 5,
            callback_group=self.callback_group)

        # ---------------------------------------------------------------------
        # Publishers Timers

        self.cam_timers = {}
        self.cam_timers = {cam_label:self.create_timer(
            timer_period_sec=1./float(self.cams_config[cam_label]["FPS"]), 
            callback=partial(self.cb_cam_img_pub, cam_label),
            callback_group=self.callback_group)
            for cam_label in self.cams_config.keys()
            if cams_status[cam_label] or self._LOCAL_RUN}

        self.tm_pub_cams_status = self.create_timer(
            1.0, self.cb_cams_status)

        # ---------------------------------------------------------------------  
        # Local gui

        if self._LOCAL_RUN:
            self.gui_rate = 1./float(max(list(map(lambda o: int(o.cam_config["FPS"]), 
                self.cameras_supervisor.camera_handlers.values()))))
            self.gui_timer = self.create_timer(
                timer_period_sec=self.gui_rate, 
                callback=self.cb_draw_local_gui,
                callback_group=self.callback_group)
            
        # ---------------------------------------------------------------------  
        # Subscribers

        # ---------------------------------------------------------------------  

    def cb_cams_status(self):

        msg = CamerasStatus()
        msg.cams_status =  [str("{}:{}".format(cam_key, int(cam_status))
            ) for cam_key, cam_status in self.cameras_supervisor.get_cameras_status().items()]
        self.pb_cams_status.publish(msg)

    def cb_cam_img_pub(self, cam_label):
        """ Callback function to publish camera images
        Args:
        Returns:
        """
        
        img = self.cameras_supervisor.camera_handlers[cam_label].image
        if img is not None and self._FR_STREAMING_OPTIMIZER:
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
        """ Callback function to draw local user interface
        Args:
        Returns:
        """

        imgs_dic = dict(map(lambda o: (o.cam_label, o.image.copy()), 
                self.cameras_supervisor.camera_handlers.values()))
        if not self.stitcher is None and self._LOCAL_RUN: 
            imgs_dic["S"] = self.img_stitch(imgs_dic) 
        self.draw_visual_debugger(img=imgs_dic["C"])
        show_local_gui(
            imgs_dic=imgs_dic, 
            win_name="LOCAL_VIDEO_STREAMING")

    def img_stitch(self, imgs_dic):
        """ Stitch the cameras center, left and right
        Args:
            imgs_dic: `dict` 
                keys: `string` video streaming camera labels
                keys: `cv2.math` video streaming images
        Returns:
            _: `cv2.math` images stitched
        """
        
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