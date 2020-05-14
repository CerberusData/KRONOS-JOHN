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

from threading import Thread, Event

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vision.utils.cam_handler import read_cams_configuration
from vision.utils.cam_handler import CamerasCaptures

from vision.utils.vision_utils import matrix_from_flat
from vision.utils.vision_utils import print_text_list
from vision.utils.vision_utils import show_local_gui
from vision.utils.vision_utils import insert_image
from vision.utils.vision_utils import printlog

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision.stitcher.stitcher import Stitcher

from usr_msgs.msg import CamerasStatus
from usr_msgs.msg import VisualMessage

from std_msgs.msg import String

from python_utils.pysubscribers import VisualDebuggerSubscriber
from python_utils.pysubscribers import ExtrinsicSubscriber

# =============================================================================
class GraphicInterface(
    VisualDebuggerSubscriber):

    def __init__(self, parent_node):
        """ 
            Object class constructor
            Methods:
                cb_visual_debugger [None]: callback function for subscriptor
                draw_visual_debugger [cv2.math]: Draws the visual debugger 
                    message
            Arguments:
                VISUAL_DEBUGGER_TIME [int]: Timer with time to show message
                visual_debugger_msg  [string]:Message to show in console
                visual_debugger_type [string]: Type of message 
                    "INFO, ERR, WARN"
            _sub_visual_debugger [subscriptor]: 
        """

        # ---------------------------------------------------------------------
        VisualDebuggerSubscriber.__init__(self, parent_node=self)

        # ---------------------------------------------------------------------
        self._VISUAL_DEBUGGER = int(os.getenv(
            key="VISUAL_DEBUGGER", default=1))

        # ---------------------------------------------------------------------

    def draw_components(self, imgs_dict):
        """ 
            Draw graphic and user components on image
            Methods:
                draw image components, graphic interface
            Arguments:
                imgs_dict: 'dict of cv2.math' dictionary with cameras 
                    images 
        """

        # ---------------------------------------------------------------------
        # OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS
        # self.draw_lateral_cams(imgs_dict)
        if "RR" in imgs_dict.keys():
            insert_image(
                original_image=imgs_dict["P"], inserted_image=imgs_dict["RR"], 
                new_width=int(imgs_dict["P"].shape[1]*0.3), 
                new_height=int(imgs_dict["P"].shape[0]*0.3), 
                position='ur')
        if "LL" in imgs_dict.keys():
            insert_image(
                original_image=imgs_dict["P"], inserted_image=imgs_dict["LL"], 
                new_width=int(imgs_dict["P"].shape[1]*0.3), 
                new_height=int(imgs_dict["P"].shape[0]*0.3), 
                position='ul')
        if "B" in imgs_dict.keys():
            insert_image(
                original_image=imgs_dict["P"], inserted_image=imgs_dict["B"], 
                new_width=int(imgs_dict["P"].shape[1]*0.4), 
                new_height=int(imgs_dict["P"].shape[0]*0.4), 
                position='uc')

        # ---------------------------------------------------------------------
        # DRAW MESSAGES DEBUGGER - DRAW MESSAGES DEBUGGER - DRAW MESSAGES DEBUG
        if self._VISUAL_DEBUGGER:
            self.draw_visual_debugger(img=imgs_dict["P"])

    def draw_extrinsic(self):
        """ 
            Draw extrinsic calibration components
            Methods:
            Arguments:
        """
        pass

    def draw_lateral_cams(self, imgs_dict):
        """ 
            Draw lateral cameras components on image
            Methods:
            Arguments:
        """
        pass

    def draw_rear_camera(self):
        """ 
            Draw rear camera component on image
            Methods:
            Arguments:
        """
        pass

    def draw_compass(self):
        """ 
            Draw compass component on image
            Methods:
            Arguments:
        """
        pass

    def draw_visual_debugger(self, img):
        """ Draw the visual debugger message
        Args:
            img: `cv2.math` image to draw visual
                debugger message
        Returns:
        """

        if not self.visual_debugger_msg:
            return

        color = (255, 255, 255)
        if (self.visual_debugger_type == "ERROR" or
            self.visual_debugger_type == "ERR"):
            color = (0, 0, 255)
        elif (self.visual_debugger_type == "WARNING" or
            self.visual_debugger_type == "WARN"):
            color = (0, 255, 255)
        elif self.visual_debugger_type == "OKGREEN":
            color = (0, 255, 0)

        print_text_list(
            img=img, tex_list=[self.visual_debugger_msg], 
            color=color, orig=(10, int(img.shape[0]*0.95)), 
            fontScale=0.7)

class StreamingOptimizer(object):

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

    def optimize(self, img):
        """     
            reduces the images quality to be sent.
        Args:
            img: `cv2.math` image to reduce quality
        Returns:
        """

        if self.inactive_timer < self.IDLE_TIME + 1:
            self.inactive_timer = time.time() - self._time_tick

        elif self.inactive_timer >= self.IDLE_TIME:
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

class MappingNode(
    Node, Thread,
    GraphicInterface):

    def __init__(self):
        """     
            Object class constructor
        Args:
        Returns:
        """
        
        # ---------------------------------------------------------------------
        super().__init__('MappingNode')

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()
        
        Thread.__init__(self)
        GraphicInterface.__init__(self, parent_node=self)

        # ---------------------------------------------------------------------
        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 
        self._LOCAL_GUI = int(os.getenv(key="LOCAL_GUI", default=0)) 
        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))

        self._FR_AGENT = int(os.getenv(key="FR_AGENT", default=0))
        self._FR_STREAMING_OPTIMIZER = int(os.getenv(
            key="FR_STREAMING_OPTIMIZER", default=0))

        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))

        self._STITCHER = int(os.getenv(
            key="STITCHER", default=0))
        self._STITCHER_SUP_MODE = int(os.getenv(
            key="STITCHER_SUP_MODE", default=0))

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
        self.cams_caps = CamerasCaptures(cams_config=self.cams_config)
        # print(cams_status = self.cams_caps.get_cameras_status())
                
        self.img_optimizer = StreamingOptimizer() 
        self.img_bridge = CvBridge()
        
        # ---------------------------------------------------------------------
        # Stitcher object
        self.stitcher = None
        if self._STITCHER:
            self.stitcher = Stitcher(
                abs_path=os.path.join(self._CONF_PATH, "stitcher_config.npz"),
                super_stitcher=self._STITCHER_SUP_MODE) if self._STITCHER else None

        # ---------------------------------------------------------------------
        # Services
        
        # ---------------------------------------------------------------------
        # Publishers

        # Cameras streaming images
        self.pub_streaming = None
        if self._FR_AGENT and "C" in self.cams_config.keys():
            if self.cams_config["C"]["TOPIC"] != "None":
                self.pub_streaming = self.create_publisher(
                    Image, self.cams_config["C"]["TOPIC"], 10) 
            else:
                printlog(msg="No topic for video camera C streaming topic"
                    ", no thread and topic will be created",
                    msg_type="ERROR")

        # Cameras status
        self.pb_cams_status = self.create_publisher(
            CamerasStatus, 'video_streaming/cams_status', 5,
            callback_group=self.callback_group)

        # Image to calibrate
        self.pb_img_to_calibrate = self.create_publisher(
            Image, 'video_calibrator/calibrate_img', 5,
            callback_group=self.callback_group)
        self.sub_calibration = self.create_subscription(
            msg_type=String, topic='video_calibrator/calibrate_cam', 
            callback=self.cb_send_img_calibrate, qos_profile=5,
            callback_group=self.callback_group)

        # ---------------------------------------------------------------------  
        # Subscribers
        self.calibrator_img = None
        self.sub_calibrator_img_res = self.create_subscription(
            msg_type=Image, topic='video_calibrator/calibrate_img_result', 
            callback=self.cb_cal_img_result, qos_profile=5, 
            callback_group=self.callback_group)
        
        # ---------------------------------------------------------------------
        self.tm_pub_cams_status = self.create_timer(
            1.0, self.cb_cams_status)

        # ---------------------------------------------------------------------  
        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.tick = time.time()
        # self.daemon = True
        self.start()

    def cb_cams_status(self):
        """ Callback function to publish cameras status
        Args:
        Returns:
        """
        
        msg = CamerasStatus()
        msg.cams_status =  [str("{}:{}".format(cam_key, int(cam_status))
            ) for cam_key, cam_status in self.cams_caps.get_cameras_status().items()]
        self.pb_cams_status.publish(msg)

    def cb_cal_img_result(self, msg):
        """ Callback function to assing image calibraion result and show it
            on main supervisors image
        Args:
        Returns:
        """

        try:
            self.calibrator_img = self.img_bridge.imgmsg_to_cv2(
                img_msg=msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            printlog(msg="erro while getting data from video_calibrator/"
                "calibrate_img_result, {}".format(e), msg_type="ERROR")

        time.sleep(int(os.getenv(
            key="VISION_CAL_SHOW_TIME", default=5)))

        self.calibrator_img = None

    def cb_send_img_calibrate(self, msg):
        """ Callback to send image to calibrate through topic
        Args:
            msg: 'CvBridge' image to send to calibrate
        Returns:
        """

        cam_label = msg.data

        if cam_label not in self.cameras_supervisor.camera_handlers.keys():
            self.get_logger().error(
                "error publishing image to calibrator, key {} no found".format(
                    cam_label))
            return

        # Send supervisor's image message to calibrator
        try:
            img_msg = self.img_bridge.cv2_to_imgmsg(
                cvim=self.cameras_supervisor.camera_handlers[cam_label].image, 
                encoding="bgr8")
            # img_msg.header.stamp = time.time()
            self.pb_img_to_calibrate.publish(img_msg)

        except CvBridgeError as e:
            self.get_logger().error(
                "publishing image to calibrator, {}".format(e))

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

    def run(self):
        """ Cycle of threadh execution
        Args:
        Returns:
        """

        while True:
            self.tick = time.time()
            try:
                # read images from cameras
                imgs_dic = dict(map(
                    lambda o: (o.cam_label, o.get_image()), 
                    self.cams_caps.camera_handlers.values()))
                
                # Show calibration if something to show
                if self.calibrator_img is not None:
                    imgs_dic["P"] = self.calibrator_img
                else: # Else user grapich components
                    imgs_dic["P"] = imgs_dic["C"].copy()
                    self.draw_components(
                        imgs_dict=imgs_dic)

                    # Optimize image
                    if self._FR_STREAMING_OPTIMIZER:
                        self.img_optimizer.optimize(img=imgs_dic["P"])

                # TODO: integrate stitcher
                # if self._STITCHER and "other_condition":
                #   self.img_stitch(imgs_dic)

                # -------------------------------------------------------------
                # Publish image
                if self.pub_streaming is not None:
                    img_msg = self.img_bridge.cv2_to_imgmsg(
                        cvim=imgs_dic["P"], encoding="bgr8")
                    t = str(self.get_clock().now().nanoseconds)
                    img_msg.header.stamp.sec = int(t[0:10])
                    img_msg.header.stamp.nanosec = int(t[10:])
                    img_msg.header.frame_id = t
                    self.pub_streaming.publish(img_msg)
                
                # -------------------------------------------------------------
                # Operate times for next frame iteration
                tock = time.time() - self.tick
                twait = 1./self.cams_config["C"]["FPS"] - tock
                if twait <= 0.:
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