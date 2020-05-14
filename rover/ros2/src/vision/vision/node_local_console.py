#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from threading import Thread, Event

from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision.utils.vision_utils import printlog

# =============================================================================
class LocalConsoleNode(Node, Thread):

    def __init__(self, streaming_topic='streaming/cam_central', win_rate=30.):
        """ Object class constructor
        Args:
            streaming_topic: `string` topic of video streaming
            win_rate: `flota` window rate 
        Returns:
        """  

        # ---------------------------------------------------------------------
        super().__init__('LocalConsoleNode')
        Thread.__init__(self)
        
        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # window properties  
        self.win_name = "LOCAL_CONSOLE"
        self.win_mouse_click = None
        self.win_time = int(1000/win_rate)
        cv2.namedWindow(self.win_name)
        cv2.setMouseCallback(self.win_name, self.cb_mouse_event)

        # window image variables
        self.streaming_img = np.zeros((300, 300, 3), np.uint8)
        self.streaming_topic = streaming_topic
        self.img_bridge = CvBridge()

        # ---------------------------------------------------------------------  
        # Subscribers
        self.sub_calibrator_img_res = self.create_subscription(
            msg_type=Image, topic=self.streaming_topic, 
            callback=self.cb_streaming_img, qos_profile=5, 
            callback_group=self.callback_group)
        
        # ---------------------------------------------------------------------  
        # Publishers
        self.pub_cam_calibrate_msg = String()
        self.pub_cam_calibrate = self.create_publisher(
            String, 'video_calibrator/calibrate_cam', 5,
            callback_group=self.callback_group)

        # ---------------------------------------------------------------------  
        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.start()  

    def draw_mouse_event(self, img):
        """ Draw mouse events
        Args:
            img: `cv2.math` image to print components
        Returns:
        """  

        if self.win_mouse_click is not None:
            cv2.line(img=img, 
                pt1=(
                    int(self.win_mouse_click[0]*img.shape[1]-10), 
                    int(self.win_mouse_click[1]*img.shape[0])), 
                pt2=(
                    int(self.win_mouse_click[0]*img.shape[1]+10), 
                    int(self.win_mouse_click[1]*img.shape[0])), 
                color=(0, 0, 255), thickness=2)
            cv2.line(img=img, 
                pt1=(
                    int(self.win_mouse_click[0]*img.shape[1]), 
                    int(self.win_mouse_click[1]*img.shape[0]-10)), 
                pt2=(
                    int(self.win_mouse_click[0]*img.shape[1]), 
                    int(self.win_mouse_click[1]*img.shape[0]+10)), 
                color=(0, 0, 255), thickness=2)
            cv2.circle(img=img, 
                center=(
                    int(self.win_mouse_click[0]*img.shape[1]), 
                    int(self.win_mouse_click[1]*img.shape[0])), 
                radius=2, color=(0, 255, 255), thickness=-1)

    def cb_streaming_img(self, data):
        """     
            When called the variable self.streaming_img will be assigned 
            with the incoming data
        Args:
            data: `sensor_msgs.msg/Image` image 
        Returns:
        """

        try:
            self.streaming_img = self.img_bridge.imgmsg_to_cv2(
                img_msg=data, desired_encoding="bgr8")
        except CvBridgeError as e:
            printlog(msg="subscriber for camera in topic {}, {}".format(
                    self.streaming_topic, e), log_type="ERROR")

    def cb_mouse_event(self, event, x, y, flags, param):
        """
            Callback for mouse events in the extrinsic calibrator window 
            Args:
                event: 'int' mouse event
                x: 'int' mouse event x axis position 
                y: 'int' mouse event y axis position 
                flags: 'list[int]' flags given by mouse event 
                param: 'XXXX' XXXX
            returns:
        """
        
        x = x/self.streaming_img.shape[1]
        y = y/self.streaming_img.shape[0]
         
        # Add point 
        if event == cv2.EVENT_LBUTTONDOWN:
            self.win_mouse_click = (x, y)

    def cb_key_event(self, key):
        """
            Get and execute action from users key
            Args:
                key: 'int' keyboard actions
            returns:
        """

        # If pressed No key then continue
        if key == -1:
            pass
        # If pressed TAB key then open show help/menu options
        elif key == 9:
            pass
        # If pressed 1 Key then calibrate camera LL
        elif key == 49:
            self.pub_cam_calibrate_msg.data = "LL"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
        # If pressed 2 Key then calibrate camera C
        elif key == 50:
            self.pub_cam_calibrate_msg.data = "C"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
        # If pressed 3 Key then calibrate camera RR
        elif key == 51:
            self.pub_cam_calibrate_msg.data = "RR"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
        # If pressed 4 key then calibrate camera B
        elif key == 52:
            self.pub_cam_calibrate_msg.data = "B"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
        # If pressed right key then move to right
        elif key == 81:
            pass
        # If pressed up key then move to forward
        elif key == 82:
            pass
        # If pressed left key then move to left
        elif key == 83:
            pass
        # If pressed down key then move to backwards
        elif key == 84:
            pass
        # If pressed A key then switch to left camera
        elif key == 97:
            pass
        # If pressed M key then stiwch bewteen manual and waypoint mode
        elif key == 109:
            pass
        # If pressed D key then switch to right camera
        elif key == 100:
            pass
        # If pressed P key then open the lid
        elif key == 112:
            pass
        # If pressed Q key then quit
        elif key == 113:
            exit()
        # If pressed R key then switch to rear camera
        elif key == 114:
            pass
        # If pressed S key then activate/desactivate stitching mode
        elif key == 115:
            pass
        # If pressed no key defined then print message
        else:
            printlog(
                msg=f"{key} key action no defined", 
                msg_type="WARN")

    def run(self):
        """
            Run cicle of threadh
            Args:
            returns:
        """

        while True:
            try:
                self.draw_mouse_event(img=self.streaming_img)
                cv2.imshow(self.win_name, self.streaming_img)
                key = cv2.waitKey(self.win_time)
                self.cb_key_event(key=key)
            except Exception as e:
                    printlog(msg=e, msg_type="ERROR")

# =============================================================================
def main(args=None):

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the 
    # executor is shutdown.
    local_console_node = LocalConsoleNode()

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