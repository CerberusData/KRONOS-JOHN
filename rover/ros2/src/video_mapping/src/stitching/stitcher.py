# =============================================================================
"""
Code Information:
	Programmer: Eng. Carlos Andres Alvarez (Charlie)
    Maintainer: Eng. John Alberto Betancourt G
	Phone: +57 (350) 283 51 22
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer Vision Team
"""

# =============================================================================
import os
import time

from threading import Thread, Event
from math import sin, atan2

import rospy
import cv2
import numpy as np

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from web_client.msg import Messages

from python_utils.utils import get_world_coords

from stitching_class import Stitcher

from tf.transformations import quaternion_from_euler

# =============================================================================
class StitcherThread(Thread, Debugger):

    def __init__(self, mono_params, image_size, **kwargs):
        super(StitcherThread,self).__init__()

        
        # self.run_event = Event() # Event for stoping threading runing
        # self.run_event.set()
        # self.daemon = True # Make thread daemonic (if father dies it dies with the father)

        
        self.stitch = False # If the thread is performing stitching operation
        self.images = None # List of images: [LL, CC, RR]
        self.result = None # Result image of processing
        self.pan = 0 # Pan for virtual pan
        self.stitch_all = False # Stitch all
        self.zoom_roi = None # Zoom roi for zooming
        self.mono_params = mono_params # Mono params object with calibrations
        self.ORIGINAL_SIZE = image_size # Size of images
        self._FR_AGENT = int(os.getenv("FR_AGENT", 0))

        # # path where is allocated the default stitch conf file
        # stitcher_config_path = os.getenv("STITCHER_PATH", "/usr/src/app")

        # # Stitcher actual object that does the stuff
        # self.stitcher = Stitcher(os.path.join(stitcher_config_path,"Stitcher_Default.npz"), **kwargs)
        # self.stitcher.set_proj_cnts(cams_params=mono_params.calibration)

        # ---------------------------------------------------------------------
        # Subscriber to know when a calbration was performed and update mono params for stitcher
        rospy.Subscriber(name="/video_mapping/calibrate/done", data_class=Bool, 
            callback=self.calibration_cb, queue_size=10)

        # Subscriber data handles clicks, converts to meters and forwards is to nav control
        rospy.Subscriber(name="/vision/video_position", data_class=PointStamped,
            callback=self._video_pos_cb, queue_size=10)

        # Subscriber for detecting when waypoint is active
        self.waypoint_active = False
        rospy.Subscriber(name="/motion_control/position_controller/error", data_class=PoseStamped, 
            callback=self._nav_control_cb, queue_size=10)

        # ---------------------------------------------------------------------
        # Navigation control publisher
        self.nav_pub = rospy.Publisher(name="/motion_control/position_controller/desired_pose", 
            data_class=PoseStamped, queue_size=10)

        # Publisher for debugging messages in supervisors console image 
        self.info_web_pub = rospy.Publisher(name='/video_mapping/message', 
            data_class=Messages, queue_size=10)
        self.info_msg = Messages()

    def calibration_cb(self, data):
        if data.data:
            time.sleep(1)
            self.stitcher.set_proj_cnts(cams_params=self.mono_params.calibration)

    def _nav_control_cb(self, data):
        # if error in x (forward) is negative, control is over, otherwise it is active
        self.waypoint_active = True if data.pose.position.x > 0 else False
            
    # Callback function to handle click actions commands from pilots console
    # It transforms the clicks to actual coordinates in meter if possible
    def _video_pos_cb(self, data):
        camera = "C"
        nav_msg = PoseStamped()
        x_cord = data.point.x
        y_cord = data.point.y
        
        if self.mono_params.calibration[camera] is not None:
            dist = -1
            # Normal waypoint mode
            if not self.stitch:
                x_dst, y_dst = int(x_cord*self.ORIGINAL_SIZE[0]), int(y_cord*self.ORIGINAL_SIZE[1])

                # Get actual pan in pixels
                pan = self.mono_params._ORIGINAL_SIZE[0]*(int(self.pan))//100

                # fix x coordinate acoording to actual pan to match actual image
                x_dst = x_dst + pan
                dist = cv2.pointPolygonTest(self.mono_params.calibration[camera]["waypoint_area"],(x_dst,y_dst),True)
            
            # Panoramic waypoint
            else:
                # String, tuple, tuple
                source_label, source_coord, coord_stitched = self.stitcher.get_coord_source([x_cord,y_cord])

                if source_label is not None:
                    if source_label == "C":
                        dist = 1
                        x_dst, y_dst = source_coord
                
            if dist > 0:

                x, y = get_world_coords(
                    x_dst,
                    y_dst,
                    self.mono_params.calibration[camera]["M"],
                    self.mono_params.calibration[camera]["bot_view_cord"],
                    self.mono_params.calibration[camera]["ppmx"],
                    self.mono_params.calibration[camera]["ppmy"],
                    self.mono_params.mtx,
                    self.mono_params.dist
                )

                # for robotics x is forward (y in an image) and y is to the sides (x in an image)
                nav_msg.header.stamp = rospy.Time.now()                   
                nav_msg.pose.position.x = y
                nav_msg.pose.position.y = x
                theta = atan2(x,y) 
                quaternion = quaternion_from_euler(0.0, 0.0, theta)
                nav_msg.pose.orientation.x = quaternion[0]
                nav_msg.pose.orientation.y = quaternion[1]
                nav_msg.pose.orientation.z = quaternion[2]
                nav_msg.pose.orientation.w = quaternion[3]

                self.nav_pub.publish(nav_msg)
            else:
                if self.waypoint_active:
                    # Canceling current command
                    nav_msg.header.stamp = rospy.Time.now()   
                    nav_msg.pose.position.x = 0
                    nav_msg.pose.position.y = 0
                    theta = 0
                    quaternion = quaternion_from_euler(0.0, 0.0, theta)
                    nav_msg.pose.orientation.x = quaternion[0]
                    nav_msg.pose.orientation.y = quaternion[1]
                    nav_msg.pose.orientation.z = quaternion[2]
                    nav_msg.pose.orientation.w = quaternion[3]
                    self.nav_pub.publish(nav_msg)
                    self.debugger_pilots_console(
                        msg="CANCELANDO CLICK", log_type="warn")
                else:
                    self.debugger_pilots_console(
                        msg="CLICK INVALIDO", log_type="err")

        else:
            self.debugger_pilots_console(
                msg="CAMERA NO CALIBRATED", log_type="err")


    # Aux function to informing in video feed some info
    def debugger_pilots_console(self, msg, log_type="info"):
        if log_type == "info":
            self.info_msg.type = Messages.INFO
        if log_type == "warn":
            self.info_msg.type = Messages.WARNING
        if log_type == "err":
            self.info_msg.type = Messages.ERROR
        self.debugger(DEBUG_LEVEL_0, msg, log_type = log_type)
        self.info_msg.data = msg
        self.info_web_pub.publish(self.info_msg)




    # Function to draw center red lines indicating the center of the bot view
    def draw_center_lines(self):
        for idx, label in enumerate(["LL", "C", "RR"]):
            if self.mono_params.calibration[label] is not None:
                p1, p2 = self.mono_params.calibration[label]["bot_center"]
                cv2.line(np.array(self.images[idx]), p1, p2, (0,0,255), 2)

    def run(self):
        
        while self.run_event.is_set():
            
            if self.stitch:
                
                # Check if got images
                if self.images is not None:

                    if self.stitch_all:
                        # Do stitching

                        if self.zoom_roi is not None: # If zoom, draw rectangle
                            xmin,ymin,xmax,ymax = self.zoom_roi
                            cv2.rectangle(self.images[1], (xmin, ymin), (xmax, ymax), (0,255,255), 2)

                        # Draw center lines of bot view
                        self.draw_center_lines()

                        # Stitch images
                        img_stitched = self.stitcher.stitch(self.images)

                        # Draw undistorted clickeable area
                        img_stitched = self.stitcher.draw_stitched_projections(img_stitched)

                        # Fit them to current resolution
                        self.result = self.stitcher.get_virtual_pan(img_src = img_stitched, pan_value = 101)
                    
                    else:
                        pan = self.pan*(-1) # because of inverted logic
                        if pan < 0:
                            img_stitched = self.stitcher.stitch([self.images[0], self.images[1], None])
                        else:
                            img_stitched = self.stitcher.stitch([None, self.images[1], self.images[2]])
                        
                        self.result = self.stitcher.get_virtual_pan(img_src = img_stitched, pan_value = pan)
            else:
                # do nothing
                time.sleep(0.1)
                # print("Doing nothing...")
