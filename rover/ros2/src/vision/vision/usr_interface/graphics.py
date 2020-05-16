#!/usr/bin/env python3
# =============================================================================
import cv2
import os

from usr_msgs.msg import VisualMessage

from vision.utils.vision_utils import insert_image
from vision.utils.vision_utils import printlog

from python_utils.pysubscribers import VisualDebuggerSubscriber
from python_utils.pysubscribers import ExtrinsicSubscriber
from python_utils.pysubscribers import Robot

# =============================================================================
class GraphicInterface():

    def __init__(self, parent_node, cam_labels=["C"]):
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
        # Subscribers
        self.sub_visual_debugger = VisualDebuggerSubscriber(
            parent_node=parent_node)
        self.sub_extrinsic = ExtrinsicSubscriber(
            parent_node=parent_node, 
            cam_labels=cam_labels)
        self.sub_bot = Robot(
            parent_node=parent_node)

        # ---------------------------------------------------------------------
        # User enviroment variables
        self._VISUAL_DEBUGGER = int(os.getenv(
            key="VISUAL_DEBUGGER", default=1))
        self._VISUAL_OVERLAY_CAMS = int(os.getenv(
            key="VISUAL_OVERLAY_CAMS", default=1))
        self._VISUAL_WAYPOINT = int(os.getenv(
            key="VISUAL_WAYPOINT", default=1))
        self._GUI_GAME_OVER_SCREEN = int(os.getenv(
            key="GUI_GAME_OVER_SCREEN", default=1))
        self._GUI_STOP_SCREEN = int(os.getenv(
            key="GUI_STOP_SCREEN", default=1))
        self._GUI_WAYPOINT_AREA = int(os.getenv(
            key="GUI_WAYPOINT_AREA", default=1))
        self._GUI_UNDISTORD_AREA = int(os.getenv(
            key="GUI_UNDISTORD_AREA", default=1))

        self._IMGS_PATH = os.path.join(os.path.dirname(__file__), "resources")

        # ---------------------------------------------------------------------
        # Graphics Images
        self.stop_sing = cv2.imread(filename=os.path.join(
            self._IMGS_PATH, "stop_sing.png"), 
            flags=cv2.IMREAD_UNCHANGED)
        self.game_over = cv2.imread(filename=os.path.join(
            self._IMGS_PATH, "game_over.png"), 
            flags=cv2.IMREAD_UNCHANGED)

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
        rcam = False
        if self.sub_bot.stream_rear_cam:
            imgs_dict["P"] = imgs_dict["B"].copy()
            rcam = True
        else:
            imgs_dict["P"] = imgs_dict["C"].copy()

        # ---------------------------------------------------------------------
        # OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS
        if self._VISUAL_OVERLAY_CAMS:
            self.draw_lateral_cams(imgs_dict, rcam=rcam)
        
        # ---------------------------------------------------------------------
        # DRAW MESSAGES DEBUGGER - DRAW MESSAGES DEBUGGER - DRAW MESSAGES DEBUG
        if self._VISUAL_DEBUGGER:
            self.sub_visual_debugger.draw(img=imgs_dict["P"])

        # ---------------------------------------------------------------------
        # WAYPOINT - WAYPOINT - WAYPOINT - WAYPOINT - WAYPOINT - WAYPOINT - WAY
        if self._VISUAL_WAYPOINT:
            self.draw_extrinsic(img=imgs_dict["P"], cam_label="C")

        # ---------------------------------------------------------------------
        # ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM -

        # ---------------------------------------------------------------------
        # OBJECT DETECTION - OBJECT DETECTION - OBJECT DETECTION - OBJECT DETEC

        # ---------------------------------------------------------------------
        # NO CALIBRATION SCREEN - NO CALIBRATION SCREEN - NO CALIBRATION SCREEN

        # ---------------------------------------------------------------------
        # COMPASS - COMPASS - COMPASS - COMPASS - COMPASS - COMPASS - COMPASS -

        # ---------------------------------------------------------------------
        # STOP SCREEN - STOP SCREEN - STOP SCREEN - STOP SCREEN - STOP SCREEN -
        if self._GUI_STOP_SCREEN:
            pass

        # ---------------------------------------------------------------------          
        # GAMEOVER SCREEN - GAMEOVER SCREEN - GAMEOVER SCREEN - GAMEOVER SCREEN
        if self._GUI_GAME_OVER_SCREEN:
            pass

        # ---------------------------------------------------------------------

    def draw_extrinsic(self, img, cam_label="C", thickness=2):
        """ 
            Draw extrinsic calibration components
            Args:
                img: 'cv2.math' image to draw components
                cam_label: 'string' camera label to get extrinsic components
            returns:
        """
        
        if cam_label in self.sub_extrinsic.extrinsic.cams.keys():
            if self.sub_extrinsic.extrinsic.cams[cam_label] is None:
                return

            # Draw surface perspective projection contour
            if self._GUI_UNDISTORD_AREA:
                cv2.drawContours(
                    image=img, 
                    contours=[self.sub_extrinsic.extrinsic.cams[
                        cam_label]["undistord_cnt"]], 
                    contourIdx=0, color=(0, 0, 200), 
                    thickness=thickness)

            # Draw waypoint area contour
            if self._GUI_WAYPOINT_AREA:
                cv2.drawContours(
                    image=img, 
                    contours=[self.sub_extrinsic.extrinsic.cams[
                        cam_label]["waypoint_area"]], 
                    contourIdx=0, color=(0, 200, 0), 
                    thickness=thickness)

    def draw_lateral_cams(self, imgs_dict, rcam):
        """ 
            Draw lateral cameras components on image
            Methods:
            Arguments:
                imgs_dict: 'dict of cv2.math' draw image components, 
                    graphic interface
        """

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

# =============================================================================