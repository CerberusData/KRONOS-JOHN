#!/usr/bin/env python3
# =============================================================================
import numpy as np
import cv2
import os

from vision.utils.vision_utils import insert_image
from vision.utils.vision_utils import overlay_image
from vision.utils.vision_utils import printlog

from python_utils.pysubscribers import VisualDebuggerSubscriber
from python_utils.pysubscribers import ExtrinsicSubscriber
from python_utils.pysubscribers import WaypointSuscriber
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
        self.sub_waypoint = WaypointSuscriber(
            parent_node=parent_node, 
            extrinsic=self.sub_extrinsic.extrinsic,
            intrinsic=self.sub_extrinsic.intrinsic)
        self.sub_bot = Robot(
            parent_node=parent_node)

        # ---------------------------------------------------------------------
        # User environment variables
        self._VISUAL_DEBUGGER = int(os.getenv(
            key="VISUAL_DEBUGGER", default=1))
        self._VISUAL_OVERLAY_CAMS = int(os.getenv(
            key="VISUAL_OVERLAY_CAMS", default=1))
        self._VISUAL_WAYPOINT = int(os.getenv(
            key="VISUAL_WAYPOINT", default=1))
        self._VISUAL_ZOOM = int(os.getenv(
            key="VISUAL_ZOOM", default=1))  
        self._VISUAL_OBJECT_DETECTOR = int(os.getenv(
            key="VISUAL_OBJECT_DETECTOR", default=1))
        self._VISUAL_COMPASS = int(os.getenv(
            key="VISUAL_COMPASS", default=1))
        self._GUI_GAME_OVER_SCREEN = int(os.getenv(
            key="GUI_GAME_OVER_SCREEN", default=1))
        self._GUI_STOP_SCREEN = int(os.getenv(
            key="GUI_STOP_SCREEN", default=1))
        self._GUI_UNDISTORD_AREA = int(os.getenv(
            key="GUI_UNDISTORD_AREA", default=1))
        self._GUI_NO_CALIBRATION_SCREEN = int(os.getenv(
            key="GUI_NO_CALIBRATION_SCREEN", default=1))

        self._IMGS_PATH = os.path.join(os.path.dirname(__file__), "resources")

        # ---------------------------------------------------------------------
        # Graphics Images
        self.comp_img_stop_sing = gui_image_overlayed(img_path=os.path.join(
            self._IMGS_PATH, "stop_sing.png"))
        self.comp_img_game_over = gui_image_overlayed(img_path=os.path.join(
            self._IMGS_PATH, "game_over.png"), sc_fc=0.3)
        self.comp_img_no_calibration = gui_image_overlayed(img_path=os.path.join(
            self._IMGS_PATH, "no_calibration.png"))

        # ---------------------------------------------------------------------

    def draw_components(self, imgs_dict):
        """ 
            Draw graphic and user components on image
            Methods:
                draw image components, graphic interface
            Arguments:
                imgs_dict: 'dict of cv2.math' dictionary with cameras images 
        """

        # ---------------------------------------------------------------------
        # SWITCH CAMERAS - SWITCH CAMERAS - SWITCH CAMERAS - SWITCH CAMERAS - S       
        if self.sub_bot.stream_rear_cam:
            imgs_dict["P"], imgs_dict["B"] = imgs_dict["B"].copy(), imgs_dict["C"]
            rcam = True
        else:
            imgs_dict["P"] = imgs_dict["C"].copy()
            rcam = False

        # ---------------------------------------------------------------------
        # OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS
        if self._VISUAL_OVERLAY_CAMS:
            self.draw_lateral_cams(imgs_dict, rcam=rcam)
        
        # ---------------------------------------------------------------------
        # INTRINSIC - INTRINSIC - INTRINSIC - INTRINSIC - INTRINSIC - INTRINSIC
        if self._GUI_UNDISTORD_AREA:
            self.draw_intrinsic(img=imgs_dict["P"])

        # ---------------------------------------------------------------------
        # WAYPOINT - WAYPOINT - WAYPOINT - WAYPOINT - WAYPOINT - WAYPOINT - WAY
        if self._VISUAL_WAYPOINT and not rcam:
            self.sub_waypoint.draw(img=imgs_dict["P"])

        # ---------------------------------------------------------------------
        # ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM -
        if self._VISUAL_ZOOM and not rcam and self.sub_bot.zoom:
            self.draw_zoom(img=imgs_dict["P"], src_img=imgs_dict["C"])

        # ---------------------------------------------------------------------
        # OBJECT DETECTION - OBJECT DETECTION - OBJECT DETECTION - OBJECT DETEC
        if self._VISUAL_OBJECT_DETECTOR:
            pass

        # ---------------------------------------------------------------------
        # COMPASS - COMPASS - COMPASS - COMPASS - COMPASS - COMPASS - COMPASS -
        if self._VISUAL_COMPASS:
            pass

        # ---------------------------------------------------------------------
        # NO CALIBRATION SCREEN - NO CALIBRATION SCREEN - NO CALIBRATION SCREEN
        if (self._GUI_NO_CALIBRATION_SCREEN and 
            self.sub_extrinsic.extrinsic.cams["C"] is None):
            self.comp_img_no_calibration.draw(imgs_dict["P"])

        # ---------------------------------------------------------------------
        # STOP SCREEN - STOP SCREEN - STOP SCREEN - STOP SCREEN - STOP SCREEN -
        if self._GUI_STOP_SCREEN:
            self.comp_img_stop_sing.draw(imgs_dict["P"])

        # ---------------------------------------------------------------------          
        # GAMEOVER SCREEN - GAMEOVER SCREEN - GAMEOVER SCREEN - GAMEOVER SCREEN
        if self._GUI_GAME_OVER_SCREEN:
            self.comp_img_game_over.draw(imgs_dict["P"])

        # ---------------------------------------------------------------------
        # DRAW MESSAGES DEBUGGER - DRAW MESSAGES DEBUGGER - DRAW MESSAGES DEBUG
        if self._VISUAL_DEBUGGER:
            self.sub_visual_debugger.draw(img=imgs_dict["P"])

        # ---------------------------------------------------------------------

    def draw_intrinsic(self, img, cam_label="C", thickness=2):
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
            cv2.drawContours(
                image=img, 
                contours=[self.sub_extrinsic.extrinsic.cams[
                    cam_label]["undistord_cnt"]], 
                contourIdx=0, color=(0, 0, 200), 
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

    def draw_compass(self, img):
        """ 
            Draw compass component on image
            Methods:
            Arguments:
        """
        pass

    def draw_zoom(self, img, src_img):

        x_min = int(self.sub_bot.zoom_roi[0]*src_img.shape[1])
        x_max = int(self.sub_bot.zoom_roi[2]*src_img.shape[1])
        y_min = int(self.sub_bot.zoom_roi[1]*src_img.shape[0])
        y_max = int(self.sub_bot.zoom_roi[3]*src_img.shape[0])

        # Get zoom image
        zoom_image = src_img[y_min:y_max, x_min:x_max, :]

        # Insert zoom over pilots image
        insert_image(original_image=img, inserted_image=zoom_image, 
            new_width=int(zoom_image.shape[1]*self.sub_bot.zoom_factor), 
            new_height=int(zoom_image.shape[0]*self.sub_bot.zoom_factor), 
            position='ll')

        # Draw rectangle of zoom in pilots image
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 255, 255), 2)

class gui_image_overlayed():

    def __init__(self, img_path, transparency=0.5, sc_fc=0.5, x=0, y=0, flip=False): 
        """ Initialize class components
        Args:
            img_path: `string` image path to overlay
            transparency: `float` transparency of image
            sc_fc: `float` image sacling factor
            bgr_bleeding: `list` with rgb bleeding values
        Returns:
        """

        self._overlayed_mask = None 
        self.transparency = transparency # Transparency of image
        self.sc_fc = sc_fc  # image scaling factor
        self.x_pos = 0
        self.y_pos = 0
        
        # If file exits then load image icon
        if os.path.isfile(img_path):
            self._overlayed_mask = cv2.imread(
                filename=img_path, flags=cv2.IMREAD_UNCHANGED)
            self._overlayed_mask = cv2.resize(src=self._overlayed_mask, 
                dsize=(int(self.x_pos + self._overlayed_mask.shape[1]*self.sc_fc), 
                       int(self.y_pos + self._overlayed_mask.shape[0]*self.sc_fc)))
            if flip:
                self._overlayed_mask = cv2.flip(
                    src=self._overlayed_mask, flipCode=-1) 
        else:
            printlog(msg="No file to load image at {}".format(img_path),
                msg_type="ERROR")

    def draw(self, img_src, position="cc", x_off=0.0, y_off=0.0):
        """ Draw class components
        Args:
            img_src: `cv2.math` input image to draw components
            position: `string` position to draw component component
                sl: superior left side
                sc: superior center side
                sr: superior right side
                cl: center left side
                cc: centered in center
                cr: center right side
                il: inferior left side
                ic: inferior center side
                ir: inferior right side
        Returns:
            img_src: `cv2.math` input image with components drawn
        """

        if self._overlayed_mask is None:
            return img_src
        
        margin = 10
        if position == "cc":
            pos_coord = (
                int(img_src.shape[1]*0.5 - self._overlayed_mask.shape[1]*0.5), 
                int(img_src.shape[0]*0.5 - self._overlayed_mask.shape[0]*0.5))
        elif position == "sl": 
            pos_coord =  (margin, margin)
        elif position == "sc": 
            pos_coord = (
                int(img_src.shape[1]*0.5 - self._overlayed_mask.shape[1]*0.5), 
                margin)
        elif position == "sr": 
            pos_coord = (
                int(img_src.shape[1] - self._overlayed_mask.shape[1]-margin), 
                margin) 
        elif position == "cl": 
            pos_coord = (
                margin, 
                int(img_src.shape[0]*0.5 - self._overlayed_mask.shape[0]*0.5))
        elif position == "cr": 
            pos_coord = (
                int(img_src.shape[1] - self._overlayed_mask.shape[1]-margin), 
                int(img_src.shape[0]*0.5 - self._overlayed_mask.shape[0]*0.5)) 
        elif position == "il": 
            pos_coord = (
                margin, 
                int(img_src.shape[0] - self._overlayed_mask.shape[0]-margin))    
        elif position == "ic": 
            pos_coord = (
                int(img_src.shape[1]*0.5 - self._overlayed_mask.shape[1]*0.5), 
                int(img_src.shape[0] - self._overlayed_mask.shape[0]-margin))   
        elif position == "ir": 
            pos_coord = (
                int(img_src.shape[1] - self._overlayed_mask.shape[1]-margin), 
                int(img_src.shape[0] - self._overlayed_mask.shape[0]-margin))  
        else:
            pos_coord = (
                int(img_src.shape[1]*(0.5 + x_off) - self._overlayed_mask.shape[1]*(0.5 + x_off)), 
                int(img_src.shape[0]*(0.5 + y_off) - self._overlayed_mask.shape[0]*(0.5 + y_off)))

        # Overlay game over image text
        img_src = overlay_image(l_img=img_src, s_img=self._overlayed_mask, 
            pos=pos_coord, transparency=self.transparency)

# =============================================================================