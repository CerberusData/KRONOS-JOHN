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
from python_utils.pysubscribers import WaypointSubscriber
from python_utils.pysubscribers import WebclientControl
from python_utils.pysubscribers import RobotSubscriber

import time

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
        self.sub_waypoint = WaypointSubscriber(
            parent_node=parent_node, 
            extrinsic=self.sub_extrinsic.extrinsic,
            intrinsic=self.sub_extrinsic.intrinsic)
        self.sub_bot = RobotSubscriber(
            parent_node=parent_node)
        self.sub_webclient_control = WebclientControl(
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
        self._VISUAL_LAT_CAMS_IDLE_TIME = int(os.getenv(
            key="VISUAL_LAT_CAMS_IDLE_TIME", default=1))
        self._VISUAL_REAR_CAM_IDLE_TIME = int(os.getenv(
            key="VISUAL_REAR_CAM_IDLE_TIME", default=1))
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
        self.timer_tick_lateral_cam = time.time()
        self.timer_tick_rear_cam = time.time()

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
        # Robot properties
        throttle = self.sub_webclient_control.throttle
        pan = self.sub_webclient_control.pan
        tilt = self.sub_webclient_control.tilt
        rcam = False

        # ---------------------------------------------------------------------
        # Conditionals and logic
        
        # if robot moves fordware
        if throttle > 0:
            self.sub_bot.zoom = False # Take off zoom if robot moves
            self.timer_tick_lateral_cam = time.time() # reset timer
        # if robot moves backwards
        elif throttle < 0:
            self.timer_tick_rear_cam = time.time() # reset timer
    
        # ---------------------------------------------------------------------
        # SWITCH CAMERAS - SWITCH CAMERAS - SWITCH CAMERAS - SWITCH CAMERAS - S              
        if self.sub_bot.stream_rear_cam:
            imgs_dict["P"], imgs_dict["B"] = imgs_dict["B"].copy(), imgs_dict["C"]
            rcam = True
        elif pan > 0:
            imgs_dict["P"] = imgs_dict["RR"]
            return
        elif pan < 0:
            imgs_dict["P"] = imgs_dict["LL"]
            return
        else:
            imgs_dict["P"] = imgs_dict["C"].copy()
        
        # ---------------------------------------------------------------------
        # OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS - OVERLAY OTHER CAMERAS
        if self._VISUAL_OVERLAY_CAMS:
            tock = time.time() - self.timer_tick_lateral_cam
            tock_rcam = time.time() - self.timer_tick_rear_cam
            self.draw_lateral_cams(
                imgs_dict=imgs_dict,  
                show_rcam=tock_rcam < self._VISUAL_REAR_CAM_IDLE_TIME,
                show_ltcams=tock < self._VISUAL_LAT_CAMS_IDLE_TIME,
                switched_cam=rcam)
                        
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
        if (self._VISUAL_ZOOM and self.sub_bot.zoom and not rcam):
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
        if self._GUI_STOP_SCREEN and self.sub_bot.door_open:
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

    def draw_lateral_cams(self, imgs_dict, switched_cam, show_rcam, show_ltcams):
        """ 
            Draw lateral cameras components on image
            Methods:
            Arguments:
                imgs_dict: 'dict of cv2.math' draw image components, 
                    graphic interface
        """

        if switched_cam:
            if "RR" in imgs_dict.keys() and "LL" in imgs_dict.keys():
                imgs_dict["RR"], imgs_dict["LL"] = imgs_dict["LL"], imgs_dict["RR"]

        if "RR" in imgs_dict.keys() and show_ltcams:
            insert_image(
                original_image=imgs_dict["P"], inserted_image=imgs_dict["RR"], 
                new_width=int(imgs_dict["P"].shape[1]*0.3), 
                new_height=int(imgs_dict["P"].shape[0]*0.3), 
                position='ur',
                border_color=(255,255,255) if self.sub_extrinsic.extrinsic.cams[
                    "RR"] is not None else (0, 0, 255))
        if "LL" in imgs_dict.keys() and show_ltcams:
            insert_image(
                original_image=imgs_dict["P"], inserted_image=imgs_dict["LL"], 
                new_width=int(imgs_dict["P"].shape[1]*0.3), 
                new_height=int(imgs_dict["P"].shape[0]*0.3), 
                position='ul',
                border_color=(255,255,255) if self.sub_extrinsic.extrinsic.cams[
                    "LL"] is not None else (0, 0, 255))
        if "B" in imgs_dict.keys() and show_rcam:
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

class gui_cliff_sensors():

    def __init__(self, topic_list):
        """ Initialize cliff sensors components 
        Args:
            topic_list: 'list' list of cliff sensors topics
        Returns:
        """

        self._sensors = [CliffSensorSuscriber(
            topic_name=sensor_topic) for sensor_topic in topic_list]
        self._CLIFF_SENSOR_TRESHOLD = float(os.getenv("CLIFF_SENSOR_TRESHOLD", 0.55))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640)) 
        self._sensor_idx = int(self._VIDEO_WIDTH/(len(self._sensors)+1))
        self._y_offset = 100
        self._x_offset = 0
        self._x_offsets = []
        self._radius = 150
        self._color = (0, 0, 255)
        self._thickness = 2
        self._inner_circles = 5
        self._inner_cileres_step = 10

        if len(self._sensors) == 2:
            self._x_offsets = [-50, 50]
        if len(self._sensors) == 3:
            self._x_offsets = [-50, 0,  50]

    def draw(self, img_src):
        """ Draw cliff sensors components
        Args:
            img_src: 'cv2.math' image to draw component
        Returns:
        """

        for idx, sensor in enumerate(self._sensors):
            if sensor.range > self._CLIFF_SENSOR_TRESHOLD:
                self.draw_sensor(img_src=img_src, sensor=sensor, 
                    idx=(int(self._sensor_idx*(idx+1))),
                    x_offset=self._x_offsets[idx] if len(self._x_offsets) else 0)

    def draw_sensor(self, img_src, sensor, idx, x_offset=0):
        """ Draw cliff sensor component
        Args:
            img_src: 'cv2.math' image to draw component
        Returns:
        """

        for circle_idx in range(self._inner_circles):
            radius = self._radius-self._inner_cileres_step*circle_idx
            cv2.circle(img=img_src, 
                center=(
                    idx + self._x_offset + x_offset, 
                    self._VIDEO_HEIGHT + self._y_offset), 
                radius=radius if radius > 0 else 1, 
                color=self._color, 
                thickness=self._thickness) 
        
class gui_distance_sensors():

    def __init__(self, topic_list):
        """ Initialize distance sensors components 
        Args:
            topic_list: 'list' list of distance sensors topics
        Returns:
        """

        self._sensors = [DistanceSensorSuscriber(
            topic_name=sensor_topic) for sensor_topic in topic_list]

        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._GUI_SENSORS_DISTANCE_MEASURE = int(os.getenv(
            key="GUI_SENSORS_DISTANCE_MEASURE", default=1))
        self._GUI_SENSORS_DISTANCE_LONG = float(os.getenv(
            key="GUI_SENSORS_DISTANCE_LONG", default=150))
        
        self._angle_start = int(90 - int(os.getenv(
            key="GUI_SENSORS_DISTANCE_APERTURE_ANGLE", default=30))*0.5)
        self._angle_end = int(90 + int(os.getenv(
            key="GUI_SENSORS_DISTANCE_APERTURE_ANGLE", default=30))*0.5)
        self._angle_start_rad = np.deg2rad(self._angle_start)
        self._angle_end_rad = np.deg2rad(self._angle_end)
        self._angle_tan = math.tan(self._angle_start_rad)
        self._sin_tan = math.sin(self._angle_start_rad)

        self._font = cv2.FONT_HERSHEY_SIMPLEX
        self._font_scale = 0.8
        self._font_thickness = 1
        self._sensor_idx = int(self._VIDEO_WIDTH/(len(self._sensors)+1))
        self._y_offset = 20
        self._x_offset = 0
        self._x_offsets = []
        self._angle = 30
        self._color = (255, 255, 255)
        self._thickness = 2

        if len(self._sensors) == 2:
            self._x_offsets = [-50, 50]
        if len(self._sensors) == 3:
            self._x_offsets = [-50, 0,  50]

    def draw(self, img_src):
        """ Draw distance sensors components
        Args:
            img_src: 'cv2.math' image to draw component
        Returns:
        """

        for idx, sensor in enumerate(self._sensors):
            if sensor.range >= sensor.min_range:
                self.draw_sensor(img_src=img_src, sensor=sensor, 
                    idx=(int(self._sensor_idx*(idx+1))),
                    x_offset=self._x_offsets[idx] if len(self._x_offsets) else 0)

    def draw_sensor(self, img_src, sensor , idx, x_offset=0):
        """ Draw distance sensor component
        Args:
            img_src: 'cv2.math' image to draw component
        Returns:
        """

        x_idx = idx + self._x_offset + x_offset
        y_idx = self._y_offset + self._VIDEO_HEIGHT
        l_dist = int(self._GUI_SENSORS_DISTANCE_LONG*sensor.range/sensor.max_range)

        x = l_dist/self._angle_tan

        # Draw right line
        cv2.line(
            img=img_src, 
            pt1=(x_idx, y_idx),
            pt2=(
                int(x_idx + x), 
                int(self._y_offset + self._VIDEO_HEIGHT - self._sin_tan*l_dist)),
            color=self._color, 
            thickness=self._thickness) 

        # Draw left line
        cv2.line(
            img=img_src, 
            pt1=(x_idx, y_idx),
            pt2=(
                int(x_idx - x), 
                int(self._y_offset + self._VIDEO_HEIGHT - self._sin_tan*l_dist)),
            color=self._color, 
            thickness=self._thickness) 

        # Draw elipse to complete haz
        cv2.ellipse(
            img=img_src, 
            center=(x_idx, y_idx), 
            axes=(l_dist, l_dist), 
            angle=180, 
            startAngle=self._angle_start, 
            endAngle=self._angle_end, 
            color=self._color, 
            thickness=self._thickness) 

        if self._GUI_SENSORS_DISTANCE_MEASURE:
            self.draw_text(img=img_src, 
                text="{}m".format(round(sensor.range, 2)), 
                org=(x_idx - 35, self._VIDEO_HEIGHT - 20), 
                color=self._color)

    def draw_text(self, img, text, org, color):
        cv2.putText(img=img, text=text, org=org, fontFace=self._font, 
            fontScale=self._font_scale, color=(0, 0, 0), thickness=self._font_thickness*4)
        cv2.putText(img=img, text=text, org=org, fontFace=self._font, 
            fontScale=self._font_scale, color=color, thickness=self._font_thickness) 

class gui_chassi_report(gui_component_base):
    
    def __init__(self):
        """ Initialize class components
        Args:
        Returns:
        """

        super(gui_chassi_report, self).__init__()

        self.robot_chassi = gui_image_overlayed(
            img_path=os.path.join(figures_path, "robot_chassi.png"),
            sc_fc=1., transparency=1.0)

        # 4 - 1
        # 3 - 2
        self.wheel_1 = gui_image_overlayed(
            img_path=os.path.join(figures_path, "wheel_error.png"),
            sc_fc=1., transparency=1.0, flip=1)
        self.wheel_2 = gui_image_overlayed(
            img_path=os.path.join(figures_path, "wheel_error.png"),
            sc_fc=1., transparency=1.0, flip=1)
        self.wheel_3 = gui_image_overlayed(
            img_path=os.path.join(figures_path, "wheel_error.png"),
            sc_fc=1., transparency=1.0)
        self.wheel_4 = gui_image_overlayed(
            img_path=os.path.join(figures_path, "wheel_error.png"),
            sc_fc=1., transparency=1.0)
        self.module = gui_image_overlayed(
            img_path=os.path.join(figures_path, "module_error.png"),
            sc_fc=1., transparency=1.0)

        self.transp=0.01
        self.subs_chassis = ChassisSuscriber()

    def draw(self, img_src):

        if True in self.subs_chassis.error:

            if self.wheel_2.transparency > 0.7: self.transp = -0.1
            elif self.wheel_2.transparency < 0.2: self.transp = 0.1 
                
            self.wheel_1.transparency += self.transp
            self.wheel_2.transparency += self.transp
            self.wheel_3.transparency += self.transp
            self.wheel_4.transparency += self.transp
            self.module.transparency += self.transp

            self.robot_chassi.draw(img_src=img_src)
            
            if self.subs_chassis.error[0]:
                self.wheel_1.draw(img_src=img_src, position="", 
                    x_off=0.095, y_off=0.255)
            if self.subs_chassis.error[1]:
                self.wheel_2.draw(img_src=img_src, position="", 
                    x_off=0.095, y_off=-0.255)
            if self.subs_chassis.error[2]:
                self.wheel_3.draw(img_src=img_src, position="", 
                    x_off=-0.095, y_off=0.255)
            if self.subs_chassis.error[3]:
                self.wheel_4.draw(img_src=img_src, position="", 
                    x_off=-0.095, y_off=-0.255)
            if self.subs_chassis.error[-1]:
                self.module.draw(img_src=img_src)

# =============================================================================