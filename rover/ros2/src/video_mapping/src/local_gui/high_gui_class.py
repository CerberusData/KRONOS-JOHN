# =============================================================================
"""
Code Information:
	Programmer: Eng. John Betancourt G.
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer Vision Team

Description:
    Class to declare and draw high gui components in image
"""

# =============================================================================
# LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPENDENCIES - LIBRARIES AND DEPEN
# =============================================================================
from __future__ import print_function

import numpy as np
import time
import math
import os

import cv2

from vision.utils import overlay_image

from python_utils.utils import get_projection_point_src
from python_utils.utils import print_text_list
from python_utils.utils import get_distor_point
from python_utils.utils import dotline
from python_utils.mono_params import MonoParams
from python_utils.suscribers import ActuatorReferenceSuscriber
from python_utils.suscribers import ActuatorControlSuscriber
from python_utils.suscribers import DataCaptureSuscriber
from python_utils.suscribers import WanNetworksSuscriber
from python_utils.suscribers import NavigationSuscriber
from python_utils.suscribers import WebClientSuscriber
from python_utils.suscribers import WaypointSuscriber
from python_utils.suscribers import OdometrySuscriber
from python_utils.suscribers import KiwiBotSuscriber
from python_utils.suscribers import BatterySuscriber
from python_utils.suscribers import ChassisSuscriber
from python_utils.suscribers import CameraSuscriber
from python_utils.suscribers import PWMoutSuscriber
from python_utils.suscribers import SkynetSuscriber
from python_utils.suscribers import VisionSuscriber
from python_utils.suscribers import VisualDebugger
from python_utils.suscribers import CliffSensorSuscriber
from python_utils.suscribers import DistanceSensorSuscriber

from utils.node_utils import Zoomer
from utils.node_utils import visualize_boxes_on_image
from utils.node_utils import draw_chess_board
from utils.node_utils import draw_pad_centers 
from utils.node_utils import predictions_warning_labels
from utils.node_utils import show_msg
from utils.node_utils import insert_image
from utils.node_utils import apply_virtual_pan
from utils.node_utils import world_to_projections
from utils.node_utils import handle_zoom_command
from utils.node_utils import get_zoom_image
from utils.node_utils import update_roi
from utils.node_utils import render_text

from stitching.stitching_class import Stitcher
from stitching.stitching_class import image_resize

# =============================================================================
figures_path = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "resources")
LOCAL_LAUNCH = int(os.getenv("LOCAL_LAUNCH", 0)) 
FR_AGENT = int(os.getenv("FR_AGENT", 0)) 

def nothing(x): pass

# =============================================================================
class high_gui():
    
    def __init__(self, cam_labels=["C"]):
        """ Initialize class components
        Args:
        cam_params
        Returns:
        """

        # ---------------------------------------------------------------------
        # Other GUI components
        self._GUI_NO_CALIBRATION_SCREEN = int(os.getenv(key="GUI_NO_CALIBRATION_SCREEN", default=1))
        self._GUI_GAME_OVER_SCREEN = int(os.getenv(key="GUI_GAME_OVER_SCREEN", default=1))
        self._GUI_STOP_SCREEN = int(os.getenv(key="GUI_STOP_SCREEN", default=1))
        self._GUI_PROJECTION = int(os.getenv(key="GUI_PROJECTION", default=1))
        self._GUI_COMPASS = int(os.getenv(key="GUI_COMPASS", default=1))
        self._GUI_BATTERY_ICON = int(os.getenv(key="GUI_BATTERY_ICON", default=1))
        self._GUI_WARNING_LABELS = int(os.getenv(key="GUI_WARNING_LABELS", default=1))
        self._GUI_OBJECT_DETECTION = int(os.getenv(key="GUI_OBJECT_DETECTION", default=1))
        self._GUI_VISUAL_DEBUG_TIME = int(os.getenv(key="GUI_VISUAL_DEBUG_TIME", default=10))
        self._GUI_AUTO_ZOOM = int(os.getenv(key="GUI_AUTO_ZOOM", default=0))
        self._GUI_CAMS_IN_TURN = int(os.getenv(key="GUI_CAMS_IN_TURN", default=1))
        self._GUI_CHASSI = int(os.getenv("GUI_CHASSI", 1))
        self._GUI_DATA_CAPTURE = int(os.getenv("GUI_DATA_CAPTURE", 0))

        # Sensors
        self._GUI_SENSORS_CLIFF = int(os.getenv("GUI_SENSORS_CLIFF", 0))
        self._GUI_SENSORS_DISTANCE = int(os.getenv("GUI_SENSORS_DISTANCE", 0))

        # Vision
        self._VISION_CALIBRATION_DAYS_OUT = int(os.getenv(key="VISION_CALIBRATION_DAYS_OUT", default=10))                 
        self._VISION_CALIBRATION_DAYS_REMOVE = int(os.getenv(key="VISION_CALIBRATION_DAYS_REMOVE", default=20))
        self._VISION_SHOW_CAMERAS_CALIBRATION = int(os.getenv("VISION_SHOW_CAMERAS_CALIBRATION", 0))
        
        # Pepwave
        self._PEPWAVE_SHOW_GUI = int(os.getenv("PEPWAVE_SHOW_GUI", 0))
        
        # ---------------------------------------------------------------------
        # Create calibration elements
        self.mono_params = MonoParams(cameras=cam_labels, dead_view_srv=True)
        if self._VISION_SHOW_CAMERAS_CALIBRATION: print(self.mono_params)

        # ---------------------------------------------------------------------
        # Create gui components
        self.warning_labels = gui_warning_labels()

        self.component_compass = gui_compass()
        self.component_waypoint_area = gui_waypoint_area()
        self.component_bot_projection = gui_body_projection()
        self.component_gui_chassi_report = gui_chassi_report()

        bgr_bleeding=[float(elem_str) for elem_str in 
            os.getenv("GUI_GAMEOVER_BLEEDING", "0.7,0.7,1").split(",")]
        self.component_game_over_screen = gui_image_overlayed(
            img_path = os.path.join(figures_path, "game_over_mask.png"),
            transparency=float(os.getenv("GUI_GAMEROVER_TRANSP", 0.35)), 
            sc_fc=float(os.getenv("GUI_GAMEROVER_SCFC", 0.5)), 
            bgr_bleeding=bgr_bleeding
            )

        bgr_bleeding=[float(elem_str) for elem_str in 
            os.getenv("GUI_STOP_SCREEN_BLEEDING", "1,0.7,0.7").split(",")]
        self.component_stop_screen = gui_image_overlayed(
            img_path = os.path.join(figures_path, "stop_sing.png"),
            transparency=float(os.getenv("GUI_STOP_SCREEN_TRANSP", 0.35)), 
            sc_fc=float(os.getenv("GUI_STOP_SCREEN_SCFC", 0.8)), 
            bgr_bleeding=bgr_bleeding
            )

        bgr_bleeding=[float(elem_str) for elem_str in 
            os.getenv("GUI_NO_CALIBRATION_SCREEN_BLEEDING", "0.7,0.8,1.").split(",")]
        self.component_no_calibration_screen = gui_image_overlayed(
            img_path = os.path.join(figures_path, "no_calibration.png"),
            transparency=float(os.getenv("GUI_NO_CALIBRATION_SCREEN_TRANSP", 0.35)), 
            sc_fc=float(os.getenv("GUI_NO_CALIBRATION_SCREEN_SCFC", 0.8)), 
            bgr_bleeding=bgr_bleeding
            )
        
        self.component_wan_Status = gui_wan_networks_status_report()

        # Sensors
        self.component_cliff_Sensors = gui_cliff_sensors(
            topic_list=[
                "/tf_mini_plus/cliff_sensor2", 
                "/tf_mini_plus/cliff_sensor1"]
        )
        self.component_distance_Sensors = gui_distance_sensors(
            topic_list=[
                "/tf_mini_plus/distance_sensor3",
                "/tf_mini_plus/distance_sensor2",
                "/tf_mini_plus/distance_sensor1"]
        )

        # ---------------------------------------------------------------------
        # Subscribers
        self.subs_followpad = VisionSuscriber(topic="/vision/followmode/detection")
        self.subs_waypoint = WaypointSuscriber(mono_params=self.mono_params)
        self.subs_actuatorreference = ActuatorReferenceSuscriber()
        self.subs_actuatorcontrol = ActuatorControlSuscriber()
        self.subs_data_capture_status = DataCaptureSuscriber()
        self.subs_visual_debugger = VisualDebugger(timer=3)
        self.subs_navigation = NavigationSuscriber()
        self.subs_web_client = WebClientSuscriber()
        self.subs_rear_camera = CameraSuscriber()
        self.subs_odometry = OdometrySuscriber()
        self.subs_battery = BatterySuscriber()
        self.subs_pwm_out = PWMoutSuscriber()
        self.subs_skynet = SkynetSuscriber()
        self.subs_bot = KiwiBotSuscriber()
        
        # ---------------------------------------------------------------------
        # Variables for GUI components draws
        self.VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
        self.VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640)) 
        self.GUI_BACK_WIN_FC = float(os.getenv(key="GUI_BACK_WIN_FC", default=0.4)) 
        self.GUI_WIN_LATERAL_CUTOFF = int(os.getenv(key="GUI_WIN_LATERAL_CUTOFF", default=150)) 
        self.back_win_size = (
            int(self.VIDEO_WIDTH*self.GUI_BACK_WIN_FC), 
            int(self.VIDEO_HEIGHT*self.GUI_BACK_WIN_FC))

        # Zoom objects for handling zoom regions
        self.zoomer_x = Zoomer()
        self.zoomer_y = Zoomer()

        # Assing lateral_cut for virtual pan depending on camera resolution => 4:3 or 16:9
        self.aspect_ratio = self.VIDEO_WIDTH/float(self.VIDEO_HEIGHT)
        self.lateral_cut = 0 if self.aspect_ratio == 4/3.0 else self.GUI_WIN_LATERAL_CUTOFF

        # ---------------------------------------------------------------------
        # Stitcher variables
        stitcher_conf_path = os.getenv(key = "STITCHER_PATH", default="/usr/src/app/")  
        super_stitcher =  int(os.getenv(key = "STITCHER_SUPER_MODE", default=0))
        self.stitch_video = int(os.getenv(key = "STITCHER_VIDEO", default=1))
        self.stitcher = None
        if self.stitch_video:
            self.stitcher = Stitcher(abs_path=os.path.join(stitcher_conf_path, 
                "stitcher_config.npz"), super_stitcher=super_stitcher)
            self.stitcher.set_proj_cnts(cams_params=self.mono_params.calibration)

    def draw_components(self, imgs_dict):
        
        # ---------------------------------------------------------------------
        pan = 0 if self.subs_bot.zoom else self.subs_web_client.pan
        roll =self.subs_odometry.roll
        pitch = self.subs_odometry.pitch
        auto_zoom_roi = None

        # ---------------------------------------------------------------------
        if "LL" not in imgs_dict.keys() and pan < 0: pan = 0
        if "RR" not in imgs_dict.keys() and pan > 0: pan = 0
        if "B" not in imgs_dict.keys(): imgs_dict["B"] = imgs_dict["C"]
        img_src = imgs_dict["C"].copy() if (not self.subs_rear_camera.switch_camera
            ) else imgs_dict["B"].copy()
    
        # ---------------------------------------------------------------------
        # WAN NETWORKS STATUS - WAN NETWORKS STATUS - WAN NETWORKS STATUS - WAN
        if not self.subs_visual_debugger.msg and self._PEPWAVE_SHOW_GUI: 
            self.component_wan_Status.draw(img_src=img_src)

        # ---------------------------------------------------------------------
        # CHASSIS ISSUES REPORT - CHASSIS ISSUES REPORT - CHASSIS ISSUES REPORT
        if self._GUI_CHASSI :
            self.component_gui_chassi_report.draw(img_src=img_src)

        # ---------------------------------------------------------------------
        # CLIFF SENSORS - CLIFF SENSORS - CLIFF SENSORS - CLIFF SENSORS - CLIFF
        if self._GUI_SENSORS_CLIFF:
            self.component_cliff_Sensors.draw(img_src=img_src)

        # ---------------------------------------------------------------------
        # DISTANCE SENSORS - DISTANCE SENSORS - DISTANCE SENSORS - DISTANCE SEN
        if self._GUI_SENSORS_DISTANCE:
            self.component_distance_Sensors.draw(img_src=img_src)

        # ---------------------------------------------------------------------
        # STITCHER - STITCHER - STITCHER - STITCHER - STITCHER - STITCHER - STI
        if self.stitch_video and self.subs_bot.stitch:
            if set(imgs_dict.keys()) == set(["C", "B", "LL", "RR"]):
                img_stitched = self.stitcher.stitch(
                    images=[imgs_dict["LL"], imgs_dict["C"], imgs_dict["RR"]])
                self.stitcher.draw_stitched_projections(img_src=img_stitched)
                return image_resize(image=img_stitched, 
                    width=self.VIDEO_WIDTH, height=self.VIDEO_HEIGHT)

        # ---------------------------------------------------------------------
        # VIRTUAL PAN - VIRTUAL PAN - VIRTUAL PAN - VIRTUAL PAN - VIRTUAL PAN -
        img_src = apply_virtual_pan(
            switch_cams=self.subs_rear_camera.switch_camera,  
            lateral_cutoff=self.lateral_cut, 
            images_dict=imgs_dict, 
            img_src=img_src, 
            pan=pan)

        # ---------------------------------------------------------------------
        # ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM - ZOOM -       
        if pan == 0:
            # traffic lights autozoom 
            if len(self.subs_skynet.object_detection) and self._GUI_AUTO_ZOOM :
                if "C" in self.subs_skynet.object_detection:
                    traffic_lights = list(filter(lambda detection: detection["name"]=='traffic light', 
                        self.subs_skynet.object_detection["C"]["predictions"]))
                    if len(traffic_lights):
                        ymin, xmin, ymax, xmax = traffic_lights[0]["box"]
                        xmin, xmax = xmin/image_size[0], xmax/image_size[0]
                        ymin, ymax = ymin/image_size[1], ymax/image_size[1]
                        auto_zoom_roi = (xmin, ymin, xmax, ymax)
                        self.subs_bot.zoom = True
                        
            # Robot has a reference in waypoint mode
            if self.subs_actuatorcontrol.throttle > 0.0:
                self.subs_bot.zoom = False
                auto_zoom_roi = None
    
            # if there is zoom, show that part of the image and handle commands
            if self.subs_bot.zoom:

                if auto_zoom_roi is not None:
                    xmin, ymin, xmax, ymax = auto_zoom_roi
                    box_center = (xmin + (xmax - xmin)*0.5, ymin + (ymax - ymin)*0.5)
                    zoom_roi_center = ( # [%]xmin, [%]ymin, [%]xmax, [%]ymax
                        self.subs_bot.zoom_roi[0] + (self.subs_bot.zoom_roi[2] - self.subs_bot.zoom_roi[0])*0.5,
                        self.subs_bot.zoom_roi[1] + (self.subs_bot.zoom_roi[3] - self.subs_bot.zoom_roi[1])*0.5)
                    dx = zoom_roi_center[0] - box_center[0]
                    dy = zoom_roi_center[1] - box_center[1]

                    if abs(dx) > 0.:
                        dx_val = int(dx/abs(dx)); dx = abs(dx)
                        self.subs_bot.zoom_roi = update_roi(
                            roi=self.subs_bot.zoom_roi, val=dx_val, 
                            base_move=dx, direction="x")
                    if abs(dy) > 0.:
                        dy_val = int(dy/abs(dy)); dy = abs(dy)
                        self.subs_bot.zoom_roi = update_roi(
                            roi=self.subs_bot.zoom_roi, val=dy_val, 
                            base_move=dy, direction="y")

                # Get ROI or image of zoom region
                zoom_image = get_zoom_image(hd_image=img_src, 
                    zoom_roi=self.subs_bot.zoom_roi, hd_image_flipped=False)
                zoom_image = cv2.resize(src=zoom_image, 
                    dsize=(int(zoom_image.shape[1]*3), int(zoom_image.shape[0]*3)))
            
                # Insert zoom over pilots image
                insert_image(original_image=img_src, inserted_image=zoom_image, 
                    new_width=zoom_image.shape[1], new_height=zoom_image.shape[0], 
                    position='ll')

                # transform to absolute pixels
                xmin, ymin, xmax, ymax = self.subs_bot.zoom_roi
                xmin_f, ymin_f, xmax_f, ymax_f = (
                    int(xmin*(img_src.shape[1] - 1)), 
                    int(ymin*(img_src.shape[0] - 1)), 
                    int(xmax*(img_src.shape[1] - 1)), 
                    int(ymax*(img_src.shape[0] - 1)))
                
                # Draw rectangle of zoom in pilots image
                cv2.rectangle(img_src, (xmin_f, ymin_f), (xmax_f, ymax_f), 
                    (0, 255, 255) if auto_zoom_roi is None else (0, 0, 255), 2)

                # update zoom roi according to incoming commands
                self.subs_bot.zoom_roi = handle_zoom_command(
                    val=self.subs_web_client.pan, zoom_obj=self.zoomer_x, 
                    move_rel=0.05, roi=self.subs_bot.zoom_roi, direction="x")
                self.subs_bot.zoom_roi = handle_zoom_command(
                    val=self.subs_web_client.tilt, zoom_obj=self.zoomer_y, 
                    move_rel=0.125, roi=self.subs_bot.zoom_roi, direction="y")
            
        # ---------------------------------------------------------------------
        # REAR CAMERA - REAR CAMERA - REAR CAMERA - REAR CAMERA - REAR CAMERA -
        
        # If the bot is going backwards or rear camera is enabled, 
        # show rear camera
        if (self.subs_web_client.throttle < 0 
                or self.subs_rear_camera.enable_camera):
            # flip camera only if it was recognized
            inserted_image = np.fliplr(imgs_dict["B"] if not self.subs_rear_camera.switch_camera else imgs_dict["C"]) if (
                not self.subs_rear_camera.switch_camera) else imgs_dict["B"] if not self.subs_rear_camera.switch_camera else imgs_dict["C"]

            # Insert rear camera in pilots image
            insert_image(
                original_image=img_src, inserted_image=inserted_image, 
                new_width=self.back_win_size[0], new_height=self.back_win_size[1], 
                position='ur')

            # Draw rear camera text if rear camera image is on the top
            if self.subs_rear_camera.switch_camera: 
                show_msg(image=img_src, msg="Camara Trasera", msg_type=None, 
                left=10, top=300, color=(0, 200, 255))

        # ---------------------------------------------------------------------
        # SHOW LATERAL CAMERAS WHEN TURNING - SHOW LATERAL CAMERAS WHEN TURNING
        if (self._GUI_CAMS_IN_TURN 
            and self.subs_web_client.throttle >= 0.0
            and self.subs_actuatorreference.steering != abs(0.0)
            and pan == 0
            and self.subs_navigation.ye > abs(0.5)):

            # chose camera to show it in video
            inserted_image, position, cam_label = (imgs_dict["LL"], 'ul', "LL") if (
                self.subs_actuatorreference.steering > 0) else (imgs_dict["RR"], 'ur', "RR")
            
            # Print camera label in subwindow
            render_text(image=inserted_image, msg="{}".format(cam_label), left=10, top=75, 
                color=(0, 255, 255), text_height=0, font_scale=3.0, font_thickness=2)

            # Insert rear camera in pilots image
            insert_image(
                original_image=img_src, inserted_image=inserted_image, 
                new_width=self.back_win_size[0], new_height=self.back_win_size[1], 
                position=position)

        # ---------------------------------------------------------------------          
        # GAMEOVER SCREEN - GAMEOVER SCREEN - GAMEOVER SCREEN - GAMEOVER SCREEN
        if self._GUI_GAME_OVER_SCREEN and (-25 < pan < 25):
            timer_on = self.component_game_over_screen.countdown_timer
            if (abs(roll) > np.pi/4.) or (abs(pitch) > np.pi/4.):
                self.component_game_over_screen.countdown_timer = 2.0
            if self.component_game_over_screen.countdown_timer:
                self.component_game_over_screen.countdown_timer = -1.0
                self.component_game_over_screen.draw(img_src=img_src)
                return img_src

        # ---------------------------------------------------------------------
        # STOP SCREEN - STOP SCREEN - STOP SCREEN - STOP SCREEN - STOP SCREEN -
        if self._GUI_STOP_SCREEN and self.subs_pwm_out.door_open:
           self.component_stop_screen.draw(img_src=img_src)
           return img_src

        # ---------------------------------------------------------------------
        # COMPASS - COMPASS - COMPASS - COMPASS - COMPASS - COMPASS - COMPASS -
        if self._GUI_COMPASS and (abs(pan) == 100 or pan == 0):
            theta = self.subs_odometry.theta if pan == 0 else (
                self.subs_odometry.theta - np.deg2rad(90.)) if (
                    pan > 0) else self.subs_odometry.theta + np.deg2rad(90.)
            theta = theta if not self.subs_rear_camera.switch_camera else theta + np.deg2rad(180.)
            self.component_compass.draw(img_src=img_src, 
                yaw_angle=theta, target_angle=None, target_distance=0.0)
        
        # ---------------------------------------------------------------------
        # DATA CAPTURE - DATA CAPTURE - DATA CAPTURE - DATA CAPTURE - DATA CAPT
        if self.subs_data_capture_status.recording and self._GUI_DATA_CAPTURE:
            print_text_list(
                img=img_src, 
                tex_list=[
                    "recording...", 
                    "IMG:{}".format(self.subs_data_capture_status.number_images),
                    "USB:{}".format(self.subs_data_capture_status.space_left)], 
                color=(0, 0, 255), 
                orig=(img_src.shape[1] - 180, 25))

        # ---------------------------------------------------------------------
        # LOGS TEXT SCREEN - LOGS TEXT SCREEN - LOGS TEXT SCREEN - LOGS TEXT SC
        if self.subs_visual_debugger.msg:
            show_msg(image=img_src, msg=self.subs_visual_debugger.msg, 
                msg_type=self.subs_visual_debugger.type)

        # ---------------------------------------------------------------------
        # WARNING LABELS - WARNING LABELS - WARNING LABELS - WARNING LABELS - W

        # Battery limit to show warning message
        if self._GUI_BATTERY_ICON and self.subs_battery.voltage < 11.25 : 
            self.draw_warning_label(img_src=img_src, 
                component="battery_icon", position="ir")

        # Object detection warning labels
        if len(self.subs_skynet.object_detection) and (-25 < pan < 25):
            if "LL" in self.subs_skynet.object_detection:
                component = predictions_warning_labels(
                    self.subs_skynet.object_detection["LL"])
                position = "cl" if not self.subs_rear_camera.switch_camera else "cr"
                if component is not None: # Left warning label object
                    self.draw_warning_label(img_src=img_src, 
                        component=component, position=position)
            if "RR" in self.subs_skynet.object_detection:
                component = predictions_warning_labels(self.subs_skynet.object_detection["RR"])
                position = "cr" if not self.subs_rear_camera.switch_camera else "cl"
                if component is not None: # Right warning label object
                    self.draw_warning_label(img_src=img_src, 
                        component=component, position=position)

        # ---------------------------------------------------------------------
        # COITIONAL FOR CENTER CAMERA: Components only will be drawn when only
        # the central camera is complete in the supervisors console image
        if pan != 0:
            return img_src

        # ---------------------------------------------------------------------  
        # FOLLOW OR POSITION MODE COMPONENTS - FOLLOW OR POSITION MODE COMPONEN
        if ((self.subs_bot.mode == "follow" 
            or self.subs_bot.mode == "positionControl"
            or LOCAL_LAUNCH)
            and not self.subs_rear_camera.switch_camera
            and pan==0):
            
            if self.mono_params.calibration["C"] is not None:

                # -------------------------------------------------------------
                # ROBOT PROJECTION - STEERING ANGLE - ROBOT PROJECTION - STEERI
                if self._GUI_PROJECTION:
                    if self.subs_actuatorcontrol.throttle >= 0.:
                        self.component_bot_projection.draw(
                            img_src=img_src,
                            curvature= self.subs_actuatorreference.steering, 
                            distord_lines=True)

                # -------------------------------------------------------------
                # WAYPOINT - CLICKABLE AREA - WAYPOINT - CLICKABLE AREA - WAYPO 
                self.component_waypoint_area.draw(img_src=img_src)

                # WAYPOINT CLICK - WAYPOINT CLICK - WAYPOINT CLICK - WAYPOINT C
                x_coords, y_coords = world_to_projections(
                    x_list=self.subs_navigation.ye, # -i for i in navigation.ye 
                    y_list=self.subs_navigation.xe, # y is positive to the left
                    ppy=self.mono_params.calibration["C"]["ppmy"],
                    ppx=self.mono_params.calibration["C"]["ppmx"],
                    bot_cord_proj=self.mono_params.calibration["C"]["bot_view_cord"])
                
                # Check if the current coordinates are inside the polygon area
                # or clickable area for a valid waypoint
                if len(x_coords) or len(y_coords):
                    ValidPoint = cv2.pointPolygonTest(
                        contour=self.mono_params.calibration["C"]["waypoint_area"], 
                        pt=(int(x_coords[-1]*self.VIDEO_WIDTH), 
                            int(y_coords[-1]*self.VIDEO_HEIGHT)), 
                        measureDist=True) 
    
                    if ValidPoint < 0.: # If valid waypoint coordinate
                        img_src= draw_pad_centers(
                            image=img_src,
                            x_coords=x_coords, y_coords=y_coords, 
                            M_inv=self.mono_params.calibration["C"]["M_inv"],
                            mtx=self.mono_params.mtx, dist=self.mono_params.dist)

                # -------------------------------------------------------------
                # FOLLOW MODE REFERENCES - FOLLOW MODE REFERENCES - FOLLOW MODE
                img_src = draw_chess_board(
                    image=img_src,
                    x_coords=self.subs_followpad.x_coords,
                    y_coords=self.subs_followpad.y_coords, 
                    M=self.mono_params.calibration["C"]["M"],
                    mtx=self.mono_params.mtx, 
                    dist=self.mono_params.dist)

        # ---------------------------------------------------------------------
        # NO CALIBRATION SCREEN - NO CALIBRATION SCREEN - NO CALIBRATION SCREEN
        if self._GUI_NO_CALIBRATION_SCREEN:
            if self.component_no_calibration_screen._mono_params.calibration["C"] is None:
                self.component_no_calibration_screen.draw(img_src=img_src)
                return img_src
            else:
                # -------------------------------------------------------------
                # CALIBRATION OUT OF DATE - CALIBRATION OUT OF DATE - CALIBRATI
                cal_state = self.component_no_calibration_screen._mono_params.calibration["C"]["out_date"]["state"]
                cal_days = self.component_no_calibration_screen._mono_params.calibration["C"]["out_date"]["days"]
                if cal_state and not self.subs_visual_debugger.msg:
                    show_msg(img_src, 
                        "La calibracion sera eliminada en {} dias".format(
                        self._VISION_CALIBRATION_DAYS_REMOVE - cal_days), 
                        None, left=10, top=300, color=(0, 0, 255))

        # ---------------------------------------------------------------------
        # OBJECT DETECTION - OBJECT DETECTION - OBJECT DETECTION - OBJECT DETEC
        if (self._GUI_OBJECT_DETECTION 
            and len(self.subs_skynet.object_detection) 
            and not self.subs_rear_camera.switch_camera): 
            if "C" in self.subs_skynet.object_detection:
                visualize_boxes_on_image(
                    img=img_src, 
                    predictions=self.subs_skynet.object_detection["C"]["predictions"], 
                    use_normalized_coordinates=False)

        # ---------------------------------------------------------------------
        return img_src

    def draw_warning_label(self, img_src, component, position="cc"):
        """ Draw robots warning labels in input image 'img_src'
        Args:
            img_src: `cv2.math` input image to draw component
            component: `string` component variable name 
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

        # Check and valid conditions
        if not self._GUI_WARNING_LABELS: 
            return img_src
        elif not position in ["sl","sc","sr","cl","cc",
            "cr","il","ic","ir"]: 
            return img_src
        
        # Select component to draw
        if component == "battery_icon":
            self.warning_labels.draw(img_src=img_src, 
                component=self.warning_labels.battery_icon, 
                position=position)
        elif component == "warning_object_icon":
            self.warning_labels.draw(img_src=img_src, 
                component=self.warning_labels.warning_icon, 
                position=position)
        elif component == "critical_object_icon":
            self.warning_labels.draw(img_src=img_src, 
                component=self.warning_labels.critical_icon, 
                position=position)
        elif component == "information_icon":
            self.warning_labels.draw(img_src=img_src, 
                component=self.warning_labels.information_icon, 
                position=position)

class gui_warning_labels():

    def __init__(self):

        self.battery_icon = gui_image_overlayed(
            img_path=os.path.join(figures_path, "sing_battery.png"),
            sc_fc=1., transparency=1.0)
        self.warning_icon = gui_image_overlayed(
            img_path=os.path.join(figures_path, "sing_warning.png"),
            sc_fc=1., transparency=1.0)
        self.critical_icon = gui_image_overlayed(
            img_path=os.path.join(figures_path, "sing_critical.png"),
            sc_fc=1., transparency=1.0)
        self.information_icon = gui_image_overlayed(
            img_path=os.path.join(figures_path, "sing_information.png"),
            sc_fc=1., transparency=1.0)
        
    def draw(self, img_src, component, position):
        """ Draw class component
        Args:
            img_src: `cv2.math` input image to draw component
            component: `gui_image_overlayed` component to draw
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
            img_src: `cv2.math` input image with component drawn
        """
        return component.draw(img_src, position)

class gui_component_base(object):
    
    def __init__(self):
        """ Initialize class components
        Args:
        Returns:
        """
        
        self._transparency = 0.0 # transparency value to overlay component

        self._x_pos = 0 # [pix] X coordinate for component
        self._y_pos = 0 # [pix] Y coordinate for component

        self._timer_on = False # [boolean] Enable/Disable Timer
        self._timer_start = 0. # [seg] Timer start time
        self._timer_out = -1.0 # [seg] Time limit for time counter. -1 turn off
        self._timer_elapse = 0.0 # [timestamp] timers elapse time

        self._sc_fc = 1.0 # Scaling factor

        self._cam_label = "C"
        self._mono_params = MonoParams(
            cameras=[self._cam_label], quite=True)

    def mono(self, key):
        if self._mono_params.calibration[self._cam_label] is not None:
            if key in self._mono_params.calibration[self._cam_label]:
                return self._mono_params.calibration[self._cam_label][key]
        return None

    @property
    def enable(self):
        return self._enable
    @enable.setter
    def enable(self, value):
        self._enable = value

    @property
    def transparency(self):
        return self._transparency
    @transparency.setter
    def transparency(self, value):
        if value < 0.:
            self._transparency = 0.
            print("[WARN]: The value of the transparency must be greater"
            "than zero. Value was setted as zero")
        elif value > 1.:
            self._transparency = 1.
            print("[WARN]: The value of the transparency must be lower than"
             "one. Value was setted as one")
        else:
            self._transparency = value

    @property
    def sc_fc(self):
        return self._sc_fc
    @sc_fc.setter
    def sc_fc(self, value):
        if value<0.:
            self._sc_fc = 0.
            print("[WARN]: The scaling factor value must be greater than zero." 
                "Value was setted as zero")
        else:
            self._sc_fc = value

    @property
    def timer_out(self):
        return self._timer_out
    @timer_out.setter
    def timer_out(self, value):
        self._timer_out = value

    @property
    def timer_elapse(self):
        return self._timer_elapse
    @timer_elapse.setter
    def timer_elapse(self, value):
        self._timer_elapse = value

    @property
    def timer_start(self):
        return self._timer_start
    @timer_start.setter
    def timer_start(self, value):
        self._timer_start = value

    @property
    def x_pos(self):
        return self._x_pos
    @x_pos.setter
    def x_pos(self, value):
        self._x_pos = int(value)

    @property
    def y_pos(self):
        return self._y_pos
    @y_pos.setter
    def y_pos(self, value):
        self._y_pos = int(value)

    @property
    def countdown_timer(self):
        return self._timer_on
    @countdown_timer.setter
    def countdown_timer(self, value):
        if value>0.:
            self._timer_on = True
            self.timer_start = time.time()
            self.timer_out = value
            self.timer_elapse = 0.0
        else:
            self.timer_elapse = time.time() - self.timer_start
            if self.timer_elapse >= self.timer_out:
                self._timer_on = False
                self.timer_start = 0
                self.timer_out = -1.0
                self.timer_elapse = 0

class gui_image_overlayed(gui_component_base):

    def __init__(self, img_path, transparency=0.5, sc_fc=0.5, 
        bgr_bleeding=[1, 1, 1], x=0, y=0, flip=None): 
        """ Initialize class components
        Args:
            img_path: `string` image path to overlay
            transparency: `float` transparency of image
            sc_fc: `float` image sacling factor
            bgr_bleeding: `list` with rgb bleeding values
        Returns:
        """

        # Initialize inherited components
        super(gui_image_overlayed, self).__init__() 

        self.transparency = transparency # Transparency of image
        self.bgr_bleeding = bgr_bleeding # Red bleeding procentage
        self.sc_fc = sc_fc  # image scaling factor
        self._overlayed_mask = None # Read and resize game over mask
        
        # If file exits then load image icon
        if os.path.isfile(img_path):
            self._overlayed_mask = cv2.imread(
                filename=img_path, flags=cv2.IMREAD_UNCHANGED)
            self._overlayed_mask = cv2.resize(src=self._overlayed_mask, 
                dsize=(int(self.x_pos + self._overlayed_mask.shape[1]*self.sc_fc), 
                       int(self.y_pos + self._overlayed_mask.shape[0]*self.sc_fc)))
            if flip is not None:
                self._overlayed_mask = cv2.flip(
                    src=self._overlayed_mask, flipCode=flip) 
        else:
            print("[ERROR]: No file to load image at {} ".format(img_path))

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

        # Apply bleeding to all image channels assuming an RGB image
        if self.bgr_bleeding[0] != 1:
            img_src[:,:,0] = img_src[:,:,0]*self.bgr_bleeding[0]
        if self.bgr_bleeding[1] != 1:
            img_src[:,:,1] = img_src[:,:,1]*self.bgr_bleeding[1]
        if self.bgr_bleeding[2] != 1:
            img_src[:,:,2] = img_src[:,:,2]*self.bgr_bleeding[2]
        
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

        return img_src

class gui_compass(gui_component_base):
    
    def __init__(self):
        """ Initialize class components
        Args:
        Returns:
        """

        super(gui_compass, self).__init__() # Initialize inherited components

        self._FONT_SCALE = float(os.getenv("GUI_COMPASS_FONT_SCALE", 0.8))
        self._VIEW_RANGE = float(os.getenv("GUI_COMPASS_VIEW_RANGE", 120.)) 
        self._RESOLUTION = int(os.getenv("GUI_COMPASS_RESOLUTION", 5))   
        self._DRAW_NUMBERS = int(os.getenv("GUI_COMPASS_NUMBERS", 1))  
        self._DRAW_GUIDES = int(os.getenv("GUI_COMPASS_GUIDES", 1))  
        self._IMG_SIZE = (
            int(os.getenv("VIDEO_WIDTH", 640)),
            int(os.getenv("VIDEO_HEIGHT", 360)))

        self._COORDS_DICT = {
            0:"N", 45:"NE", 90:"E", 135:"SE", 
            180:"S", 225:"SW", 270:"W", 315:"NW"}
        self._COORDS_LABELS = [self._COORDS_DICT[idx] if idx in 
            self._COORDS_DICT.keys() else str(idx) 
            for idx in range(0, 360, self._RESOLUTION)]
        self._COORDS_VALUES = [idx for idx in range(0, 360, self._RESOLUTION)]
        self._step_pix = int(self._RESOLUTION*self._IMG_SIZE[0]/self._VIEW_RANGE)

        # Define triangle for straight forward mark
        self._ctx=int(self._IMG_SIZE[0]*0.5); 
        self._cty=int(self._IMG_SIZE[1]*0.08); 
        self._my = 0
        self._straight_mark = np.array([(self._ctx - 10, self._my), 
            (self._ctx + 10, self._my), (self._ctx, self._my + 20)])

        target_icon_path = os.path.join(os.path.join(
            figures_path, "donut_icon.png"))
        self.target_icon=cv2.resize(cv2.imread(
            target_icon_path, cv2.IMREAD_UNCHANGED), (30, 30))

        # Filter parameters
        self._yaw_angle_list=[]
        self._list_length=10
        self._yaw_angle=0.0
        self._filter_a = 0.6

        self.test_angle = 0.0

    def draw(self, img_src, yaw_angle=0., target_angle=None, 
        target_distance=None):
        """ Draw compass component in input image 'img_src'
        Args:
            img_src: `cv2.math` input image to draw compass
            yaw_angle: `float` [deg] robots yaw angle 
            target_angle: `float` [deg] robots target relative angle
            target_distance: `float` [m] robots target relative distance
        Returns:
            img_src: `cv2.math` input image with compass drawn
        """

        if self._DRAW_GUIDES: # Draw rectangle as compass background
            cv2.rectangle(img=img_src, pt1=(0, 0), pt2=(self._IMG_SIZE[0], 40), 
                color=(100, 100, 100), thickness=-1)

        # Draw straight forward mark
        cv2.drawContours(image=img_src, contours=[self._straight_mark], 
            contourIdx=-1, color=(0,0,255), thickness=-1)

        # Get first angle coordinate to left
        self.yaw_angle = yaw_angle
        x_orig_value = self.yaw_angle - self._VIEW_RANGE*0.5
        x_orig_value = x_orig_value if x_orig_value > 0 else x_orig_value + 360.
        x_first_coord = min(self._COORDS_VALUES, key=lambda x:abs(x - x_orig_value))
        x_first_coord = x_first_coord if x_first_coord >= x_orig_value else x_first_coord + 5
        x_first_coord = x_first_coord if x_first_coord < 360 else 0

        # Get first pixel to draw first angle coordinate
        org_x = int((x_first_coord - x_orig_value)*self._step_pix/self._RESOLUTION)
    
        # Draw first centesimal lines
        if self._DRAW_GUIDES:
            for i in range(1, 5):
                idx_2 = org_x - int(i*self._step_pix/self._RESOLUTION)
                cv2.line(img=img_src, pt1=(idx_2, self._cty + 10), 
                    pt2=(idx_2, self._cty + 15), color=(0, 0, 0), thickness=6)
                cv2.line(img=img_src, pt1=(idx_2, self._cty + 10), 
                    pt2=(idx_2, self._cty + 15), color=(255, 255, 255), thickness=2)

        idx = self._COORDS_VALUES.index(x_first_coord)
        for x_pix in range(org_x, self._IMG_SIZE[0], self._step_pix):
            angle = self._COORDS_LABELS[idx] # get angle text
            
            # re-calculate drawing variables
            color = (0, 255, 255) if angle in self._COORDS_DICT.values() else (255, 255, 255)
            fontScale = self._FONT_SCALE if angle in self._COORDS_DICT.values() else self._FONT_SCALE*0.8
            
            if angle in self._COORDS_DICT.values() or self._DRAW_NUMBERS:
                try:
                    if int(angle) > 70 :
                        fontScale = fontScale*0.8
                except:
                    pass

                if self._DRAW_GUIDES:
                    # Draw decimal lines
                    cv2.line(img=img_src, pt1=(x_pix, self._cty + 5), 
                        pt2=(x_pix, self._cty + 15), color=(0, 0, 0), 
                        thickness=6)
                    cv2.line(img=img_src, pt1=(x_pix, self._cty + 5), 
                        pt2=(x_pix, self._cty + 15), color=(255, 255, 255), 
                        thickness=2)
                    
                    # Draw centesimal lines
                    sm_step_pix = int(np.ceil(self._step_pix/self._RESOLUTION))
                    for idx_2 in range(x_pix + sm_step_pix, x_pix + self._step_pix, sm_step_pix):
                        cv2.line(img=img_src, pt1=(idx_2, self._cty + 10), 
                            pt2=(idx_2, self._cty + 15), color=(0, 0, 0), 
                            thickness=6)
                        cv2.line(img=img_src, pt1=(idx_2, self._cty + 10), 
                            pt2=(idx_2, self._cty + 15), color=(255, 255, 255), 
                            thickness=2)

                # re-calculate drawing variables
                mv_fc = 0.6*(fontScale/self._FONT_SCALE)
                if len(angle) == 2: mv_fc = 1.1
                elif len(angle) == 3: mv_fc = 1.8
                org = (int(x_pix - 11*mv_fc), self._cty)

                # Draw angle text at position
                cv2.putText(img=img_src, text="{}".format(angle), org = org, 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=fontScale, 
                    color=(0, 0, 0), thickness = 4, lineType = cv2.LINE_AA)
                cv2.putText(img=img_src, text="{}".format(angle), org = org, 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=fontScale, 
                    color=color, thickness = 1, lineType = cv2.LINE_AA)

            # Increment index
            idx = idx + 1 if idx < len(self._COORDS_LABELS)-1 else 0

        # --------------------------------------------------------------------- 
        # Draw target icon
        if target_angle is not None:

            # Calculate distance from center angle to target angle for the 
            # left and right
            down_limit = self.yaw_angle - self._VIEW_RANGE*0.5
            Lr = abs(target_angle - self.yaw_angle)
            Ll = 360. - Lr

            y_idx = 5
            x_idx = int(img_src.shape[1]*0.5 + ((target_angle - self.yaw_angle
                )*self._step_pix/self._RESOLUTION))
            x_idx = int(x_idx-self.target_icon.shape[1]*0.5)# Center image in target_angle
            x_idx = x_idx if x_idx > 0 and Lr < Ll else 0 

            if Lr > Ll:
                if target_angle>down_limit + 360:
                    x_idx = int((target_angle-down_limit - 360)*self._step_pix/self._RESOLUTION 
                        - self.target_icon.shape[1]*0.5)
                    x_idx = x_idx if x_idx > 0 else 0  
                        
            x_idx = x_idx if x_idx < img_src.shape[1] - self.target_icon.shape[1] \
                else img_src.shape[1]-self.target_icon.shape[1]

            if target_distance is not None:
                cv2.putText(img=img_src, text="{}[m]".format(round(target_distance, 2)), 
                    org = (x_idx - 10, y_idx + 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale=0.5, color=(0, 0, 0), thickness=4, lineType=cv2.LINE_AA)
                cv2.putText(img=img_src, text="{}[m]".format(round(target_distance,2)), 
                    org = (x_idx - 10, y_idx + 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale=0.5, color=(255, 255, 0), thickness=1, lineType=cv2.LINE_AA)

            img_src = overlay_image(l_img=img_src, s_img=self.target_icon,
                pos=(x_idx, y_idx), transparency=1.0)

        return img_src

    @property
    def filter_a(self):
        return self._filter_a
    @filter_a.setter
    def filter_a(self, value):
        if value<0.:
            self._filter_a = 0
        elif value>1.0:
            self._filter_a = 1.0
        else:
            self._filter_a = value

    @property
    def yaw_angle(self):
        return self._yaw_angle
    @yaw_angle.setter
    def yaw_angle(self, value):

        # self.test_angle += 0.03
        # if self.test_angle < math.pi*2.:
        #     self.test_angle += math.pi*2.
        # if self.test_angle > math.pi*2.:
        #     self.test_angle -= math.pi*2.
        # value = self.test_angle
        value -= math.pi/2.
        if value <= math.pi/2. and self._yaw_angle > 270:
            for idx, val in enumerate(self._yaw_angle_list):
                if val >= 200:
                    self._yaw_angle_list[idx] -= 360
            self._yaw_angle -= 360 

        elif value >= math.pi*3/2. and self._yaw_angle > 0 and self._yaw_angle < 90:
            for idx, val in enumerate(self._yaw_angle_list):
                if val >= 0:
                    self._yaw_angle_list[idx] += 360
            self._yaw_angle += 360 

        value = int(np.rad2deg(value))
        if len(self._yaw_angle_list):
            y_t  = value
            y_tt = self._yaw_angle
            y_t = self.filter_a*y_t + (1. - self.filter_a)*y_tt 
            self._yaw_angle_list.append(int(y_t))

            if len(self._yaw_angle_list) > self._list_length: 
                self._yaw_angle_list.pop(0)
        else:
            self._yaw_angle_list.append(value)

        self._yaw_angle = np.mean(self._yaw_angle_list)

class gui_body_projection(gui_component_base):
    
    def __init__(self):
        """ Initialize class components
        Args:
        Returns:
        """

        super(gui_body_projection, self).__init__() # Initialize inherited components

        self._CURV_OFFSET = float(os.getenv("GUI_PROJECTION_BOT_CURV_OFFSET", 0.003)) # `float` Curvature offset for right line
        self._BOT_MARGI = float(os.getenv("GUI_PROJECTION_BOT_MARGI", 0.08))        # `float` [m] margin to add to robots projection
        self._BOT_WIDTH = float(os.getenv("GUI_PROJECTION_BOT_WIDTH", 0.43))        # `float` [m] robot's width to draw its projection 
        self._HORZL_LEN = int(os.getenv("GUI_PROJECTION_BOT_HORZL_LEN", 10))        # `int` horizontal lines length 
        self._VER_THICK = int(os.getenv("GUI_PROJECTION_BOT_VER_THICK", 2))         # `int` vertical lines thickness
        self._HOZ_THICK = int(os.getenv("GUI_PROJECTION_BOT_HOZ_THICK", 2))         # `int` horizontal lines thickness
        self._SHOW_PROJ = int(os.getenv("GUI_PROJECTION_SHOW_LOCAL", 0))         # `int` horizontal lines thickness
        self._COLORS = [(0, 0, 255), (32, 165, 218), (0, 255, 255), (0, 255, 0)]    # [R, G, B]
        self._DISTAN_M = [1.0, 1.5, 3.0, 5.0] # [m]

        self.local_proj = False # For local launch and to show image projection
        self._M = self.mono("M")
        self._INVM = None
        self._ppmx = 1.0
        self._ppmy = 1.0
        self._dist = None
        self._mtx = None
        self._view_cord = None
        self._UNWARPED_SIZE = None
        self._cnt_line = []
        self._proj_m = []
        self._dead_view_pix = 0
        self.y_limit = 0

        self._waypoint_cnt = None

        # Curve coefficients for left and right lines
        self.AR = 0.0
        self.BR = 0.0
        self.CR = 0.0
        self.AL = 0.0
        self.BL = 0.0
        self.CL = 0.0
        
        if self._M is not None:
            self.update_params()

    def update_params(self):
        
        self._UNWARPED_SIZE = self.mono("unwarped_size")
        self._INVM = self.mono("M_inv")
        self._M = self.mono("M")
        self._ppmx = self.mono("ppmx")
        self._ppmy = self.mono("ppmy")
        self._dist = self._mono_params.dist
        self._mtx = self._mono_params.mtx
        self._view_cord = self.mono("bot_view_cord")
        self._dead_view_pix = self.mono("dead_view_pix")  
        self.y_limit = int(self._view_cord[1] - self._DISTAN_M[-1]*self._ppmy)
        self._proj_m = []
        self._waypoint_cnt = self.mono("waypoint_area")

        sp = get_projection_point_src(
            coords_dst=(self._view_cord[0], self.y_limit, 1), INVM=self._INVM) 
        sp = get_distor_point(pt=sp, mtx=self._mtx, dist=self._dist)
        ip = get_projection_point_src(coords_dst=(self._view_cord[0], 
            self.mono("unwarped_size")[1], 1), INVM=self._INVM)
        ip = get_distor_point(pt=ip, mtx=self._mtx, dist=self._dist)
        self._cnt_line = (sp, ip)
        
        # Lines Distance from center or bot's view coordinate
        ct_dist = int(np.ceil((self._BOT_WIDTH*0.5 + self._BOT_MARGI)*self._ppmx)) 
        curvature = 0
        # Variables for polynomial
        self.AR = (0.01 + self._CURV_OFFSET)*curvature if curvature > 0. else (
                   0.01 - self._CURV_OFFSET)*curvature
        self.BR = 0.0
        self.CR = self._view_cord[0] + ct_dist

        self.AL = 0.01*curvature
        self.BL = 0.0
        self.CL = self._view_cord[0] - ct_dist

        self._proj_m = []; increment = 1
        idx_y = - (self._view_cord[1] - self._UNWARPED_SIZE[1])
        while idx_y >= -self._view_cord[1] + self.y_limit:
            self._proj_m.append(round(abs(idx_y)/self._ppmy, 2))
            idx_y -= increment; increment += 1            

    def draw(self, img_src, curvature=0.0, distord_lines=True):
        """ Draw class components
        Args:
            img_src: `cv2.math` input image to draw components
            curvature: `float` robots curvature of steering value 
            distord_lines: `boolean` Enable/Disable convert robot projection 
                to distorsion image (slows down process)
        Returns:
            img_src: `cv2.math` input image with components drawn
        """
        
        # Check to update calibration
        if self._mono_params.update_components:
            self._mono_params.update_components = False
            self.update_params()
            
        if self._M is None:
            return img_src
        
        # ---------------------------------------------------------------------
        if LOCAL_LAUNCH and self._SHOW_PROJ: # Draw image projection
            win_name = "LOCAL_LAUNCH_IMG_PROJ"
            if not self.local_proj:
                cv2.imshow(win_name, img_src)
                cv2.createTrackbar('curvature', win_name, 100, 200, nothing)
                self.local_proj = True
            curvature = (100-cv2.getTrackbarPos('curvature', win_name))/100.
            self.draw_in_proj(img_src=img_src.copy(), curvature=curvature, 
                win_name=win_name)

        # ---------------------------------------------------------------------
        # Variables for polynomial
        curvature *= -1
        self.AR = (0.01 + self._CURV_OFFSET)*curvature if curvature > 0. else (
                   0.01 - self._CURV_OFFSET)*curvature
        self.AL = 0.01*curvature

        # ---------------------------------------------------------------------
        # Get Left and right line points
        right_proj = []; left_proj = []; increment = 1
        idx_y = - (self._view_cord[1] - self._UNWARPED_SIZE[1])
        lout = False; rout = False
        while idx_y > -self._view_cord[1] + self.y_limit:
            
            # Left projection line
            if not lout:
                lp = get_projection_point_src((
                    self.AL*(idx_y**2) + self.BL*idx_y + self.CL, 
                    self._view_cord[1] + idx_y, 1), self._INVM)
                if distord_lines:
                    lp = get_distor_point(lp, self._mtx, self._dist)
                    in_cnt = cv2.pointPolygonTest(contour=self.mono("waypoint_area"), 
                        pt=lp, measureDist=True) 
                else:
                    in_cnt = cv2.pointPolygonTest(contour=self.mono("undistord_cnt"), 
                        pt=lp, measureDist=True) 
                if in_cnt >= 0:
                    left_proj.append(lp)
                else:
                    lout = True
            
            # Right projection line
            if not rout:
                rp = get_projection_point_src((
                    self.AR*(idx_y**2) + self.BR*idx_y + self.CR, 
                    self._view_cord[1] + idx_y, 1), self._INVM)
                if distord_lines:
                    rp = get_distor_point(rp, self._mtx, self._dist)
                    in_cnt = cv2.pointPolygonTest(contour=self.mono("waypoint_area"), 
                        pt=rp, measureDist=True) 
                else:
                    in_cnt = cv2.pointPolygonTest(contour=self.mono("undistord_cnt"), 
                        pt=rp, measureDist=True) 
                if in_cnt >= 0:
                    right_proj.append(rp)
                else:
                    rout = True
            
            idx_y -= increment; increment += 1

        left_proj = np.array(left_proj)
        right_proj = np.array(right_proj)

        # ---------------------------------------------------------------------
        # Draw robots projection lines

        # Left side
        color_idx = 0
        thickness_idx = len(self._COLORS)
        for idx in range(len(left_proj) - 1):
            if self._proj_m[idx] > self._DISTAN_M[-1]:
                break
            # Draw horizontal lines in body projection
            while self._proj_m[idx] > self._DISTAN_M[color_idx]:
                color_idx += 1 # Change Color
                thickness_idx -= 1 if thickness_idx > 0 else thickness_idx
                cv2.line(img=img_src, 
                    pt1=(left_proj[idx][0], left_proj[idx][1]), 
                    pt2=(left_proj[idx][0] + self._HORZL_LEN, left_proj[idx][1]), 
                    color=self._COLORS[color_idx], 
                    thickness=self._HOZ_THICK + thickness_idx)
            # Draw vertical lines in body projection
            cv2.line(img=img_src, 
                pt1=tuple(left_proj[idx]), pt2=tuple(left_proj[idx + 1]), 
                color=self._COLORS[color_idx], 
                thickness=self._VER_THICK + thickness_idx)

        # right side
        color_idx = 0
        thickness_idx = len(self._COLORS)
        for idx in range(len(right_proj) - 1):
            if self._proj_m[idx] > self._DISTAN_M[-1]:
                break
            # Draw horizontal lines in body projection
            while self._proj_m[idx] > self._DISTAN_M[color_idx]:
                color_idx += 1 # Change Color
                thickness_idx -= 1 if thickness_idx > 0 else thickness_idx
                cv2.line(img=img_src, 
                    pt1=(right_proj[idx][0], right_proj[idx][1]), 
                    pt2=(right_proj[idx][0] - self._HORZL_LEN, right_proj[idx][1]), 
                    color=self._COLORS[color_idx], 
                    thickness=self._HOZ_THICK + thickness_idx)  

            # Draw vertical lines in body projection
            cv2.line(img=img_src, 
                pt1=tuple(right_proj[idx]), pt2=tuple(right_proj[idx + 1]), 
                color=self._COLORS[color_idx], 
                thickness=self._VER_THICK + thickness_idx)

        # ---------------------------------------------------------------------
        # Print doted center line
        dotline(src=img_src, p1=self._cnt_line[0], p2=self._cnt_line[1],
            color=(255, 255, 255), thickness=self._VER_THICK, Dl=10)

        return img_src

    def draw_in_proj(self, img_src, curvature=0.0, 
        win_name="LOCAL_LAUNCH_IMG_PROJ"):

        # ---------------------------------------------------------------------
        # Get bird view image
        img_src = cv2.undistort(img_src, self._mtx, self._dist, None, 
            None) if img_src is not None else np.zeros((self.mono("image_size")[1], 
                self.mono("image_size")[0], 3), np.uint8)
        img_dst = cv2.warpPerspective(img_src, self._M, 
            (self._UNWARPED_SIZE[0], self._view_cord[1]))
        
        # ---------------------------------------------------------------------
        # Draw some geometries
        cv2.line(img_dst, (0, self._UNWARPED_SIZE[1]), 
            (self._UNWARPED_SIZE[0], self._UNWARPED_SIZE[1]), (0, 0, 255), 1)
        cv2.line(img_dst, (self._view_cord[0], 0), (self._view_cord[0], 
            self._view_cord[1]), (0, 255, 255), 1)
        cv2.circle(img=img_dst, center=self.mono("bot_view_cord"), radius=5, 
            color=(0, 0, 255), thickness=-1)

        # ---------------------------------------------------------------------
        # Get Left and right line points
        right_proj = []; left_proj = []; increment = 1
        idx_y = - (self._view_cord[1] - self._UNWARPED_SIZE[1])
        while idx_y >= -self._view_cord[1] + self.y_limit:
            left_proj.append((int(self.AL*(idx_y**2) + self.BL*idx_y + self.CL), 
                self._view_cord[1] + idx_y))
            right_proj.append((int(self.AR*(idx_y**2) + self.BR*idx_y + self.CR), 
                self._view_cord[1] + idx_y))
            idx_y -= increment; increment += 1            
        left_proj = np.array(left_proj)
        right_proj = np.array(right_proj)

        color_idx = 0
        thickness_idx = len(self._COLORS)
        for idx in range(len(left_proj) - 1):
            if self._proj_m[idx] > self._DISTAN_M[-1]:
                break
            # Draw horizontal lines in body projection
            while self._proj_m[idx] > self._DISTAN_M[color_idx]:
                color_idx += 1 # Change Color
                thickness_idx -= 1 if thickness_idx > 0 else thickness_idx
                cv2.line(img=img_dst, 
                    pt1=(left_proj[idx][0], left_proj[idx][1]), 
                    pt2=(left_proj[idx][0] + self._HORZL_LEN, left_proj[idx][1]), 
                    color=self._COLORS[color_idx], 
                    thickness=self._HOZ_THICK + thickness_idx)
                cv2.line(img=img_dst, 
                    pt1=(right_proj[idx][0], right_proj[idx][1]), 
                    pt2=(right_proj[idx][0] - self._HORZL_LEN, right_proj[idx][1]), 
                    color=self._COLORS[color_idx], 
                    thickness=self._HOZ_THICK + thickness_idx)

            # Draw vertical lines in body projection
            cv2.line(img=img_dst, 
                pt1=tuple(left_proj[idx]), pt2=tuple(left_proj[idx + 1]), 
                color=self._COLORS[color_idx], 
                thickness=self._VER_THICK + thickness_idx)
            cv2.line(img=img_dst, 
                pt1=tuple(right_proj[idx]), pt2=tuple(right_proj[idx + 1]), 
                color=self._COLORS[color_idx], 
                thickness=self._VER_THICK + thickness_idx)

        # ---------------------------------------------------------------------
        cv2.imshow(win_name, img_dst); cv2.waitKey(1)

class gui_waypoint_area(gui_component_base):
    
    def __init__(self):
        """ Initialize class components
        Args:
        Returns:
        """

        super(gui_waypoint_area, self).__init__()

        self._waypoint_cnt = None
        self._undistord_cnt = None
        
        self._WAYPOINT_CLICKABLE_AREA = int(os.getenv("GUI_WAYPOINT_CLICKABLE_AREA", 1))
        self._UNDISTORD_PROJECTION = int(os.getenv("GUI_UNDISTORD_PROJECTION", 0))
        self._WAYPOINT_THICKNESS = int(os.getenv("GUI_WAYPOINT_THICKNESS", 2))

        self.update_params()

    def update_params(self):
        """ Update parameters when a calibration is performed
        Args:
        Returns:
        """

        self._waypoint_cnt = self.mono("waypoint_area")
        self._undistord_cnt = self.mono("undistord_cnt")

    def draw(self, img_src):
        """ Draw class components
        Args:
            img_src: `cv2.math` input image to draw components
        Returns:
            img_src: `cv2.math` input image with components drawn
        """
        
        if self._mono_params.update_components:
            self.update_params()
            self._mono_params.update_components = False

        # Draw surface perspective projection contour
        if self._UNDISTORD_PROJECTION and self._undistord_cnt is not None:
            cv2.drawContours(image=img_src, contours=[self._undistord_cnt], 
                contourIdx=0, color=(0, 0, 200), thickness=self._WAYPOINT_THICKNESS)

        # Draw waypoint area contour
        if self._WAYPOINT_CLICKABLE_AREA and self._waypoint_cnt is not None:
            cv2.drawContours(image=img_src, contours=[self._waypoint_cnt], 
                contourIdx=0, color=(0, 200, 0), thickness=self._WAYPOINT_THICKNESS)

        return img_src

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

class gui_wan_networks_status_report():

    def __init__(self):

        self.subs_wan_networks = WanNetworksSuscriber()

        # Text features variables
        self.full_description = int(os.getenv("PEPWAVE_SHOW_GUI_FULL", 0))
        self._colors = [
            (133, 169, 252),
            (93, 202, 133),
            (163, 191, 72),
            (191, 137, 117),
            (182, 140, 249)
        ]
        self._idx = 0
        self._font = cv2.FONT_HERSHEY_SIMPLEX
        self._thickness = 1
        self._scale = 0.5
        self._x_org = 10
        self._y_org = 20
        self._y_off_spped = 20 if not FR_AGENT else 60
        self._x_off_spped = 5

        self._t_tick = 0.
        self._t_limit = 15.

        # icons
        self._signal_icons = {
            0:cv2.imread(filename=os.path.join(figures_path, "signal_0.png"), flags=cv2.IMREAD_UNCHANGED),
            1:cv2.imread(filename=os.path.join(figures_path, "signal_1.png"), flags=cv2.IMREAD_UNCHANGED),
            2:cv2.imread(filename=os.path.join(figures_path, "signal_2.png"), flags=cv2.IMREAD_UNCHANGED),
            3:cv2.imread(filename=os.path.join(figures_path, "signal_3.png"), flags=cv2.IMREAD_UNCHANGED),
            4:cv2.imread(filename=os.path.join(figures_path, "signal_4.png"), flags=cv2.IMREAD_UNCHANGED),
            5:cv2.imread(filename=os.path.join(figures_path, "signal_5.png"), flags=cv2.IMREAD_UNCHANGED),
            }
        self._download_icon = cv2.imread(
            filename=os.path.join(figures_path, "download.png"), 
            flags=cv2.IMREAD_UNCHANGED)
        self._upload_icon = cv2.imread(
            filename=os.path.join(figures_path, "upload.png"), 
            flags=cv2.IMREAD_UNCHANGED)

    def draw(self, img_src):
        """ Draw wan networks status list and wan speeds
        Args:
        Returns:
        """

        wan_list_status=self.subs_wan_networks.wan_networks
        wan_speed=self.subs_wan_networks.speed

        # ---------------------------------------------------------------------
        y_idx = 22
        wan_info_str = []
        for idx, wan in enumerate(wan_list_status):

            color = (255, 255, 255) if not self.full_description else self._colors[idx]
            if wan.type == "wifi":
                self.draw_text(img=img_src, text="   {}".format(wan.carrier), 
                    org=(self._x_org, self._y_org + int(idx*y_idx)), color=color)
                if idx == self._idx:
                    # wan_info_str.append("id: {}".format(wan.id))
                    wan_info_str.append("WanName: {}".format(wan.WanName))
                    # wan_info_str.append("type: {}".format(wan.type))
                    # wan_info_str.append("carrier: {}".format(wan.carrier))
                    # wan_info_str.append("message: {}".format(wan.message))
                    wan_info_str.append("signalLevel: {}".format(wan.AllowanceUse))
                    wan_info_str.append("strength: {}".format(wan.AllowanceLimit))
                    # wan_info_str.append("AllowanceUse: {}".format(wan.AllowanceUse))
                    # wan_info_str.append("AllowanceLimit: {}".format(wan.AllowanceLimit))

            elif wan.type =="cellular":
                self.draw_text(img=img_src, text="   {}".format(wan.carrier), 
                    org=(self._x_org, self._y_org + int(idx*y_idx)), color=color)
                if idx == self._idx:
                    # wan_info_str.append("id: {}".format(wan.id))
                    wan_info_str.append("WanName: {}".format(wan.WanName))
                    # wan_info_str.append("type: {}".format(wan.type))
                    # wan_info_str.append("carrier: {}".format(wan.carrier))
                    # wan_info_str.append("message: {}".format(wan.message))
                    wan_info_str.append("rssi: {}".format(round(wan.rssi, 2)))
                    wan_info_str.append("rsrq: {}".format(round(wan.rsrq, 2)))
                    wan_info_str.append("rsrp: {}".format(round(wan.rsrp, 2)))
                    wan_info_str.append("sinr: {}".format(round(wan.sinr, 2)))
                    # wan_info_str.append("AllowanceUse: {}".format(wan.AllowanceUse))
                    # wan_info_str.append("AllowanceLimit: {}".format(wan.AllowanceLimit))
            else:
                wan.signalLevel = 0
                self.draw_text(img=img_src, text="   {}".format(wan.WanName), 
                    org=(self._x_org, self._y_org + int(idx*y_idx)), color=color)
                
            overlay_image(
                l_img=img_src, s_img=self._signal_icons[wan.signalLevel],
                pos=(self._x_org, 5 + int(idx*y_idx)), transparency=1.0)

        if self.full_description:
            y_offset = len(wan_list_status)*y_idx
            
            if time.time() - self._t_tick > self._t_limit:
                self._t_tick = time.time()
                self._idx = 0 if self._idx >= len(wan_list_status) - 1 else self._idx + 1

            for idx, srt_ in enumerate(wan_info_str):
                self.draw_text(img=img_src, text=srt_, 
                    org=(self._x_org, int(y_offset + (idx+1.3)*y_idx)), 
                    color=self._colors[self._idx])

        # ---------------------------------------------------------------------
        # Report newtwork speeds
        self.draw_text(img=img_src, text="   {} {}".format(
            wan_speed["download"], wan_speed["unit"]), 
            org=(self._x_off_spped, img_src.shape[0] - self._y_off_spped - 27), 
            color=(255, 255, 255))
        overlay_image(
            l_img=img_src, s_img=self._download_icon,
            pos=(self._x_off_spped, int(img_src.shape[0] - self._y_off_spped - 42)), 
            transparency=1.0)

        self.draw_text(img=img_src, text="   {} {}".format(
            wan_speed["upload"], wan_speed["unit"]), 
            org=(self._x_off_spped, int(img_src.shape[0] - self._y_off_spped)), 
            color=(255, 255, 255))
        overlay_image(
            l_img=img_src, s_img=self._upload_icon,
            pos=(self._x_off_spped, int(img_src.shape[0] - self._y_off_spped - 15)), 
            transparency=1.0)

        # ---------------------------------------------------------------------
        # Report GPS status
        gps_color = (0, 255, 0)  if self.subs_wan_networks.gps["status"] else (0, 0, 255)
        x_offset = 100 if not FR_AGENT else 160
        self.draw_text(
            img=img_src, 
            text="pepGPS", 
            org=(img_src.shape[1] - x_offset, img_src.shape[0] - 25), 
            color=gps_color, fontScale=0.2)

        return img_src

    def draw_text(self, img, text, org, color, fontScale=0):
        cv2.putText(img=img, text=text, org=org, fontFace=self._font, 
            fontScale=self._scale + fontScale, color=(0, 0, 0), thickness=self._thickness + 2)
        cv2.putText(img=img, text=text, org=org, fontFace=self._font, 
            fontScale=self._scale + fontScale, color=color, thickness=self._thickness) 

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

# =============================================================================