#!/usr/bin/env python3
# =============================================================================
import numpy as np
import cv2
import os

from usr_msgs.msg import VisualMessage

from vision.utils.vision_utils import insert_image
from vision.utils.vision_utils import printlog
from vision.utils.vision_utils import dotline

from python_utils.pysubscribers import VisualDebuggerSubscriber
from python_utils.pysubscribers import ExtrinsicSubscriber
from python_utils.pysubscribers import Robot

from vision.extrinsic.extrinsic_utils import get_projection_point_src
from vision.extrinsic.extrinsic_utils import get_distor_point

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
        # Class components
        self.comp_waypoint_proj = CompBodyProjection()

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
            self.draw_intrinsic(img=imgs_dict["P"], cam_label="C")
            self.comp_waypoint_proj.draw(
                extrinsic=self.sub_extrinsic.extrinsic,
                img_src=imgs_dict["P"], 
                curvature=0.0)

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

class CompBodyProjection():
    
    def __init__(self):
        """ Initialize class components
        Args:
        Returns:
        """

        self._CURV_OFFSET = float(os.getenv("GUI_PROJECTION_BOT_CURV_OFFSET", 0.003))
        self._BOT_MARGI = float(os.getenv("GUI_PROJECTION_BOT_MARGI", 0.08))
        self._BOT_WIDTH = float(os.getenv("GUI_PROJECTION_BOT_WIDTH", 0.5))
        self._HORZL_LEN = int(os.getenv("GUI_PROJECTION_BOT_HORZL_LEN", 10))
        self._VER_THICK = int(os.getenv("GUI_PROJECTION_BOT_VER_THICK", 2))
        self._HOZ_THICK = int(os.getenv("GUI_PROJECTION_BOT_HOZ_THICK", 2))
        self._SHOW_PROJ = int(os.getenv("GUI_PROJECTION_SHOW_LOCAL", 0))
        self._COLORS = [(0, 0, 255), (32, 165, 218), (0, 255, 255), (0, 255, 0)]
        self._DISTAN_M = [1.0, 1.5, 3.0, 5.0]

        # Curve coefficients for left and right lines
        self.y_limit = 0
        self.AR = 0.0
        self.BR = 0.0
        self.CR = 0.0
        self.AL = 0.0
        self.BL = 0.0
        self.CL = 0.0
        
        self._proj_m = None
        self._cnt_line = None
        self.y_limit = None
             
    def draw(self, img_src, extrinsic=None, cam_label= "C", curvature=0.0, 
        distord_lines=True):
        """ Draw class components
        Args:
            img_src: `cv2.math` input image to draw components
            curvature: `float` robots curvature of steering value
            extrinsic: 'Extrinsic class' extrinsic calibration 
            distord_lines: `boolean` Enable/Disable convert robot projection 
                to distorsion image (slows down process)
        Returns:
            img_src: `cv2.math` input image with components drawn
        """
        
        # ---------------------------------------------------------------------
        # If not extrinsic calibration then continue
        if extrinsic.cams[cam_label] is None:
            return
        elif extrinsic.mtx is None:
            return

        # Update params
        if self._proj_m is None:
            
            self.y_limit = int(
                extrinsic.cams[cam_label]["view_coord"][1] - \
                self._DISTAN_M[-1]*extrinsic.cams[cam_label]["ppmy"])

            sp = get_projection_point_src(
                coords_dst=(
                    extrinsic.cams[cam_label]["view_coord"][0], 
                    self.y_limit, 1), 
                INVM=extrinsic.cams[cam_label]["M_inv"]) 
            sp = get_distor_point(
                pt=sp, 
                mtx=extrinsic.mtx, 
                dist=extrinsic.dist)
            ip = get_projection_point_src(
                coords_dst=(
                    extrinsic.cams[cam_label]["view_coord"][0], 
                    extrinsic.cams[cam_label]["unwarped_size"][1], 1), 
                INVM=extrinsic.cams[cam_label]["M_inv"])
            ip = get_distor_point(
                pt=ip, 
                mtx=extrinsic.mtx, 
                dist=extrinsic.dist)
            self._cnt_line = (sp, ip)
            self._proj_m = []; increment = 1
            
            idx_y = - (extrinsic.cams[cam_label]["view_coord"][1] - \
                extrinsic.cams[cam_label]["unwarped_size"][1])
            
            while idx_y >= -extrinsic.cams[cam_label]["view_coord"][1] + self.y_limit:
                self._proj_m.append(round(abs(idx_y)/extrinsic.cams[cam_label]["ppmy"], 2))
                idx_y -= increment; increment += 1       

        # ---------------------------------------------------------------------
        # Variables for polynomial
        curvature *= -1
        # Lines Distance from center or bot's view coordinate
        ct_dist = int(np.ceil((self._BOT_WIDTH*0.5 + \
            self._BOT_MARGI)*extrinsic.cams[cam_label]["ppmy"])) 
        self.AR = (0.01 + self._CURV_OFFSET)*curvature if curvature > 0. else (
                   0.01 - self._CURV_OFFSET)*curvature
        self.AL = 0.01*curvature
        self.CR = extrinsic.cams[cam_label]["view_coord"][0] + ct_dist
        self.CL = extrinsic.cams[cam_label]["view_coord"][0] - ct_dist
        self.AL = 0.01*curvature
        self.BL = 0.0

        # ---------------------------------------------------------------------
        # Get Left and right line points
        right_proj = []; left_proj = []; increment = 1
        idx_y = - (
            extrinsic.cams[cam_label]["view_coord"][1] - \
            extrinsic.cams[cam_label]["unwarped_size"][1])
        lout = False; rout = False

        while idx_y > - extrinsic.cams[cam_label]["view_coord"][1] + self.y_limit:

            # Left projection line
            if not lout:
                lp = get_projection_point_src(
                    coords_dst=(self.AL*(idx_y**2) + self.BL*idx_y + self.CL, 
                    extrinsic.cams[cam_label]["view_coord"][1] + idx_y, 1), 
                    INVM=extrinsic.cams[cam_label]["M_inv"])
                if distord_lines:
                    lp = get_distor_point(
                        pt=lp, 
                        mtx=extrinsic.mtx, 
                        dist=extrinsic.dist)
                    in_cnt = cv2.pointPolygonTest(
                        contour=extrinsic.cams[cam_label]["waypoint_area"], 
                        pt=lp, measureDist=True) 
                else:
                    in_cnt = cv2.pointPolygonTest(
                        contour=extrinsic.cams[cam_label]["undistord_cnt"], 
                        pt=lp, measureDist=True) 
                if in_cnt >= 0:
                    left_proj.append(lp)
                else:
                    lout = True
        
            # Right projection line
            if not rout:
                rp = get_projection_point_src(
                    coords_dst=(self.AR*(idx_y**2) + self.BR*idx_y + self.CR, 
                    extrinsic.cams[cam_label]["view_coord"][1] + idx_y, 1), 
                    INVM=extrinsic.cams[cam_label]["M_inv"])
                if distord_lines:
                    rp = get_distor_point(
                        pt=rp, 
                        mtx=extrinsic.mtx, 
                        dist=extrinsic.dist)
                    in_cnt = cv2.pointPolygonTest(
                        contour=extrinsic.cams[cam_label]["waypoint_area"], 
                        pt=rp, measureDist=True) 
                else:
                    in_cnt = cv2.pointPolygonTest(
                        contour=extrinsic.cams[cam_label]["undistord_cnt"], 
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

# =============================================================================