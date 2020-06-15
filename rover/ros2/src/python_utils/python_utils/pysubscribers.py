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
import time
import cv2
import os

from usr_msgs.msg import Extrinsic as Extrinsic_msg
from usr_msgs.msg import Control as WebControl
from usr_msgs.msg import VisualMessage
from usr_msgs.msg import Waypoint
from usr_msgs.msg import PWMOut
from usr_msgs.msg import Motors
from usr_msgs.msg import ChassisState
from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
s
from vision.utils.vision_utils import print_text_list
from vision.utils.vision_utils import printlog
from vision.utils.vision_utils import dotline

from vision.intrinsic.intrinsic_utils import IntrinsicClass

from vision.extrinsic.extrinsic_utils import Extrinsic
from vision.extrinsic.extrinsic_utils import get_projection_point_src
from vision.extrinsic.extrinsic_utils import get_projection_point_dst
from vision.extrinsic.extrinsic_utils import get_undistor_point
from vision.extrinsic.extrinsic_utils import get_distor_point

# =============================================================================
class VisualDebuggerSubscriber:
    def __init__(self, parent_node):

        # Timer with time to show message
        self._VISUAL_DEBUGGER_TIME = int(
            os.getenv(key="VISUAL_DEBUGGER_TIME", default=10)
        )

        # Message to show in console
        self.msg = ""
        # Type of message "INFO, ERROR, WARN"
        self.type = "INFO"
        self.font_scale = 0.5

        # Subscribers
        self._sub_visual_debugger = parent_node.create_subscription(
            msg_type=VisualMessage,
            topic="video_streaming/visual_debugger",
            callback=self.cb_visual_debugger,
            qos_profile=5,
            callback_group=parent_node.callback_group,
        )

    def cb_visual_debugger(self, msg):
        """ Draws the visual debugger message
        Args:
            msg: `VisualMessage` message for visual debugger
                data: `string` message content
                type: `string` message type
        Returns:
        """

        self.msg = msg.data
        self.type = msg.type

        time.sleep(self._VISUAL_DEBUGGER_TIME)

        self.msg = ""
        self.type = "INFO"

    def draw(self, img):
        """ Draw the visual debugger message
        Args:
            img: `cv2.math` image to draw visual
                debugger message
        Returns:
        """

        if not self.msg:
            return

        color = (255, 255, 255)
        if self.type == "ERROR" or self.type == "ERR":
            color = (153, 153, 255)
        elif self.type == "WARNING" or self.type == "WARN":
            color = (0, 255, 255)
        elif self.type == "OKGREEN":
            color = (153, 255, 51)

        print_text_list(
            img=img,
            tex_list=[self.msg],
            color=color,
            orig=(10, int(img.shape[0] * 0.95)),
            fontScale=self.font_scale,
        )


class ExtrinsicSubscriber:
    def __init__(self, parent_node, cam_labels=["C"]):

        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))

        # Read intrinsic parameters from file
        self.intrinsic = IntrinsicClass()

        # extrinsic parameters dictionary
        self.extrinsic = Extrinsic(
            dist=self.intrinsic.distortion_coefficients,
            mtx=self.intrinsic.mtx,
            cam_labels=cam_labels,
        )

        # Subscribers
        self._sub_extrinsic_params = parent_node.create_subscription(
            msg_type=Extrinsic_msg,
            topic="video_calibrator/extrinsic_parameters",
            callback=self.cb_extrinsic_params,
            qos_profile=2,
            callback_group=parent_node.callback_group,
        )

    def cb_extrinsic_params(self, msg):
        """ Re-assing extrinsic calibration 
        Args:
            msg: `VisualMessage` message for visual debugger
                data: `string` message content
                type: `string` message type
        Returns:
        """

        try:
            if msg.cam_label in self.extrinsic.cams.keys():
                self.extrinsic.cams[msg.cam_label] = self.extrinsic.load(
                    mtx=self.intrinsic.mtx,
                    dist=self.intrinsic.distortion_coefficients,
                    FILE_NAME="Extrinsic_{}_{}_{}.yaml".format(
                        self._VIDEO_WIDTH, self._VIDEO_HEIGHT, msg.cam_label
                    ),
                )

                for key, _ in self.extrinsic.update.items():
                    self.extrinsic.update[key] = True

            printlog(
                msg="Extrinsic parameters for CAM{} updated".format(msg.cam_label),
                msg_type="INFO",
            )

        except Exception as e:
            printlog(
                msg="Error getting extrinsic calibration" "from topic, {}".format(e),
                msg_type="ERROR",
            )
            return False


class WebclientControl:
    def __init__(self, parent_node):

        self.pan = 0
        self.throttle = 0
        self.direction = 0.0

        self._sub_freedom_control = parent_node.create_subscription(
            topic="freedom_client/cmd_vel",
            msg_type=TwistStamped,
            callback=self.cb_sub_freedom_control,
            qos_profile=5,
            callback_group=parent_node.callback_group,
        )

        self._sub_web_client_control = parent_node.create_subscription(
            topic="web_client/control",
            msg_type=WebControl,
            callback=self.cb_sub_web_client_control,
            qos_profile=5,
            callback_group=parent_node.callback_group,
        )

    def cb_sub_freedom_control(self, data):
        try:
            self.throttle = int(data.twist.linear.x * (100.0 / 1.5))  # [m/s]
            self.direction = data.twist.angular.z * (np.pi / 1.5)  # [rad/s]

        except Exception as e:
            printlog(
                msg="Error getting msg for WebclientControl class, {}".format(e),
                msg_type="ERROR",
            )

    def cb_sub_web_client_control(self, data):
        try:
            self.pan = data.pan
            self.throttle = data.speed
            self.direction = -data.tilt / 100
        except Exception as e:
            printlog(
                msg="Error getting msg for WebclientControl class, {}".format(e),
                msg_type="ERROR",
            )


class WaypointSubscriber:
    def __init__(self, parent_node, extrinsic, intrinsic, webclient_control):

        # ---------------------------------------------------------------------
        # Environment variables
        self._GUI_WAYPOINT_AREA = int(os.getenv(key="GUI_WAYPOINT_AREA", default=1))
        self._LOCAL_LAUNCH = int(os.getenv(key="LOCAL_LAUNCH", default=1))
        self._GUI_WAYPOINT_DESCRIPTION = int(
            os.getenv(key="GUI_WAYPOINT_DESCRIPTION", default=1)
        )

        # ---------------------------------------------------------------------
        self.extrinsic = extrinsic
        self.extrinsic.update["WaypointSuscriber"] = True
        self.intrinsic = intrinsic
        self.webclient_control = webclient_control

        self.cam_label = "C"  # Camera label to get extrinsic
        self.x_norm = None  # Normalized X axis Waypoint coordinate
        self.y_norm = None  # Normalized Y axis Waypoint coordinate
        self.x_img = None  # X axis Waypoint coordinate
        self.y_img = None  # Y axis Waypoint coordinate
        self.x_warp = None  # X axis Waypoint coordinate in warped space
        self.y_warp = None  # Y axis Waypoint coordinate in warped space
        self.x_m = 0.0  # X axis Waypoint coordinate in warped space
        self.y_m = 0.0  # Y axis Waypoint coordinate in warped space
        self.abs_m = 0.0  # Absolute distance from origin to waypoint
        self.angle = 0.0  # Angle of coordinate
        self.incnt = False  # Current waypoint is inside contour

        # ---------------------------------------------------------------------
        # Boot projection
        self._CURV_OFFSET = float(os.getenv("GUI_PROJECTION_BOT_CURV_OFFSET", 0.003))
        self._BOT_MARGI = float(os.getenv("GUI_PROJECTION_BOT_MARGI", 0.3))
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

        self.left_proj = []
        self.right_proj = []

        self._proj_m = None
        self._cnt_line = None
        self.y_limit = None

        self.curvature = 1.0
        self.new_curvature = 0.0
        self.distord_lines = True  # Enable/Disable distord bot projection
        self.prev_webclient_control_direction = -10000

        # ---------------------------------------------------------------------
        # Publishers
        self._sub_screen_point = parent_node.create_subscription(
            msg_type=Waypoint,
            topic="video_streaming/waypoint_pt",
            callback=self.cb_screen_point,
            qos_profile=2,
            callback_group=parent_node.callback_group,
        )

        # ---------------------------------------------------------------------
        # Publishers
        self.WaypointZoom_msg = Waypoint()
        self.pub_waypoint_zoom = parent_node.create_publisher(
            Waypoint, "video_streaming/zoom", 1
        )

        self.bool_msg = Bool()
        self.pub_streaming_idle_restart = parent_node.create_publisher(
            Bool,
            "video_streaming/optimizer/idle_restart",
            1,
            callback_group=parent_node.callback_group,
        )

    def cb_screen_point(self, msg):

        self.pub_streaming_idle_restart.publish(self.bool_msg)

        try:
            self.x_norm = msg.x
            self.y_norm = msg.y
            # print(self.x_norm, self.y_norm, flush=True)

            # If there's no camera calibartion
            if self.extrinsic.cams[self.cam_label] is None:
                raise Exception(f"camera {self.cam_label} no calibrated")

            # print(self.extrinsic.cams[self.cam_label].keys())
            self.x_img = int(
                self.x_norm * self.extrinsic.cams[self.cam_label]["image_size"][0]
            )
            self.y_img = int(
                self.y_norm * self.extrinsic.cams[self.cam_label]["image_size"][1]
            )

            # Check if the current coordinates are inside the polygon area
            # or clickable area for a valid way-point
            ValidPoint = cv2.pointPolygonTest(
                contour=self.extrinsic.cams[self.cam_label]["waypoint_area"],
                pt=(self.x_img, self.y_img),
                measureDist=True,
            )

            if ValidPoint > 0:
                self.incnt = True
                x, y = get_undistor_point(
                    pt=(self.x_img, self.y_img),
                    mtx=self.extrinsic.mtx,
                    dist=self.extrinsic.dist,
                )
                warped_coord = get_projection_point_dst(
                    M=self.extrinsic.cams[self.cam_label]["M"], coords_src=(x, y, 1)
                )
                self.x_warp, self.y_warp = warped_coord[0], warped_coord[1]

                view_coord = self.extrinsic.cams[self.cam_label]["view_coord"]
                ppmx = self.extrinsic.cams[self.cam_label]["ppmx"]
                ppmy = self.extrinsic.cams[self.cam_label]["ppmy"]

                self.x_m = (self.x_warp - view_coord[0]) / ppmx
                self.y_m = (view_coord[1] - self.y_warp) / ppmy
                self.abs_m = np.sqrt(self.x_m ** 2 + self.y_m ** 2)
                self.angle = np.rad2deg(np.arctan2(self.x_m, self.y_m))

                self.new_curvature = (self.x_warp - view_coord[0]) / (
                    0.01 * (abs(view_coord[1] - self.y_warp) ** 2)
                )

            else:
                self.incnt = False
                self.x_warp = None
                self.y_warp = None
                self.x_m = 0.0
                self.y_m = 0.0
                self.abs_m = 0.0
                self.angle = 0.0

                # Publish for zoom
                self.WaypointZoom_msg.x = msg.x
                self.WaypointZoom_msg.y = msg.y
                self.pub_waypoint_zoom.publish(self.WaypointZoom_msg)

        except Exception as e:
            printlog(msg="error processing waypoint, {}".format(e), msg_type="ERROR")
            self.x_norm = None  # Normalized X axis Waypoint coordinate
            self.y_norm = None  # Normalized Y axis Waypoint coordinate
            self.x_img = None  # X axis Waypoint coordinate
            self.y_img = None  # Y axis Waypoint coordinate
            self.x_warp = None  # X axis Waypoint coordinate in warped space
            self.y_warp = None  # Y axis Waypoint coordinate in warped space
            self.x_m = 0.0  # X axis Waypoint coordinate in warped space
            self.y_m = 0.0  # Y axis Waypoint coordinate in warped space
            self.abs_m = 0.0
            self.angle = 0.0

    def update_params(self):

        # ---------------------------------------------------------------------
        if self.extrinsic.update["WaypointSuscriber"]:
            self.y_limit = int(
                self.extrinsic.cams[self.cam_label]["view_coord"][1]
                - self._DISTAN_M[-1] * self.extrinsic.cams[self.cam_label]["ppmy"]
            )

            sp = get_projection_point_src(
                coords_dst=(
                    self.extrinsic.cams[self.cam_label]["view_coord"][0],
                    self.y_limit,
                    1,
                ),
                INVM=self.extrinsic.cams[self.cam_label]["M_inv"],
            )
            sp = get_distor_point(
                pt=sp, mtx=self.extrinsic.mtx, dist=self.extrinsic.dist
            )
            ip = get_projection_point_src(
                coords_dst=(
                    self.extrinsic.cams[self.cam_label]["view_coord"][0],
                    self.extrinsic.cams[self.cam_label]["unwarped_size"][1],
                    1,
                ),
                INVM=self.extrinsic.cams[self.cam_label]["M_inv"],
            )

            ip = get_distor_point(
                pt=ip, mtx=self.extrinsic.mtx, dist=self.extrinsic.dist
            )

            self._cnt_line = (sp, ip)
            self._proj_m = []
            increment = 1

            idx_y = -(
                self.extrinsic.cams[self.cam_label]["view_coord"][1]
                - self.extrinsic.cams[self.cam_label]["unwarped_size"][1]
            )

            while (
                idx_y
                >= -self.extrinsic.cams[self.cam_label]["view_coord"][1] + self.y_limit
            ):
                self._proj_m.append(
                    round(abs(idx_y) / self.extrinsic.cams[self.cam_label]["ppmy"], 2)
                )
                idx_y -= increment
                increment += 1

            self.extrinsic.update["WaypointSuscriber"] = False

        # ---------------------------------------------------------------------
        if self.webclient_control.direction != self.prev_webclient_control_direction:
            self.prev_webclient_control_direction = self.webclient_control.direction
            self.new_curvature = self.webclient_control.direction * 3

        if self.new_curvature != self.curvature:
            self.curvature = self.new_curvature
        else:
            return

        # ---------------------------------------------------------------------
        # Variables for polynomial
        # Lines Distance from center or bot's view coordinate
        ct_dist = int(
            np.ceil(
                (self._BOT_WIDTH * 0.5 + self._BOT_MARGI)
                * self.extrinsic.cams[self.cam_label]["ppmy"]
            )
        )
        self.AR = (
            (0.01 + self._CURV_OFFSET) * self.curvature
            if self.curvature > 0.0
            else (0.01 - self._CURV_OFFSET) * self.curvature
        )
        self.AL = 0.01 * self.curvature
        self.CR = self.extrinsic.cams[self.cam_label]["view_coord"][0] + ct_dist
        self.CL = self.extrinsic.cams[self.cam_label]["view_coord"][0] - ct_dist
        # self.BL = 0.0
        # self.BR = 0.0

        # ---------------------------------------------------------------------
        # Get Left and right line points
        right_proj = []
        left_proj = []
        increment = 1
        idx_y = -(
            self.extrinsic.cams[self.cam_label]["view_coord"][1]
            - self.extrinsic.cams[self.cam_label]["unwarped_size"][1]
        )
        lout = False
        rout = False

        while (
            idx_y > -self.extrinsic.cams[self.cam_label]["view_coord"][1] + self.y_limit
        ):

            # Left projection line
            if not lout:
                lp = get_projection_point_src(
                    coords_dst=(
                        self.AL * (idx_y ** 2) + self.BL * idx_y + self.CL,
                        self.extrinsic.cams[self.cam_label]["view_coord"][1] + idx_y,
                        1,
                    ),
                    INVM=self.extrinsic.cams[self.cam_label]["M_inv"],
                )
                if self.distord_lines:
                    lp = get_distor_point(
                        pt=lp, mtx=self.extrinsic.mtx, dist=self.extrinsic.dist
                    )
                    in_cnt = cv2.pointPolygonTest(
                        contour=self.extrinsic.cams[self.cam_label]["waypoint_area"],
                        pt=lp,
                        measureDist=True,
                    )
                else:
                    in_cnt = cv2.pointPolygonTest(
                        contour=self.extrinsic.cams[self.cam_label]["undistord_cnt"],
                        pt=lp,
                        measureDist=True,
                    )
                if in_cnt >= 0:
                    left_proj.append(lp)
                else:
                    lout = True

            # Right projection line
            if not rout:
                rp = get_projection_point_src(
                    coords_dst=(
                        self.AR * (idx_y ** 2) + self.BR * idx_y + self.CR,
                        self.extrinsic.cams[self.cam_label]["view_coord"][1] + idx_y,
                        1,
                    ),
                    INVM=self.extrinsic.cams[self.cam_label]["M_inv"],
                )
                if self.distord_lines:
                    rp = get_distor_point(
                        pt=rp, mtx=self.extrinsic.mtx, dist=self.extrinsic.dist
                    )
                    in_cnt = cv2.pointPolygonTest(
                        contour=self.extrinsic.cams[self.cam_label]["waypoint_area"],
                        pt=rp,
                        measureDist=True,
                    )
                else:
                    in_cnt = cv2.pointPolygonTest(
                        contour=self.extrinsic.cams[self.cam_label]["undistord_cnt"],
                        pt=rp,
                        measureDist=True,
                    )
                if in_cnt >= 0:
                    right_proj.append(rp)
                else:
                    rout = True

            idx_y -= increment
            increment += 1

        self.left_proj = np.array(left_proj)
        self.right_proj = np.array(right_proj)

    def draw(self, img):

        # ---------------------------------------------------------------------
        # If not extrinsic calibration then continue
        if self.extrinsic.cams[self.cam_label] is None or (
            self.extrinsic.mtx is None and self.distord_lines
        ):
            return

        # ---------------------------------------------------------------------
        # Update params
        self.update_params()

        # ---------------------------------------------------------------------
        # Draw warped space
        if self._LOCAL_LAUNCH and self._SHOW_PROJ:
            self.draw_in_proj(img_src=img.copy())

        # ---------------------------------------------------------------------
        #  Draw waypoint area contour
        if self._GUI_WAYPOINT_AREA:
            cv2.drawContours(
                image=img,
                contours=[self.extrinsic.cams[self.cam_label]["waypoint_area"]],
                contourIdx=0,
                color=(0, 200, 0),
                thickness=2,
            )

        # ---------------------------------------------------------------------
        # Draw robots projection lines

        # Left side
        color_idx = 0
        thickness_idx = len(self._COLORS)
        for idx in range(len(self.left_proj) - 1):
            if self._proj_m[idx] > self._DISTAN_M[-1]:
                break

            # Draw horizontal lines in body projection
            while self._proj_m[idx] > self._DISTAN_M[color_idx]:
                color_idx += 1  # Change Color
                thickness_idx -= 1 if thickness_idx > 0 else thickness_idx
                cv2.line(
                    img=img,
                    pt1=(self.left_proj[idx][0], self.left_proj[idx][1]),
                    pt2=(
                        self.left_proj[idx][0] + self._HORZL_LEN,
                        self.left_proj[idx][1],
                    ),
                    color=self._COLORS[color_idx],
                    thickness=self._HOZ_THICK + thickness_idx,
                )

            # Draw vertical lines in body projection
            cv2.line(
                img=img,
                pt1=tuple(self.left_proj[idx]),
                pt2=tuple(self.left_proj[idx + 1]),
                color=self._COLORS[color_idx],
                thickness=self._VER_THICK + thickness_idx,
            )

        # right side
        color_idx = 0
        thickness_idx = len(self._COLORS)
        for idx in range(len(self.right_proj) - 1):
            if self._proj_m[idx] > self._DISTAN_M[-1]:
                break
            # Draw horizontal lines in body projection
            while self._proj_m[idx] > self._DISTAN_M[color_idx]:
                color_idx += 1  # Change Color
                thickness_idx -= 1 if thickness_idx > 0 else thickness_idx
                cv2.line(
                    img=img,
                    pt1=(self.right_proj[idx][0], self.right_proj[idx][1]),
                    pt2=(
                        self.right_proj[idx][0] - self._HORZL_LEN,
                        self.right_proj[idx][1],
                    ),
                    color=self._COLORS[color_idx],
                    thickness=self._HOZ_THICK + thickness_idx,
                )

            # Draw vertical lines in body projection
            cv2.line(
                img=img,
                pt1=tuple(self.right_proj[idx]),
                pt2=tuple(self.right_proj[idx + 1]),
                color=self._COLORS[color_idx],
                thickness=self._VER_THICK + thickness_idx,
            )

        # ---------------------------------------------------------------------
        # Print doted center line
        dotline(
            src=img,
            p1=self._cnt_line[0],
            p2=self._cnt_line[1],
            color=(255, 255, 255),
            thickness=self._VER_THICK,
            Dl=10,
        )

        # ---------------------------------------------------------------------
        # Draw waypoint coordinate
        if self.x_norm is not None and self.y_norm is not None:
            cv2.circle(
                img=img,
                center=(
                    int(self.x_norm * img.shape[1]),
                    int(self.y_norm * img.shape[0]),
                ),
                radius=10,
                color=(255, 255, 255) if self.incnt else (0, 0, 255),
                thickness=1,
            )
            cv2.circle(
                img=img,
                center=(
                    int(self.x_norm * img.shape[1]),
                    int(self.y_norm * img.shape[0]),
                ),
                radius=2,
                color=(0, 0, 255),
                thickness=-1,
            )
            if self._GUI_WAYPOINT_DESCRIPTION and self.incnt:
                cv2.line(
                    img=img,
                    pt1=(
                        int(self.x_norm * img.shape[1]),
                        int(self.y_norm * img.shape[0]),
                    ),
                    pt2=(
                        int(self.x_norm * img.shape[1] + 100),
                        int(self.y_norm * img.shape[0]),
                    ),
                    color=(255, 255, 255),
                    thickness=1,
                )
                print_text_list(
                    img=img,
                    tex_list=[
                        f"absm: {round(self.abs_m, 2)}",
                        f"deg: {round(self.angle, 2)}",
                        f"xm: {round(self.x_m, 2)}",
                        f"ym: {round(self.y_m, 2)}",
                    ],
                    color=(255, 255, 255),
                    orig=(
                        int(self.x_norm * img.shape[1] + 18),
                        int(self.y_norm * img.shape[0] + 15),
                    ),
                    fontScale=0.4,
                    y_jump=16,
                )

    def draw_in_proj(self, img_src, win_name="LOCAL_LAUNCH_IMG_PROJ"):

        view_coord = self.extrinsic.cams[self.cam_label]["view_coord"]
        unwarped_size = self.extrinsic.cams[self.cam_label]["unwarped_size"]

        # ---------------------------------------------------------------------
        # Get bird view image
        img_src = (
            cv2.undistort(img_src, self.extrinsic.mtx, self.extrinsic.dist, None, None)
            if img_src is not None
            else np.zeros(
                (
                    self.extrinsic.intrinsic.image_width,
                    self.extrinsic.intrinsic.image_height,
                    3,
                ),
                dtype=np.uint8,
            )
        )
        img_dst = cv2.warpPerspective(
            img_src,
            self.extrinsic.cams[self.cam_label]["M"],
            (unwarped_size[0], view_coord[1]),
        )

        # ---------------------------------------------------------------------
        # Draw waypoint coordinate
        if self.x_warp is not None and self.y_warp is not None:
            cv2.circle(
                img=img_dst,
                center=(self.x_warp, self.y_warp),
                radius=10,
                color=(255, 255, 255),
                thickness=1,
            )
            cv2.circle(
                img=img_dst,
                center=(self.x_warp, self.y_warp),
                radius=2,
                color=(0, 0, 255),
                thickness=-1,
            )

        # ---------------------------------------------------------------------
        # Draw some geometries
        cv2.line(
            img_dst,
            (0, unwarped_size[1]),
            (unwarped_size[0], unwarped_size[1]),
            (0, 0, 255),
            1,
        )
        cv2.line(
            img_dst,
            (view_coord[0], 0),
            (view_coord[0], view_coord[1]),
            (0, 255, 255),
            1,
        )
        cv2.circle(
            img=img_dst, center=view_coord, radius=5, color=(0, 0, 255), thickness=-1
        )

        # ---------------------------------------------------------------------
        # Get Left and right line points
        right_proj = []
        left_proj = []
        increment = 1
        idx_y = -(view_coord[1] - unwarped_size[1])
        while idx_y >= -view_coord[1] + self.y_limit:
            left_proj.append(
                (
                    int(self.AL * (idx_y ** 2) + self.BL * idx_y + self.CL),
                    view_coord[1] + idx_y,
                )
            )
            right_proj.append(
                (
                    int(self.AR * (idx_y ** 2) + self.BR * idx_y + self.CR),
                    view_coord[1] + idx_y,
                )
            )
            idx_y -= increment
            increment += 1
        left_proj = np.array(left_proj)
        right_proj = np.array(right_proj)

        color_idx = 0
        thickness_idx = len(self._COLORS)
        for idx in range(len(left_proj) - 1):
            if self._proj_m[idx] > self._DISTAN_M[-1]:
                break
            # Draw horizontal lines in body projection
            while self._proj_m[idx] > self._DISTAN_M[color_idx]:
                color_idx += 1  # Change Color
                thickness_idx -= 1 if thickness_idx > 0 else thickness_idx
                cv2.line(
                    img=img_dst,
                    pt1=(left_proj[idx][0], left_proj[idx][1]),
                    pt2=(left_proj[idx][0] + self._HORZL_LEN, left_proj[idx][1]),
                    color=self._COLORS[color_idx],
                    thickness=self._HOZ_THICK + thickness_idx,
                )
                cv2.line(
                    img=img_dst,
                    pt1=(right_proj[idx][0], right_proj[idx][1]),
                    pt2=(right_proj[idx][0] - self._HORZL_LEN, right_proj[idx][1]),
                    color=self._COLORS[color_idx],
                    thickness=self._HOZ_THICK + thickness_idx,
                )

            # Draw vertical lines in body projection
            cv2.line(
                img=img_dst,
                pt1=tuple(left_proj[idx]),
                pt2=tuple(left_proj[idx + 1]),
                color=self._COLORS[color_idx],
                thickness=self._VER_THICK + thickness_idx,
            )
            cv2.line(
                img=img_dst,
                pt1=tuple(right_proj[idx]),
                pt2=tuple(right_proj[idx + 1]),
                color=self._COLORS[color_idx],
                thickness=self._VER_THICK + thickness_idx,
            )

        # ---------------------------------------------------------------------
        cv2.imshow(win_name, img_dst)
        cv2.waitKey(1)


class RobotSubscriber:
    def __init__(self, parent_node):

        # Stitcher
        self.stream_stitch = False
        self._sub_stitch = parent_node.create_subscription(
            msg_type=Bool,
            topic="video_streaming/stitch",
            callback=self.cb_video_streaming_stitch,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

        # Rear camera
        self.stream_rear_cam = False
        self._sub_rear_cam = parent_node.create_subscription(
            msg_type=Bool,
            topic="video_streaming/rear_cam",
            callback=self.cb_video_streaming_rear_cam,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

        # Zoom variables
        self.zoom = False
        self.zoom_width = 0.1
        self.zoom_height = 0.25
        self.zoom_factor = 3.0
        # [%]xmin, [%]ymin, [%]xmax, [%]ymax
        self.zoom_roi = (0.0, 0.0, self.zoom_width, self.zoom_height)
        self._sub_zoom = parent_node.create_subscription(
            msg_type=Waypoint,
            topic="video_streaming/zoom",
            callback=self.cb_zoom,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

        # Door variables
        self.door_open = False
        self._sub_pwm = parent_node.create_subscription(
            msg_type=PWMOut,
            topic="pwm/output",
            callback=self.cb_sub_pwm,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

    def cb_sub_pwm(self, data):
        self.door_open = int(data.channels[2]) > 1435

    def cb_video_streaming_rear_cam(self, data):
        self.stream_rear_cam = not self.stream_rear_cam

    def cb_video_streaming_stitch(self, data):
        self.stream_stitch = not self.stream_stitch

    def cb_zoom(self, data):

        if (self.zoom_roi[0] < data.x < self.zoom_roi[2]) and (
            self.zoom_roi[1] < data.y < self.zoom_roi[3]
        ):
            self.zoom_roi = (0, 0, 0, 0)
            self.zoom = False
            return
        else:
            self.zoom = True

        x = data.x - self.zoom_width * 0.5
        y = data.y - self.zoom_height * 0.5

        if x + self.zoom_width > 1.0:
            x = 1.0 - self.zoom_width
        elif x < 0.0:
            x = 0.0

        if y + self.zoom_height > 1.0:
            y = 1.0 - self.zoom_height
        elif y < 0.0:
            y = 0.0

        self.zoom_roi = (x, y, x + self.zoom_width, y + self.zoom_height)


class ChassisSubscriber:
    def __init__(self, parent_node):

        self.error = [False, False, False, False, False]
        self.armed = False

        self._sub_motors_status = parent_node.create_subscription(
            msg_type=Motors,
            topic="/canlink/chassis/motors_status",
            callback=self.cb_chassis_motors,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

        self._sub_chassis_status = parent_node.create_subscription(
            msg_type=ChassisState,
            topic="/canlink/chassis/status",
            callback=self.cb_chassis_module,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

    def cb_chassis_motors(self, data):
        motor_errors = data.error_status
        for idx, status in enumerate(motor_errors):
            self.error[idx] = True if status > 0 else False

    def cb_chassis_module(self, data):
        self.error[-1] = not data.connected
        self.armed = data.armed


class DistanceSensorSuscriber:
    def __init__(self, parent_node, topic_name, min_range=1.0, max_range=10.0):

        self.min_range = min_range  # [m]
        self.max_range = max_range  # [m]
        self.range = min_range
        self.sub_dist_sensor = parent_node.create_subscription(
            msg_type=Range,
            topic=topic_name,
            callback=self.cb_dist_sensor,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

    def cb_dist_sensor(self, data):

        self.range = data.range if data.range < self.max_range else self.max_range
        self.min_range = data.min_range  # [m]
        self.max_range = data.max_range  # [m]


class CliffSensorSuscriber:
    def __init__(self, parent_node, topic_name):

        self.range = 0.0
        self.sub_cliff_sensor = parent_node.create_subscription(
            msg_type=Range,
            topic=topic_name,
            callback=self.cb_cliff_sensor,
            qos_profile=1,
            callback_group=parent_node.callback_group,
        )

    def cb_cliff_sensor(self, data):
        self.range = data.range


# =============================================================================
