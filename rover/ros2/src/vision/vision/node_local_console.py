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

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range

from usr_msgs.msg import Control as WebControl
from usr_msgs.msg import PWMOut
from usr_msgs.msg import Motors
from std_msgs.msg import String
from std_msgs.msg import Bool

from cv_bridge import CvBridge, CvBridgeError

from vision.utils.vision_utils import printlog

from usr_msgs.msg import Waypoint

# =============================================================================
class LocalConsoleNode(Node, Thread):
    def __init__(self, streaming_topic="streaming/cam_central", win_rate=30.0):
        """ Object class constructor
        Args:
            streaming_topic: `string` topic of video streaming
            win_rate: `float` window rate 
        Returns:
        """

        # ---------------------------------------------------------------------
        super().__init__("LocalConsoleNode")
        Thread.__init__(self)

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # window properties
        self.win_name = "LOCAL_CONSOLE"
        self.win_mouse_click = None
        self.win_time = int(1000 / win_rate)
        cv2.namedWindow(self.win_name)
        cv2.setMouseCallback(self.win_name, self.cb_mouse_event)

        # window image variables
        self.streaming_img = np.zeros((300, 300, 3), np.uint8)
        self.streaming_topic = streaming_topic
        self.img_bridge = CvBridge()

        # ---------------------------------------------------------------------
        # Subscribers
        self.sub_streaming = self.create_subscription(
            msg_type=Image,
            topic=self.streaming_topic,
            callback=self.cb_streaming_img,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        # ---------------------------------------------------------------------
        # Publishers
        self.pub_cam_calibrate_msg = String()
        self.pub_cam_calibrate = self.create_publisher(
            String,
            "video_calibrator/calibrate_cam",
            1,
            callback_group=self.callback_group,
        )

        self.pub_video_streaming_stitch = self.create_publisher(
            Bool, "video_streaming/stitch", 1, callback_group=self.callback_group
        )

        self.pub_video_streaming_rear_cam = self.create_publisher(
            Bool, "video_streaming/rear_cam", 1, callback_group=self.callback_group
        )

        self.waypoint_msg = Waypoint()
        self.pub_video_streaming_waypoint = self.create_publisher(
            Waypoint,
            "video_streaming/waypoint_pt",
            1,
            callback_group=self.callback_group,
        )

        self.web_client_control_msg = WebControl()
        self.pub_web_client_control = self.create_publisher(
            WebControl, "web_client/control", 1, callback_group=self.callback_group
        )

        self.pwm_msg = PWMOut()
        self.pwm_msg.channels = [0, 0, 0, 0, 0]
        self.pub_pwm = self.create_publisher(
            PWMOut, "pwm/output", 1, callback_group=self.callback_group
        )

        self.motors_msg = Motors()
        self.motors_msg.error_status = [0, 0, 0, 0, 0]
        self.sim_motros_report = False
        self.pub_motors = self.create_publisher(
            Motors,
            "/canlink/chassis/motors_status",
            1,
            callback_group=self.callback_group,
        )

        self.sim_cliff_sensors = False
        self.dist_sensor_msg = Range()
        self.dist_sensor_msg.max_range = 10.0
        self.dist_sensor_msg.min_range = 1.0
        self.dist_sensor_inc = 0.2
        self.pubs_dist_sensors = [
            self.create_publisher(Range, topic, 1, callback_group=self.callback_group)
            for topic in [
                "/tf_mini_plus/distance_sensor3",
                "/tf_mini_plus/distance_sensor2",
                "/tf_mini_plus/distance_sensor1",
            ]
        ]

        self.sim_dist_sensors = False
        self.cliff_sensor_inc = 0.0
        self.cliff_sensor_msg = Range()
        self.pubs_cliff_sensors = [
            self.create_publisher(Range, topic, 1, callback_group=self.callback_group)
            for topic in ["/tf_mini_plus/cliff_sensor1", "/tf_mini_plus/cliff_sensor2",]
        ]

        self.bool_msg = Bool()
        self.pubs_streaming_idle_restart = self.create_publisher(
            Bool,
            "video_streaming/optimizer/idle_restart",
            1,
            callback_group=self.callback_group,
        )

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
            cv2.line(
                img=img,
                pt1=(
                    int(self.win_mouse_click[0] * img.shape[1] - 10),
                    int(self.win_mouse_click[1] * img.shape[0]),
                ),
                pt2=(
                    int(self.win_mouse_click[0] * img.shape[1] + 10),
                    int(self.win_mouse_click[1] * img.shape[0]),
                ),
                color=(0, 0, 255),
                thickness=2,
            )
            cv2.line(
                img=img,
                pt1=(
                    int(self.win_mouse_click[0] * img.shape[1]),
                    int(self.win_mouse_click[1] * img.shape[0] - 10),
                ),
                pt2=(
                    int(self.win_mouse_click[0] * img.shape[1]),
                    int(self.win_mouse_click[1] * img.shape[0] + 10),
                ),
                color=(0, 0, 255),
                thickness=2,
            )
            cv2.circle(
                img=img,
                center=(
                    int(self.win_mouse_click[0] * img.shape[1]),
                    int(self.win_mouse_click[1] * img.shape[0]),
                ),
                radius=2,
                color=(0, 255, 255),
                thickness=-1,
            )

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
                img_msg=data, desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            printlog(
                msg="Subscriber for camera in topic {}, {}".format(
                    self.streaming_topic, e
                ),
                msg_type="ERROR",
            )

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

        x = x / self.streaming_img.shape[1]
        y = y / self.streaming_img.shape[0]

        # Add point
        if event == cv2.EVENT_LBUTTONDOWN:
            self.win_mouse_click = (x, y)

            try:
                # Publish waypoint
                self.waypoint_msg.x = x
                self.waypoint_msg.y = y
                self.pub_video_streaming_waypoint.publish(self.waypoint_msg)

            except Exception as e:
                printlog(
                    msg="Error publishing waypoint coord " "trough topic, {}".format(e),
                    msg_type="ERROR",
                )

    def cb_key_event(self, key):
        """
            Get and execute action from users key
            Args:
                key: 'int' keyboard actions
            returns:
        """

        self.web_client_control_msg.pan = 0.0
        self.web_client_control_msg.speed = 0.0
        self.web_client_control_msg.tilt = 0.0

        # If pressed No key then continue
        if key == -1:
            pass
        # If pressed 1 Key then calibrate camera LL
        elif key == 49:
            self.pub_cam_calibrate_msg.data = "LL"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
            return
        # If pressed 2 Key then calibrate camera C
        elif key == 50:
            self.pub_cam_calibrate_msg.data = "C"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
            return
        # If pressed 3 Key then calibrate camera RR
        elif key == 51:
            self.pub_cam_calibrate_msg.data = "RR"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
            return
        # If pressed 4 key then calibrate camera B
        elif key == 52:
            self.pub_cam_calibrate_msg.data = "B"
            self.pub_cam_calibrate.publish(self.pub_cam_calibrate_msg)
            return
        # If pressed right key then move to right
        elif key == 81:
            self.web_client_control_msg.tilt = 100.0
            self.pub_web_client_control.publish(self.web_client_control_msg)
        # If pressed up key then move to forward
        elif key == 82:
            self.web_client_control_msg.speed = 100.0
            self.pub_web_client_control.publish(self.web_client_control_msg)
        # If pressed left key then move to left
        elif key == 83:
            self.web_client_control_msg.tilt = -100.0
            self.pub_web_client_control.publish(self.web_client_control_msg)
        # If pressed down key then move to backwards
        elif key == 84:
            self.web_client_control_msg.speed = -100.0
        # If pressed A key then switch to left camera
        elif key == 97:
            self.web_client_control_msg.pan = -100.0
            self.pub_web_client_control.publish(self.web_client_control_msg)
        # If pressed H key then print help
        elif key == 104:
            print(
                f"\n\t1 - Calibrate camera LL"
                f"\n\t2 - Calibrate camera C"
                f"\n\t3 - Calibrate camera RR"
                f"\n\t4 - Calibrate camera B\n"
                f"\n\tA - Show left camera"
                f"\n\tD - Show right camera"
                f"\n\tM - Activate/deactivate waypoint"
                f"\n\tP - Open lid"
                f"\n\tR - Switch to rear camera"
                f"\n\tZ - Simulate motor and chassis error"
                f"\n\tX - Simulate distance sensors"
                f"\n\tC - Simulate cliff sensors\n"
                f"\n\tLeft row - Move left"
                f"\n\tRight row - Move right"
                f"\n\tUp row - Move forward"
                f"\n\tDown row - Move backwards\n",
                flush=True,
            )
            return
        # If pressed M key then switch between manual and waypoint mode
        elif key == 109:
            pass
            return
        # If pressed D key then switch to right camera
        elif key == 100.0:
            self.web_client_control_msg.pan = 100.0
            self.pub_web_client_control.publish(self.web_client_control_msg)
        # If pressed P key then open the lid
        elif key == 112:
            self.pwm_msg.channels[2] = 2000 if self.pwm_msg.channels[2] < 300 else 0
            self.pub_pwm.publish(self.pwm_msg)
            return
        # If pressed R key then switch to rear camera
        elif key == 114:
            msg = Bool()
            msg.data = True
            self.pub_video_streaming_rear_cam.publish(msg)
            return
        # If pressed S key then activate/desactivate stitching mode
        elif key == 115:
            msg = Bool()
            msg.data = True
            self.pub_video_streaming_stitch.publish(msg)
            return
        # If pressed Z key then simulate chasis errors
        elif key == 122:
            self.sim_motros_report = not self.sim_motros_report
            if self.sim_motros_report:
                self.motors_msg.error_status = [1, 1, 1, 1, 1]
                self.pub_motors.publish(self.motors_msg)
            else:
                self.motors_msg.error_status = [0, 0, 0, 0, 0]
                self.pub_motors.publish(self.motors_msg)
            return
        # If pressed X key then simulate distance sensors
        elif key == 120:
            self.sim_dist_sensors = not self.sim_dist_sensors
            if not self.sim_dist_sensors:
                self.dist_sensor_msg.range = self.dist_sensor_msg.min_range
                for pub in self.pubs_dist_sensors:
                    pub.publish(self.dist_sensor_msg)
        # If pressed C key then simulate cliff sensors
        elif key == 99:
            self.sim_cliff_sensors = not self.sim_cliff_sensors
            self.cliff_sensor_msg.range = 1.0 if self.sim_cliff_sensors else 0.0
            for pub in self.pubs_cliff_sensors:
                pub.publish(self.cliff_sensor_msg)
        # If pressed no key defined then print message
        else:
            printlog(msg=f"{key} key action no defined", msg_type="WARN")
            return

        # ---------------------------------------------------------------------
        if key != -1:
            self.pubs_streaming_idle_restart.publish(self.bool_msg)

        # ---------------------------------------------------------------------
        if self.sim_dist_sensors:
            if self.dist_sensor_msg.range >= self.dist_sensor_msg.max_range:
                self.dist_sensor_inc = -abs(self.dist_sensor_inc)
                self.dist_sensor_msg.range = self.dist_sensor_msg.max_range
            elif self.dist_sensor_msg.range <= self.dist_sensor_msg.min_range:
                self.dist_sensor_inc = abs(self.dist_sensor_inc)
                self.dist_sensor_msg.range = self.dist_sensor_msg.min_range
            self.dist_sensor_msg.range += self.dist_sensor_inc
            for pub in self.pubs_dist_sensors:
                pub.publish(self.dist_sensor_msg)

    def run(self):
        """
            Run cycle of threads
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
    printlog(msg="press H Key for help", msg_type="INFO")

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(local_console_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    local_console_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
