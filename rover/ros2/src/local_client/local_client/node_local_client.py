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
import json
import os

from threading import Thread, Event

from vision.utils.vision_utils import printlog

from std_msgs.msg import Bool
from std_msgs.msg import String
from usr_msgs.msg import VisualMessage
from usr_msgs.msg import PWMOut
from usr_msgs.msg import State
from usr_msgs.msg import CaptureStatus
from geometry_msgs.msg import TwistStamped

from socketIO_client import SocketIO

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# =============================================================================
class WSClient(object):
    def __init__(self, host="localhost", port=4567):
        super(WSClient, self).__init__()

        self._port = port
        self._host = host

        self._on_connect = []
        self._on_steer = []
        self._sio = None

    def __del__(self):
        if self._sio:
            self._sio.dis_connect()

    def start(self):

        self._sio = SocketIO(self._host, self._port)

        self._sio.on("steer", self._steer)
        self._sio.on("connect", self._connect)

        self._thread = Thread(target=self._run)
        self._thread.daemon = True
        self._thread.start()

    def _run(self):
        self._sio.wait()

    def _steer(self, data):

        for f in self._on_steer:
            f(data)

    def _connect(self):
        for f in self._on_connect:
            f()

    def emit(self, **kwargs):
        self._sio.emit("telemetry", {key: str(value) for key, value in kwargs.items()})

    def on_message(self, f):

        self._on_steer.append(f)

        return f

    def on_connect(self, f):

        self._on_connect.append(f)

        return f


class RoverWSClient(object):
    def __init__(self, *args, **kwargs):

        super(RoverWSClient, self).__init__()

        self._port = kwargs["port"]
        self._host = kwargs["host"]

        self._on_connect = []
        self._on_steer = []
        self._sio = None

        self._wait_next = False
        self._wait_next = kwargs.pop("wait_next", False)
        self._emit_on_callback = kwargs.pop("emit_on_callback", False)
        self._command = kwargs.pop("initial_command", self.initial_command)
        self._data = kwargs.pop("initial_data", self.initial_data)
        self._socket_obj = kwargs.pop("socket_obj", None)

        self._socket_obj = WSClient(*args, **kwargs)
        self._socket_obj.on_message(self.on_message)
        self._socket_obj.start()

        if self._emit_on_callback:
            self._socket_obj.emit(**self._command)

    def on_message(self, data):
        self._data = data
        if self._emit_on_callback:
            self._socket_obj.emit(**self._command)

    def get_data(self, wait_next=None):
        wait_next = (
            wait_next if isinstance(wait_next, (bool, int, float)) else self._wait_next
        )

        if wait_next:
            if isinstance(wait_next, bool):
                self._data = None
                while self._data is None:
                    time.sleep(0.001)
            else:
                time.sleep(wait_next)

        return self._data

    def set_command(self, **kwargs):
        self._command = kwargs

    def emit(self, **kwargs):
        if kwargs:
            self._command = kwargs

        # print(self._socket_obj)
        self._socket_obj.emit(**self._command)

    def step(self, **kwargs):
        self.set_command(**kwargs)

        if not self._emit_on_callback:
            self.emit()

        return self.get_data()

    def reset(self):
        return self.get_data(wait_next=True)

    @property
    def initial_command(self):
        return dict()

    @property
    def initial_data(self):
        return dict()


class ClientNode(Node, Thread):
    def __init__(self, debugger=True):

        # ---------------------------------------------------------------------
        super().__init__("ClientNode")

        Thread.__init__(self)

        self._LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0))


        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()
        self.debugger = debugger
        self.rate = 15.0

        self.ws_client = RoverWSClient(**{"host": "localhost", "port": 4567})

        self.we_client_params = {
            "lid_opened": False,
            "recording": False,
            "enable_driving": False,
            "autonomous": False,
            "local": False,
            "steering_angle": 0.0,
            "throttle": 0.0,
        }

        # ---------------------------------------------------------------------
        # Publishers
        self.pub_arm_request = self.create_publisher(
            Bool, "/canlink/chassis/arm", 5, callback_group=self.callback_group,
        )

        self.pwm_msg = PWMOut()
        self.pwm_msg.channels = [0, 0, 0, 0, 0]
        self.pub_pwm = self.create_publisher(
            PWMOut, "pwm/output", 5, callback_group=self.callback_group,
        )

        self.control_msg = TwistStamped()
        self.pub_control = self.create_publisher(
            TwistStamped,
            "freedom_client/cmd_vel",
            5,
            callback_group=self.callback_group,
        )

        self.bool_msg = Bool()
        self.pub_streaming_idle_restart = self.create_publisher(
            Bool,
            "video_streaming/optimizer/idle_restart",
            1,
            callback_group=self.callback_group,
        )

        self.pub_data_capture = self.create_publisher(
            Bool,
            "data_capture/capture",
            1,
            callback_group=self.callback_group,
        )

        # ---------------------------------------------------------------------
        # Subscribers
        self.sub_ws_client_message = self.create_subscription(
            msg_type=VisualMessage,
            topic="video_streaming/visual_debugger",
            callback=self.cb_ws_client_message,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.data_capture_recording = False
        self.data_capture_images = 0
        self.data_capture_percentage = 100
        self.sub_data_capture_status = self.create_subscription(
            msg_type=CaptureStatus,
            topic="data_capture/status",
            callback=self.cb_data_capture_status,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.armed = False
        self.mode = "manual" # local
        self.sub_canlink_chassis_status = self.create_subscription(
            msg_type=State,
            topic="canlink/chassis/status",
            callback=self.cb_canlink_chassis_status,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        # ---------------------------------------------------------------------
        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.tick = time.time()
        # self.daemon = True
        self.start()

    def cb_ws_client_message(self, msg):
        try:
            client_msg_str = json.dumps({"data": msg.data})
            client_msg = json.loads(client_msg_str)
            if msg.type == "INFO":
                self.ws_client.emit(info=client_msg["data"])
            elif msg.type == "WARN":
                self._ws_client.emit(warning=client_msg["data"])
        except Exception as e:
            printlog(msg=e, msg_type="ERROR")

    def cb_data_capture_status(self, msg):
        try:
            self.data_capture_recording = msg.recording
            self.data_capture_images = msg.images
            self.data_capture_percentage = msg.percentage
        except Exception as e:
            printlog(msg=e, msg_type="ERROR")

    def cb_canlink_chassis_status(self, msg):
        self.armed = msg.armed

    def run(self):
        """ Cycle of threads execution
        Args:
        Returns:
        """

        control_update = False

        while True:
            try:
                self.tick = time.time()
                # -------------------------------------------------------------
                response = self.ws_client.get_data()
                if len(response):
                    for key, val in self.we_client_params.items():
                        if key not in response.keys():
                            printlog(
                                msg=f"{key} key no found in response dic",
                                msg_type="WARN",
                                flush=self.debugger,
                            )
                            self.we_client_params[key] = None
                            continue

                        if key == "steering_angle" or key == "throttle":
                            resp = float(response[key])
                            if val != resp:
                                self.we_client_params[key] = resp
                                printlog(
                                    msg=f"{key}: val {round(val, 4)} -> {round(resp, 4)}",
                                    msg_type="INFO",
                                    flush=self.debugger,
                                )
                                if key == "throttle":
                                    self.control_msg.twist.linear.x = resp / 1.5
                                elif key == "steering_angle":
                                    self.control_msg.twist.angular.z = resp / np.pi
                                control_update = True

                        else:
                            resp = response[key] == "true"
                            if val != resp:
                                self.we_client_params[key] = resp
                                printlog(
                                    msg=f"{key}: val {val} -> {response[key]}",
                                    msg_type="INFO",
                                    flush=self.debugger,
                                )
                            if key == "lid_opened":
                                self.pwm_msg.channels[2] = 2000 if resp else 0
                                self.pub_pwm.publish(self.pwm_msg)
                            elif key == "recording" and resp:
                                self.pub_data_capture.publish(self.bool_msg)
                            elif key == "enable_driving":
                                self.pub_arm_request.publish(self.bool_msg)
                            elif key == "autonomous" and resp:
                                self.we_client_params["local"] = False
                            elif key == "local" and resp:
                                self.we_client_params["autonomous"] = False
                                self.mode = "local"

                    self.ws_client.emit(
                        recording = self.data_capture_recording,
                        armed = self.armed,
                        mode = self.mode,
                        auto_active = self.we_client_params["autonomous"],
                        # steering = 0,
                        space_left=self.data_capture_percentage,
                        number_images=self.data_capture_images,
                    )

                    if (control_update 
                    and (self.we_client_params["local"] or self._LOCAL_RUN)
                    and (self.armed or self._LOCAL_RUN)):
                        self.pub_control.publish(self.control_msg)
                        self.pub_streaming_idle_restart.publish(self.bool_msg)
                        control_update = False

                # -------------------------------------------------------------
                # Operate times for next frame iteration
                tock = time.time() - self.tick
                twait = 1.0 / self.rate - tock
                if twait <= 0.0:
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
    client_node = ClientNode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(client_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
