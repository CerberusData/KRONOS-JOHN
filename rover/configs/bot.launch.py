#!/usr/bin/env python3

# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Examples:
    https://github.com/ros2/launch/blob/a89671962220c8691ea4f128717bca599c711cda/launch/examples/launch_counters.py

"""

# =============================================================================
import inspect
import yaml
import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from typing import cast

# ============================================================================
class bcolors:
    LOG = {
        "WARN": ["\033[33m", "WARN"],
        "ERROR": ["\033[91m", "ERROR"],
        "OKGREEN": ["\033[32m", "INFO"],
        "INFO": ["\033[0m", "INFO"],  # ['\033[94m', "INFO"],
        "BOLD": ["\033[1m", "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = "\033[1m"
    ENDC = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    GRAY = "\033[90m"
    UNDERLINE = "\033[4m"


def printlog(msg, msg_type="INFO", flush=True):
    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(bcolors.LOG[msg_type][1], org, caller, msg)
    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=flush)


def read_node_launch(default_file):

    CONF_PATH = os.path.dirname(os.path.abspath(__file__))
    CONF_FILE = "nodes_local_launch.yaml"
    FILE_PATH = os.path.join(CONF_PATH, CONF_FILE)

    if not os.path.exists(FILE_PATH):

        try:
            with open(FILE_PATH, "w") as outfile:
                yaml.dump(default_file, outfile, default_flow_style=False)
            printlog(msg="Nodes launch file created", msg_type="WARN")

        except Exception as e:
            printlog(
                msg="Error creating nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_file

    else:

        try:
            with open(FILE_PATH, "r") as stream:
                default_file = yaml.safe_load(stream)
            printlog(
                msg="Nodes local launch file {}".format(CONF_FILE), msg_type="OKGREEN"
            )

        except Exception as e:
            printlog(
                msg="Error reading nodes launch file: {}".format(e), msg_type="ERROR"
            )

        return default_file


def generate_launch_description():

    ld = launch.LaunchDescription(
        [launch.actions.LogInfo(msg="Launching Kiwibot ROS2 ..."),]
    )

    # -------------------------------------------------------------------------
    # Add here your ROS2 node actions and logics
    nodes = {
        # ---------------------------------------------------------------------
        # Vision Nodes
        "NODE_VIDEO_MAPPING": {
            "node_executable": "video_mapping",
            "node_name": "video_mapping",
            "package": "vision",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_VIDEO_MAPPING", default=0)),
        },
        "NODE_VIDEO_CALIBRATION": {
            "node_executable": "video_calibrator",
            "node_name": "video_calibrator",
            "package": "vision",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_VIDEO_CALIBRATION", default=0)),
        },
        "NODE_VIDEO_PARTICLE": {
            "node_executable": "video_particle",
            "node_name": "video_particle",
            "package": "vision",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_VIDEO_PARTICLE", default=0)),
        },
        "NODE_LOCAL_CONSOLE": {
            "node_executable": "local_console",
            "node_name": "local_console",
            "package": "vision",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_LOCAL_CONSOLE", 
                default=1 if int(os.getenv(key="LOCAL_LAUNCH", default=0)) else 0)),
        },
        # ---------------------------------------------------------------------
        # Control Nodes
        "NODE_CANLINK_CHASSIS": {
            "node_executable": "canlink_chassis",
            "package": "canlink",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_CANLINK_CHASSIS", default=0)),
        },
        "NODE_CANLINK_CABIN": {
            "node_executable": "canlink_cabin",
            "package": "canlink",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_CANLINK_CABIN", default=0)),
        },
        # ---------------------------------------------------------------------
        # Local node
        "NODE_LOCAL_CLIENT": {
            "node_executable": "local_client",
            "package": "local_client",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_LOCAL_CLIENT", default=1)),
        },
        # ---------------------------------------------------------------------
        # Data Capture
        "NODE_DATA_CAPTURE": {
            "node_executable": "data_capture",
            "package": "data_capture",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_DATA_CAPTURE", default=1)),
        },
        # ---------------------------------------------------------------------
        # Incredible node
        "NODE_INCREDIBLE_NODE": {
            "node_executable": "control_node",
            "package": "incredible_control",
            "output": "screen",
            "launch": int(os.getenv(key="NODE_INCREDIBLE_NODE", default=1)),
        },
    }

    if os.getenv(key="LOCAL_LAUNCH", default=0):
        nodes = read_node_launch(default_file=nodes)

    # Print nodes to launch
    srt_ = "\n\nLaunching:"
    for key, node_args in nodes.items():
        if node_args["launch"]:
            srt_ = srt_ + "\n\tNode {}\tfrom {} package".format(
                node_args["node_name"]
                if "node_name" in node_args.keys()
                else "(Executable)",
                node_args["package"],
            )
    ld = launch.LaunchDescription([launch.actions.LogInfo(msg=srt_ + "\n"),])

    for key, node_args in nodes.items():
        if node_args["launch"]:
            if "node_name" in node_args.keys():
                ld.add_action(
                    launch_ros.actions.Node(
                        node_executable=node_args["node_executable"],
                        node_name=node_args["node_name"],
                        package=node_args["package"],
                        output=node_args["output"],
                    )
                )
            else:
                ld.add_action(
                    launch_ros.actions.Node(
                        node_executable=node_args["node_executable"],
                        package=node_args["package"],
                        output=node_args["output"],
                    )
                )

    # -------------------------------------------------------------------------
    # Add here your python scripts
    # ld.add_action(launch.actions.ExecuteProcess(
    #     cmd=[sys.executable, '-u', './script_name.py', '--ignore-sigint', '--ignore-sigterm']
    # ))

    return ld

    # ============================================================================
