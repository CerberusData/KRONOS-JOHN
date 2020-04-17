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
def generate_launch_description():

    ld = launch.LaunchDescription([
        launch.actions.LogInfo(msg='Launching Kiwibot ROS2 ...'),
    ])

    # Add here your ROS2 node actions and logics
    nodes = []
    nodes.append(launch_ros.actions.Node(
        node_executable='video_mapping', 
        node_name='video_mapping',
        package='vision', 
        output='screen')
        )

    nodes.append(launch_ros.actions.Node(
        node_executable='video_calibrator', 
        node_name='video_calibrator',
        package='vision', 
        output='screen')
        )

    # nodes.append(launch_ros.actions.Node(
    #     node_executable='video_particle', 
    #     node_name='video_particle',
    #     package='vision', 
    #     output='screen')
    #     )

    for node in nodes:
        ld.add_action(node)

    # Add here your python scripts 
    # ld.add_action(launch.actions.ExecuteProcess(
    #     cmd=[sys.executable, '-u', './script_name.py', '--ignore-sigint', '--ignore-sigterm']
    # ))
    
    return ld