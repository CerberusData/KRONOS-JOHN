import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time

class VisualDebugger(Node):

    def __init__(self, timer=10):

        super().__init__('VisualDebugger')

        self._subcriber = self.create_subscription(
            msg_type=String, topic='video_streaming/visual_debugger', 
            callback=self.debugger_cb, qos_profile=5)

        self._subcriber

        self.msg = "" # Message to show in console
        # self.type = "" # Type of message "info, err, warn"
        self.timer = timer # Timer with time to show message

    def debugger_cb(self, msg):

        print("*"*100, flush=True)
        self.msg = msg.data
        #self.type = data.type
        time.sleep(self.timer)
        self.msg = ""
        print("+"*100, flush=True)
