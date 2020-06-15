#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import subprocess
import datetime
import time
import cv2
import os

from glob import glob

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from threading import Thread, Event

from vision.utils.vision_utils import printlog

# =============================================================================
class DataCaptureNode(Node, Thread):
    def __init__(self):
        """     
            Object class constructor
        Args:
        Returns:
        """

        # ---------------------------------------------------------------------
        super().__init__("DataCaptureNode")

        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        Thread.__init__(self)

        # ---------------------------------------------------------------------
        self.rate = 20

        self.quality = int(os.getenv(key="DATA_CAPTURE_IMG_QUALITY", default=80))
        self.write_imgs = int(os.getenv(key="DATA_CAPTURE_WRITE_IMAGES", default=1))
        self.write_csv = int(os.getenv(key="DATA_CAPTURE_WRITE_DATA", default=1))
        self.capture_id = 0  # Current data capture identifier for the csv file
        self.recording = False  # Enable/Disable data recording
        self.space_left = 100.0  # Space lef in usb device
        self.dest_folder = None
        self.csv_file = None
        self.csv_header = []
        self.ready = False
        self.num_imgs = 0

        # ---------------------------------------------------------------------
        # Check for connected usb and check conditions and create folder
        date = datetime.date.today().strftime("%m-%d-%y")  # Get current date
        device = None
        sub_folder_path = "data"  # Get base path
        usb_devices = self.get_mount_points()  # Get connected usb devices
        if len(usb_devices):
            # Get absolute path to root in usb device
            if usb_devices[-1][0][1:].split("/")[0] == "dev":
                self.dest_folder = usb_devices[-1][-1]
                device = usb_devices[-1][0]  # Get usb device
                self.dest_folder = os.path.join(
                    self.dest_folder, "data_capture-{}".format(date)
                )
                printlog(msg=f"USB Detected: {device}", msg_type="INFO")

        # ---------------------------------------------------------------------
        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.tick = time.time()
        # self.daemon = True
        self.start()

    def create_folder_csv_4data_capture(self):
        """ creates data.csv headers if it not exists and Creates folder for 
            capturing data
        Args:
        Returns:
            csv_file: `string` csv file absolute path
            message: `string` message for debugger
        """

        # Create folder if does not exits
        if os.path.exists(self.dest_folder):
            message = "Folder for datacapture exists"
        else:
            message = "Folder for datacapture does not exist"
            os.mkdir(self.dest_folder)

        # Path to csv file
        csv_file = os.path.join(self.dest_folder, "data.csv")

        # If csv file does not exits in destination path
        if not os.path.isfile(csv_file):
            with open(csv_file, "a") as fd:
                writer = csv.writer(fd)
                writer.writerow(self.csv_header)

        return csv_file, message

    def get_usb_devices(self):
        sdb_devices = list(map(os.path.realpath, glob("/sys/block/sd*")))
        usb_idx = None
        for idx, dev in enumerate(sdb_devices):
            if "usb" in dev:
                usb_idx = idx
        if usb_idx is not None:
            usb_devices = {sdb_devices[usb_idx][-1], sdb_devices[usb_idx]}
            return usb_devices
        else:
            return []

    def get_mount_points(self, devices=None):
        devices = devices or self.get_usb_devices()
        output = subprocess.check_output(["mount"]).splitlines()
        print("*"*100, output, flush=True)
        is_usb = lambda path: any(dev in path for dev in devices)
        usb_info = (line for line in output if is_usb(line.split()[0]))
        return [(info.split()[0], info.split()[2]) for info in usb_info]

    def usb_space_left(self, device_path, percentage=True):
        """ calculates left space in device
        Args:
            device_path: `string` absolute path to device
            percentage: `boolean` return value in percentage
        Returns:
            _: `float` total of space in device in bytes or percentage
        """

        if device_path is None:
            return 100
        try:
            disk = os.statvfs(device_path)
        except Exception as e:
            logerr_cond(True, "Error reading USB {}".format(e))
            return -1
        totalBytes = float(disk.f_bsize * disk.f_blocks)
        totalAvailSpace = float(disk.f_bsize * disk.f_bfree)
        if percentage:
            return totalAvailSpace / totalBytes * 100.0
        else:
            return totalAvailSpace / 1024.0 / 1024.0 / 1024.0

    def write_images(self, images, img_format="jpg"):
        """ Write images in absolute path destination
        Args:
            images: `list` of cv2.math images to save in path
            dest: `string` absolute path to save images
            cam_label: `list` of strings with camera labels
            quality: `int` percentage of quality to save images
            images: `string` file extension/format to save images
        Returns:
            timestamp: `timestamp` current timestamp with the moment when images were saved
            images: `list` of string with the absolute path where images were saved
        """

        timestamp = int(time.time() * 1000)  # Get current time in timestamp format
        prefix = binascii.b2a_hex(os.urandom(2))  # Get a marihunero prefix
        cam_file_names = {}

        for cam_label, img in images.items():
            file_name = "{}-{}_{}.{}".format(prefix, timestamp, cam_label, img_format)
            cv2.imwrite(
                os.path.join(self.dest_folder, "data", file_name),
                img,
                [cv2.IMWRITE_JPEG_QUALITY, self.quality],
            )
            cam_file_names[cam_label] = file_name
            self.num_imgs += 1

        return timestamp, cam_file_names

    def run(self):
        """ Cycle of threads execution
        Args:
        Returns:
        """

        while True:
            self.tick = time.time()
            try:

                pass

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
    capture_node = DataCaptureNode()

    # Runs callbacks in a pool of threads.
    executor = MultiThreadedExecutor()

    # Execute work and block until the context associated with the
    # executor is shutdown. Callbacks will be executed by the provided
    # executor.
    rclpy.spin(capture_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    capture_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()

# =============================================================================
