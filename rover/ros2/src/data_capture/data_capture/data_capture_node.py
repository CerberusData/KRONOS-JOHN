#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus, Computer Vision & Ai Team
"""

# =============================================================================
import numpy as np
import sys
import csv
import os

import subprocess
import binascii
import datetime
import rospy
import time
import cv2

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from video_mapping.srv import CamerasStatus
from vision.srv import VideoStreamingConf
from std_msgs.msg import Bool

from easy_memmap import MultiImagesMemmap
from data_capture.msg import Status
from glob import glob

from python_utils.suscribers import OdometrySuscriber
from python_utils.suscribers import ActuatorControlSuscriber
from python_utils.suscribers import WanNetworksSuscriber
from python_utils.suscribers import WebClientSuscriber

# =============================================================================
class DataCapture():

    def __init__(self, debugger):
        """ initializes class
        Args:
            csv_file: `string` csv absolute path to save images
            dest_folder: `string` destination folder to save images
            data_capture: `boolean` Enable/Disable data capture process
        Returns:
        """

        # ---------------------------------------------------------------------
        # Subscriber to enable/disable data capturing
        rospy.Subscriber(name="/data_capture/capture", data_class=Bool, 
            callback=self.capture_cb, queue_size=2)

        self.status_pub = rospy.Publisher(name='/data_capture/status', 
            data_class=Status, queue_size=2)
        self._status = Status()

        self.subs_actuator_control = ActuatorControlSuscriber()
        self.subs_wan_networks = WanNetworksSuscriber()
        self.subs_odometry = OdometrySuscriber()
        self.subs_webclient = WebClientSuscriber()

        # ---------------------------------------------------------------------
        self.quality = int(os.getenv(key='DATA_CAPTURE_IMG_QUALITY', default=80))
        self.write_imgs = int(os.getenv(key='DATA_CAPTURE_WRITE_IMAGES', default=1))
        self.write_csv = int(os.getenv(key='DATA_CAPTURE_WRITE_DATA', default=1))
        self.capture_id = 0 # Current data capture identifier for the csv file
        self.recording = False # Enable/Disable data recording
        self.space_left = 100. # Space lef in usb device
        self.debugger = debugger
        self.dest_folder = None
        self.csv_file = None
        self.csv_header = []
        self.ready = False
        self.num_imgs = 0
        
        # ---------------------------------------------------------------------
        # video streaming status service to check video streaming configuration
        self.streaming_status_service = rospy.ServiceProxy(
            name='kiwibot/video_streaming_status', 
            service_class=VideoStreamingConf)
        self.streaming_status = {"video_width":0, "video_height":0, "video_rate":0,
            "video_color":0, "quality":0}

        # ---------------------------------------------------------------------
        # Camera status service to check state of cameras 
        self.camera_status_service = rospy.ServiceProxy(
            name='video_mapping/cameras_status', 
            service_class=CamerasStatus)

        # Get cameras status and labels of cameras working
        self.cameras_status = self.get_camera_Status() # Get camera status
        
        self.csv_header = (
            ['capture_id', 'timestamp'] +
            ["CAM{}_file".format(cam_label) for cam_label, status in self.cameras_status.items()] +
            ["odom_roll[rad]", "odom_pitch[rad]", "odom_yaw[rad]"] +
            ["actuator_steering", "actuator_throttle"] +
            ["pepwave_timestamp", "wan_upload_speed [Kbs]", "wan_download_speed [Kbs]"] +
            ["gps_status", "gps_vdop", "gps_hdop", "gps_pdop", "gps_timestamp"] +
            ["gps_altitude", "gps_longitude", "gps_latitude", "gps_speed"] +
            ["sim1_carrier", "sim1_signalLevel", "sim1_rssi", "sim1_rsrq", "sim1_rsrp", "sim1_sinr"] +
            ["sim2_carrier", "sim2_signalLevel", "sim2_rssi", "sim2_rsrq", "sim2_rsrp", "sim2_sinr"] +
            ["hb_latency[sec]", "hb_latency[nsec]", "pilot_status", 
            "video_width[pix]", "video_height[pix]", "video_rate[fps]", 
            "video_color", "video_quality[%]"])

        # ---------------------------------------------------------------------
        # Check for connected usb and check conditions and create folder
        date = datetime.date.today().strftime("%m-%d-%y") # Get current date
        device = None; sub_folder_path = "data" # Get base path
        usb_devices = self.get_mount_points() # Get connected usb devices
        if len(usb_devices):
            # Get absolute path to root in usb device
            if usb_devices[-1][0][1:].split("/")[0] == "dev":
                self.dest_folder = usb_devices[-1][-1] 
                device = usb_devices[-1][0] # Get usb device 
                self.dest_folder = os.path.join(
                    self.dest_folder,"data_capture-{}".format(date))
                self.debugger.debugger(DEBUG_LEVEL_0, "USB Detected: {}".format(device), 
                    log_type='info')

        # If path already exits then rename destination folder with new index
        if self.dest_folder is not None:
            if os.path.isdir(self.dest_folder):
                i=1; axu_dest=self.dest_folder+"({})".format(i)
                while(os.path.isdir(axu_dest)):
                    i+=1; axu_dest=self.dest_folder+"({})".format(i)
                self.dest_folder=axu_dest
            # Create subfolder to save images
            sub_folder_path = os.path.join(self.dest_folder, sub_folder_path)
            if not os.path.isdir(sub_folder_path):
                os.makedirs(str(sub_folder_path))
        else:
            self.csv_file, log_msg = None, "USB device no found or it may have been disconnected"
            self.debugger.debugger(DEBUG_LEVEL_0, log_msg, log_type='warn')
            return
        try: # Create data.csv headers if it not exists
            self.debugger.debugger(DEBUG_LEVEL_0, "Destination folder: {}".format(
                self.dest_folder), log_type='info')
            self.csv_file, log_msg = self.create_folder_csv_4data_capture()
        except Exception as err:
            self.csv_file, log_msg = None, "Error creating csv file"
            self.debugger.debugger(DEBUG_LEVEL_0, err, log_type='err')

        # ---------------------------------------------------------------------
        file1 = open(os.path.join(self.dest_folder, "env_variables.txt"), "a") 
        for key, value in os.environ.items():
            file1.write("{}={}\n".format(key, value)) 
        file1.close() 

        # ---------------------------------------------------------------------
        # Variables to record video
        # Define the codec and create VideoWriter object
        self.video_record = int(os.getenv(key='DATA_CAPTURE_RECORD_VIDEO', default=1))
        self.video_fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        self.video_fps = int(os.getenv(key='DATA_CAPTURE_RATE', default=10))
        self.video_width = int(os.getenv(key='VIDEO_WIDTH', default=640))
        self.video_height = int(os.getenv(key='VIDEO_HEIGHT', default=80))
        self.video_out = None

        if self.video_record and self.dest_folder is not None:
            self.debugger.debugger(DEBUG_LEVEL_0, "video recording enabled", 
                log_type='info')

        # ---------------------------------------------------------------------

    def publish_status(self):
        """ Publish data capture status
        Args:
        Returns:
        """

        self._status.recording = bool(self.recording)
        self._status.number_images = int(self.num_imgs)
        self._status.space_left = float(100.-self.space_left)
        self.status_pub.publish(self._status)

    def capture_cb(self, data):
        """ Callback function to update data capture class
        Args:
            data: `Bool` data from message
        Returns:
        """

        self.ready = False
        # check if usb is right mounted and with space left
        self.space_left = self.usb_space_left(
            device_path=self.dest_folder, 
            percentage=True)

        # -----------------------------------------------------------------
        # Check possible error cases
        # 1 - Check for space in storing device
        if self.space_left <= float(os.getenv('DATA_CAPTURE_MIN_USB_SPACE', 3)):
            self.debugger.debugger(DEBUG_LEVEL_0, "Can't record video, USB FULL", 
                log_type = 'warn')
            self.recording = False
        
        # 2 - Check if the usb device is connected
        elif self.dest_folder is None:
            self.debugger.debugger(DEBUG_LEVEL_0, "Can't record video, USB is not mounted", 
                log_type = 'warn')
            self.recording = False

        # 3 - Check if there's no csv file
        elif self.csv_file is None:
            self.debugger.debugger(DEBUG_LEVEL_0, "Can't record video, some problems with the USB", 
                log_type = 'warn')
            self.recording = False

        # -----------------------------------------------------------------
        self.recording = not self.recording
        if self.recording: 
            self.debugger.debugger(DEBUG_LEVEL_0, "Data recording {} started".format(
                self.capture_id), log_type = 'info')
            if self.video_record and self.dest_folder is not None:
                date = str(datetime.date.today().strftime("%m%d%y"))
                file_name = "{}_{}.{}".format(date, self.capture_id, "avi")
                file_path = os.path.join(self.dest_folder, file_name)
                self.video_out = cv2.VideoWriter(
                    file_path, self.video_fourcc, self.video_fps, 
                    (self.video_width, self.video_height))
        else: 
            self.debugger.debugger(DEBUG_LEVEL_0, "Data recording {} stopped".format(
                self.capture_id), log_type = 'info')
            self.capture_id += 1 # Increment capture identifier
            if self.video_record:
                self.video_out.release()
            self.publish_status()
        self.ready = True

    def get_camera_Status(self):

        cameras_status_str = self.camera_status_service().cameras_status

        cameras_status = dict(map(lambda x: (x.split(":")[0], int(x.split(":")[-1])), 
            cameras_status_str))

        return cameras_status

    def get_streaming_status(self):

        streaming_status = self.streaming_status_service()

        self.streaming_status["video_width"] = streaming_status.video_width
        self.streaming_status["video_height"] = streaming_status.video_height
        self.streaming_status["video_rate"] = streaming_status.video_rate
        self.streaming_status["video_color"] = streaming_status.video_color
        self.streaming_status["quality"] = streaming_status.quality

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

        timestamp = int(time.time()*1000) # Get current time in timestamp format
        prefix = binascii.b2a_hex(os.urandom(2)) # Get a marihunero prefix
        cam_file_names = {}

        for cam_label, img in images.items():
            file_name = '{}-{}_{}.{}'.format(prefix, timestamp, cam_label, img_format)
            cv2.imwrite(
                os.path.join(
                    self.dest_folder, "data", file_name), 
                img, [cv2.IMWRITE_JPEG_QUALITY, 
                self.quality])
            cam_file_names[cam_label] = file_name
            self.num_imgs += 1

        return timestamp, cam_file_names

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
            return totalAvailSpace/totalBytes * 100.
        else:
            return totalAvailSpace / 1024. / 1024. / 1024.

    def write_data(self, images_dict):

        if not self.ready: 
            return

        # ---------------------------------------------------------------------
        if self.video_record:
            try:
                time_ = datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")
                img = np.array(images_dict["P"])
                srt_ = "cap: {} - {}".format(self.capture_id, time_)
                img = cv2.putText(img, srt_, (180, 350), cv2.FONT_HERSHEY_SIMPLEX ,  
                    0.6, (255, 255, 255), 2, cv2.LINE_AA) 
                self.video_out.write(img)
            except Exception as e:
                self.debugger.debugger(DEBUG_LEVEL_0, 
                    "something happened recording video: {}".format(e), log_type='err')
                self.recording = False
                return

        # ---------------------------------------------------------------------
        if self.video_record: images_dict.pop("P")
        if self.write_imgs:
            try:
                timestamp, cam_file_names = self.write_images(images=images_dict)
            except Exception as e:
                self.debugger.debugger(DEBUG_LEVEL_0, 
                    "something happened saving images: {}".format(e), log_type='err')
                self.recording = False
                return
        else:
            cam_file_names = {cam_label:"NA" for cam_label in self.cameras_status.keys()}
            timestamp = int(time.time()*1000)

        # ---------------------------------------------------------------------
        if self.write_csv:
            
            sim_1_data = self.subs_wan_networks.get_wan_sim_info(id=2)
            sim_2_data = self.subs_wan_networks.get_wan_sim_info(id=3)
            rows = [
                [self.capture_id, timestamp]+
                ["cam error" if not cam_stat else cam_file_names[cam_label] for cam_label, cam_stat in self.cameras_status.items()] +
                [self.subs_odometry.roll, self.subs_odometry.pitch, self.subs_odometry.theta,
                    self.subs_actuator_control.steering, self.subs_actuator_control.throttle,
                    self.subs_wan_networks.timestamp,
                    self.subs_wan_networks.speed["upload"], self.subs_wan_networks.speed["download"],
                    self.subs_wan_networks.gps["status"], self.subs_wan_networks.gps["vdop"], 
                    self.subs_wan_networks.gps["hdop"], self.subs_wan_networks.gps["pdop"], 
                    self.subs_wan_networks.gps["timestamp"],
                    self.subs_wan_networks.gps["altitude"], 
                    self.subs_wan_networks.gps["longitude"], 
                    self.subs_wan_networks.gps["latitude"], 
                    self.subs_wan_networks.gps["speed"]] +
                    [sim_1_data["carrier"], sim_1_data["signalLevel"], 
                    sim_1_data["rssi"], sim_1_data["rsrq"], sim_1_data["rsrp"] ,sim_1_data["sinr"]] +
                    [sim_2_data["carrier"], sim_2_data["signalLevel"], 
                    sim_2_data["rssi"], sim_2_data["rsrq"], sim_2_data["rsrp"] ,sim_2_data["sinr"]] +
                    [self.subs_webclient.state_hb_latency_sec,
                     self.subs_webclient.state_hb_latency_nsec,
                     self.subs_webclient.state_pilot,
                    self.streaming_status["video_width"],
                    self.streaming_status["video_height"],
                    self.streaming_status["video_rate"],
                    self.streaming_status["video_color"],
                    self.streaming_status["quality"]]
                ]
            try: # Write data and variables in csv file
                with open(self.csv_file, 'a') as fd:
                    writer = csv.writer(fd); writer.writerows(rows)
            except Exception as e:
                self.debugger.debugger(DEBUG_LEVEL_0, 
                    "something happened saving data in csv file", log_type='err')
                self.recording = False
                return

        # ---------------------------------------------------------------------

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
            message = 'Folder for datacapture exists'
        else:
            message = 'Folder for datacapture does not exist'
            os.mkdir(self.dest_folder)

        # Path to csv file
        csv_file = os.path.join(self.dest_folder,'data.csv')

        # If csv file does not exits in destination path
        if not os.path.isfile(csv_file):
            with open(csv_file, 'a') as fd:
                writer = csv.writer(fd)
                writer.writerow(self.csv_header)

        return csv_file, message

    def get_usb_devices(self):
        sdb_devices = map(os.path.realpath, glob('/sys/block/sd*'))
        usb_idx = None
        for idx, dev in enumerate(sdb_devices):
            if 'usb' in dev: usb_idx = idx
        if usb_idx is not None:
            usb_devices = {sdb_devices[usb_idx][-1], sdb_devices[usb_idx]}
            return usb_devices
        else:
            return []

    def get_mount_points(self, devices=None):
        devices = devices or self.get_usb_devices()
        output = subprocess.check_output(['mount']).splitlines()
        is_usb = lambda path: any(dev in path for dev in devices)
        usb_info = (line for line in output if is_usb(line.split()[0]))
        return [(info.split()[0], info.split()[2]) for info in usb_info]

# =============================================================================
def setProcessName(name):
    """ sets name fot current process
    Args:
        name: `string` name of process
    Returns:
    """
    
    if sys.platform in ['linux2', 'linux']:
        import ctypes
        libc = ctypes.cdll.LoadLibrary('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    else:
        raise Exception("Can not set the process name on non-linux systems: " + 
            str(sys.platform))

# =============================================================================
def main():

    # -------------------------------------------------------------------------
    # Initialize data capture ros node
    rospy.init_node('data_capture_node', anonymous=True)
    rospy.set_param('/data_capture/debug', 0)
    setProcessName("data_capture_node")

    # create and set debugger: 0-DEBUG, 1-INFO, 2-WARNING, 3-ERROR, 4-FATAL_ERROR
    main_debugger = Debugger()

    # -------------------------------------------------------------------------
    try: # Wait for cameras status response
        main_debugger.debugger(DEBUG_LEVEL_0, "Waiting for cameras status response", 
            log_type='info')
        rospy.wait_for_service(service='video_mapping/cameras_status', timeout=20)
    except (rospy.ServiceException, rospy.ROSException), e:
        main_debugger.debugger(DEBUG_LEVEL_0, "Did not get cameras response status", 
            log_type='err')
        return 1
    main_debugger.debugger(DEBUG_LEVEL_0, "Got camera status service", 
        log_type='info')

    # -------------------------------------------------------------------------
    try: # Wait for cameras status response
        main_debugger.debugger(DEBUG_LEVEL_0, "Waiting for video streaming status response", 
            log_type='info')
        rospy.wait_for_service(service='kiwibot/video_streaming_status', timeout=20)
    except (rospy.ServiceException, rospy.ROSException), e:
        main_debugger.debugger(DEBUG_LEVEL_0, "Did not get video streaming response status", 
            log_type='err')
        return 1
    main_debugger.debugger(DEBUG_LEVEL_0, "Got video streaming status service", 
        log_type='info')

    # -------------------------------------------------------------------------
    # Create object for data capture
    BotDataCapture = DataCapture(
        debugger=main_debugger)
    # Finish and stop node is usb was not detected
    if BotDataCapture.dest_folder is None: 
        main_debugger.debugger(DEBUG_LEVEL_0, 
            "data capture node killed due to no usb", 
            log_type='err')
        time.sleep(2.0)
        return 0

    # -------------------------------------------------------------------------
    # Initialize memmap variable and wait for data
    video_map = MultiImagesMemmap(mode="r", name="main_stream", 
        memmap_path=os.getenv("MEMMAP_PATH", "/tmp"))
    video_map.wait_until_available() #initialize and find video data
    main_debugger.debugger(DEBUG_LEVEL_0, "Memmap video data ready!", 
        log_type='info')

    # -------------------------------------------------------------------------
    # Init ros node cycle
    rate = 15 #args.rate
    r = rospy.Rate(hz=int(os.environ.get("DATA_CAPTURE_RATE", 10))) # Set ros node rate
    main_debugger.debugger(DEBUG_LEVEL_0, "Ready for data capture!", log_type='info')
    while not rospy.is_shutdown():

        if BotDataCapture.recording:

            BotDataCapture.get_streaming_status()

            images_dict={cam_label:video_map.read(cam_label) for cam_label, cam_status 
                in BotDataCapture.cameras_status.items() if cam_status}
            
            if BotDataCapture.video_record: 
                images_dict["P"] = video_map.read("P")
            
            BotDataCapture.write_data(images_dict=images_dict)
            BotDataCapture.publish_status()

        r.sleep()

    if BotDataCapture.video_record:
        BotDataCapture.video_out.release()
        main_debugger.debugger(DEBUG_LEVEL_0, "forced video recording stop", 
            log_type='warn')

# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except:
        logerr_cond(True, sys.exc_info())

# =============================================================================