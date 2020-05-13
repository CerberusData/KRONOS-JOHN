#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision &Ai Team

Sources:
https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
https://unix.stackexchange.com/questions/10671/usb-performance-traffic-monitor
https://www.ximea.com/support/wiki/usb3/multiple_cameras_setup
"""

# =============================================================================
import numpy as np
import subprocess
import datetime
import yaml
import time
import cv2
import os
import re

from .vision_utils import printlog

LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 

# =============================================================================
def read_cams_configuration(CONF_PATH, FILE_NAME):
    """ Reads the camera labels, ports and other settings from file
    Args:
        CONF_PATH: `string` absolute path to configuration of cameras
        FILE_NAME: `string` name of cameras configuration file 
    Returns:
        _: `dictionary` key: camera labels, values: dictionary with camera 
            properties and settings, see yaml file for more details
    """    

    abs_path = os.path.join(CONF_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        return None

def find_cameras():
    """ Finds the camera numbers and ports that correspond to real cameras 
        Note: (devices may be repeated)
    Args:
    Returns:
        usb_ports_cameras" `list` list with video devices ports and numbers
    """

    # Finds and lists video devices numbers
    cams = find_video_devices()

    # Finds and lists video devices ports
    avail_ports = find_usb_ports(cams)

    # Zip video devices ports and numbers
    usb_ports_cameras = dict()
    for port, cam in zip(avail_ports, cams):
        if not port in usb_ports_cameras:
            usb_ports_cameras[port] = cam

    # list with video devices ports and numbers
    return usb_ports_cameras

def find_video_devices():
    """ Finds and lists video devices numbers
    Args:
    Returns:
        cams_list: `list` with video camera devices numbers
    """

    # Check for video devicesio
    p = re.compile(r".+video(?P<video>\d+)$", re.I)
    devices = subprocess.check_output("ls -l /dev", shell=True).decode('utf-8')
    avail_cameras = []

    for device in devices.split('\n'):
        if device:
            info = p.match(device)
            if info:
                dinfo = info.groupdict()
                avail_cameras.append(dinfo["video"])
    cams_list = list(sorted(map(int,avail_cameras)))

    return cams_list
 
def find_usb_ports(cameras):
    """ Finds and lists video devices ports
    Args:
        cameras: `list` with video camera devices numbers
    Returns:
        avail_ports: `list` with video camera devices ports
    """
    
    # find usb port given a video number device
    avail_ports = []
    p = re.compile(r"\d-(?P<video>[0-9.]+).+", re.I)

    for cam in cameras:
        try:
        # List of physical ports used in /dev/video# (some devices maybe represent same thing)
            path = subprocess.check_output("udevadm info --query=path --name=/dev/video" + str(cam), 
                shell=True).decode('utf-8')

        except Exception as e:
            printlog(msg="----- ERROR READING VIDEO DEVICE ----- (Error: {})".format(
                e), msg_type="ERROR")
            avail_ports.append('None-{}'.format(cam))
            continue

        printlog(msg=path.strip('\n'), msg_type="INFO")
        path = path[0:path.find("video4linux")].split('/')[-2] #get last item where the port is explicit
        info = p.match(path) #get actual address
        if info:
            dinfo = info.groupdict()
            avail_ports.append(dinfo["video"])

    return avail_ports

def decode_fourcc(v):
 # https://shimat.github.io/opencvsharp_docs/html/5e5a9f7a-b360-809c-b542-799b01ac1aa2.htm
  v = int(v)
  return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

# =============================================================================
class CameraHandler():

    def __init__(self, device_number, cam_config, cam_label): 
        """ Initializes camera  
        Args:
            device_number: `int` camera video device number
            cam_config: `dic` dictionaray with camera configuration, see yaml for 
                more details
            cam_label: `string` camera label
        Returns:
        """

        # start legacy component
        super(CameraHandler, self).__init__() 

        self.cam_device_number = device_number # Camera port number
        self.cam_device_path = "/dev/video{}".format(device_number)# Camera video device
        self.cam_config = cam_config # General camera configuration 
        self.cam_label = cam_label # Camera label

        # Video properties
        self.cap = None
        self.video_flip = self.cam_config["FLIP"] if "FLIP" in self.cam_config.keys() else 0
        self.video_height = self.cam_config["HEIGHT"][1]
        self.video_width  = self.cam_config["WIDTH"][1]
        self.video_format=None
        self.fps = 0
        
        # Camera properties        
        self.image = np.zeros((self.video_height, self.video_width, 3),
            dtype=np.uint8) # Camera Image
        self.grabbed = False # Camera video status
        self.disconnected = False # Camera connection status
        
        # Start camera port
        self.init_video_handler() 

    def init_video_handler(self):
        """ InitializeS video handler
        Args:
        Returns:
        """

        # If video device number is None then don not start video capture
        if self.cam_device_number is None:
            printlog("{}: not recognized".format(self.cam_label), msg_type="WARN")
            self.set_error_image("{} {}".format(self.cam_label, "NO RECOGNIZED"))
            self.cap = None
            return

        # Open camera port
        self.cap = cv2.VideoCapture(self.cam_device_path)
        
        # opens successfully the camera
        if self.cap.isOpened(): 

            # Set desired size to camera handler
            # Some OpenCV versions requirers assing the size twice
            self.set_properties()

            try: # Grab frame from camera and assing properties                
                self.grabbed, self.image = self.cap.read()

                if self.grabbed:

                    self.video_format = decode_fourcc(self.cap.get(
                        cv2.CAP_PROP_FOURCC))
                    printlog(msg="(GOT VIDEO) {}: DEVICE:{} - SIZE:{}X{} - FPS:{}"
                        "/{} - PROP_MODE:{} - EXPOSURE:{}".format(
                        self.cam_label, self.cam_device_number, 
                        int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), 
                        int(self.cam_config["FPS"]),
                        int(self.cap.get(cv2.CAP_PROP_FPS)), 
                        self.video_format, 
                        self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
                        ), msg_type="INFO")
                    
                    # Print warning if desired fps are not supported
                    if self.cam_config["FPS"]>float(self.cap.get(
                        cv2.CAP_PROP_FPS)):
                        printlog("{}: FRAME RATE AS {} NOT SUPPORTED, SETTED "
                            "AS {}".format(
                                self.cam_label, 
                                int(self.cam_config["FPS"]), 
                                int(self.cap.get(cv2.CAP_PROP_FPS))), 
                            msg_type="WARN")
                        self.cam_config["FPS"]=self.cap.get(cv2.CAP_PROP_FPS)

                else:
                    printlog("{}:{} did not capture first image, possibly not "
                        "space left on device".format(
                        self.cam_label, self.cam_device_number), msg_type="WARN")
                    self.set_error_image("{} NO SPACE ON DEVICE".format(
                        self.cam_label))

            # If not possible open camera port print exception 
            except Exception as e:
                printlog("Something error ocurred reading frames from camera "
                    "{} (device {}) -> {}".format(self.cam_label, 
                    self.cam_device_number, e), msg_type="ERROR")
                self.set_error_image("{} READING ERROR".format(self.cam_label))
                self.grabbed = False
                self.cap = None
        else:
            self.set_error_image("{} {}".format(self.cam_label, "CAN NOT OPEN"))
            self.grabbed = False
            self.cap = None
    
    def set_properties(self):
        """ Assigns video handler's properties
        Args:
        Returns:
        """

        # Open cameras in MJPG format requires less bandwithd 
        self.cap.set(
            cv2.CAP_PROP_FOURCC, 
            cv2.VideoWriter_fourcc(*"MJPG"))

        for cam_prop_key in self.cam_config:

            if type(self.cam_config[cam_prop_key]) is list:
                self.cap.set(
                    self.cam_config[cam_prop_key][0], 
                    self.cam_config[cam_prop_key][1])

                time.sleep(0.2)
                # Check that parameters was setted
                prop_val = self.cap.get(
                    self.cam_config[cam_prop_key][0])
                if prop_val != self.cam_config[cam_prop_key][1]:
                    if cam_prop_key == "FOURCC":
                        printlog("{} - DEVICE:{} CAN NOT SET PIXEL FORMAT AS"
                            " {}".format(self.cam_label, self.cam_device_number, 
                            decode_fourcc(self.cam_config[cam_prop_key][1])), 
                            msg_type="WARN")
                    elif cam_prop_key != "WIDTH" and cam_prop_key!="HEIGHT":
                        printlog("{} - DEVICE:{} CAN NOT SET {} AS {}".format(
                            self.cam_label, self.cam_device_number, cam_prop_key, 
                            self.cam_config[cam_prop_key][1]), msg_type="WARN")

    def set_error_image(self, error_msg):
        """ Set in camera thread image a default error image
        Args:
            error_msg: `string` error message to print on image
        Returns:
        """

        self.image = np.zeros(
            (self.video_height, self.video_width, 3), dtype=np.uint8)
        #empirical formula so that: when w=6400 -> font=1.5 and w=1920 -> font=3.5
        font_size = 0.0015625*self.video_width + 0.3 
        cv2.putText(self.image, error_msg, 
            (int(self.video_width*0.15), int(self.video_height*0.5)),
            cv2.FONT_HERSHEY_SIMPLEX, font_size, (255,255,255), 4)

    def __str__(self):

        _str="CAM:{}, VIDEO:{}, SIZE:{}X{}, FPS:{}, PROP_MODE:{}".format(
            self.cam_label, 
            self.cam_device_number,
            self.video_width, 
            self.video_height,
            self.fps,
            self.video_format)
        return _str

    def get_image(self):
        
        # If camera got image capture new frame 
        if self.grabbed and self.cap is not None: 
            
            try: # Grab a frame
                self.grabbed, cap_image = self.cap.read()
                
                if not self.grabbed: # If camera was disconnected
                    printlog("Camera: {} HOT Unplugged!".format(
                        self.cam_label), msg_type="ERROR")
                    self.set_error_image("HOT UNPLUGGED")
                    self.cap = None
                    return self.image

                if cap_image is None: 
                    raise Exception(
                        "video capture did not catch image for {} "
                        "camera".format(self.cam_label))

                # Flip the image if option is enable
                if self.video_flip != 0:
                    cv2.flip(src=cap_image, flipCode=-1)

                # Resize image
                self.image = cv2.resize(cap_image, 
                    (self.video_width, self.video_height), 
                    interpolation=cv2.INTER_LINEAR) if (
                        cap_image.shape[0]!= self.video_height or 
                        cap_image.shape[1]!= self.video_width) else cap_image

            except Exception as e: # Error reading image
                self.set_error_image("{} ERROR WHILE READING".format(
                    self.cam_label))
                printlog("Camera: {} error while reading: {}".format(
                    self.cam_label, e), msg_type="ERROR")
                self.cap = None
                return self.image

        # cv2.rectangle(self.image, (0, 0), (300, 50), (0, 0, 0), -1) 
        # current_time = datetime.datetime.now().strftime("%c")
        # cv2.putText(self.image, str(current_time), (15, 25),
        #     cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        return self.image

class CamerasCaptures():

    def __init__(self, cams_config):

        # Get available connected cameras
        self.usb_ports_cameras = find_cameras()
        if not len(self.usb_ports_cameras):
            printlog("No video devices detected", 
                msg_type="ERROR")
        elif len(self.usb_ports_cameras)/2 < len(cams_config):
            printlog("Video device numbers detected (no enough): {}".format(
                len(self.usb_ports_cameras)), msg_type="WARN")
        else:
            printlog("Video device numbers detected: {}".format(
                len(self.usb_ports_cameras)))

        # Get camera ports for usb devices
        cams_ports=[cam_dic["PORT"] for cam_dic in cams_config.values()]
        self.video_numbers = [self.usb_ports_cameras[str(port)] 
            if port in self.usb_ports_cameras 
            else None for port in cams_ports]

        # Initializes camera handler threads objects
        self.camera_handlers = dict(
            (camera_label, CameraHandler(
                device_number=device_number, 
                cam_config=cams_config[camera_label], 
                cam_label=camera_label) ) 
            for device_number, camera_label in zip(
                self.video_numbers, cams_config.keys()) )

        # Check cameras status
        self.cameras_status = ["{}:{}".format(
            cam_key, int(self.camera_handlers[cam_key].grabbed)
            ) for cam_key in self.camera_handlers.keys()]

    def get_cameras_status(self):

        return {key: True if not cam_handler.cap is None else False 
            for key, cam_handler in self.camera_handlers.items()}

# =============================================================================