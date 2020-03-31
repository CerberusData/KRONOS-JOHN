# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision &Ai Team

SOurces:
https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
https://unix.stackexchange.com/questions/10671/usb-performance-traffic-monitor
https://www.ximea.com/support/wiki/usb3/multiple_cameras_setup
"""

# =============================================================================
import numpy as np
import time
import yaml
import os

import subprocess
import rospy
import cv2
import re

from threading import Thread, Event

from extended_rospylogs import Debugger, update_debuggers, loginfo_cond, logerr_cond
from extended_rospylogs import DEBUG_LEVEL_0, DEBUG_LEVEL_1, DEBUG_LEVEL_2, DEBUG_LEVEL_3, DEBUG_LEVEL_4

from video_mapping.srv import CamerasStatus
from video_mapping.srv import CamerasStatusResponse

LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 

# =============================================================================
def read_cams_configuration(FILE_PATH=os.path.dirname(os.path.abspath(__file__)), 
    FILE_NAME="cams_config.yaml"):
    """ Reads the camera labels and port from file
    Args:
        FILE_PATH: `string` absolute path to configuration of cameras
        FILE_NAME: `string` name of cameras configuration file 
    Returns:
        _: `dictionary` key: camera labels, values: dictionary with camera 
            properties, see yaml for more details
    """    
    abs_path = os.path.join(FILE_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
    else:
        cam_config_debug = Debugger()
        cam_config_debug.debugger(DEBUG_LEVEL_0, 
            "Camera configuration file '{}' not found".format(
            FILE_NAME), log_type="err")
        return None

def find_cameras(debugger):
    """ Finds the camera numbers and ports that correspond to real cameras 
        Note: (devices may be repeated)
    Args:
    Returns:
        usb_ports_cameras" `list` list with video devices ports and numbers
    """

    # Finds and lists video devices numbers
    cams = find_video_devices()

    # Finds and lists video devices ports
    avail_ports = find_usb_ports(debugger, cams)

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
 
def find_usb_ports(debugger, cameras):
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
            path = subprocess.check_output("udevadm info --query=path --name=/dev/video" + str(cam), shell=True).decode('utf-8')

        except Exception as e:
            debugger(DEBUG_LEVEL_0, "----- ERROR READING VIDEO DEVICE ----- (Error: {})".format(e), log_type = 'err')
            avail_ports.append('None-{}'.format(cam))
            continue

        debugger(DEBUG_LEVEL_0, path.strip('\n'))
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
class CameraHandler(Thread, Debugger):

    def __init__(self, device_number, cam_config, cam_label): 
        """ Initializes camera Handler thread 
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
        self.video_handler = None
        self.video_width  = self.cam_config["WIDTH"][1]
        self.video_height = self.cam_config["HEIGHT"][1]
        self.video_flip = self.cam_config["FLIP"] if "FLIP" in self.cam_config.keys() else 0
        self.fps = 0
        self.video_format=None
        # [int][sec] time to keep loking for a camera disconnected
        self.video_idle=int(os.getenv(key="VIDEO_IDLE_TIME", default=120)) 

        # Camera properties        
        self.image = np.zeros((self.video_height, self.video_width, 3),
            dtype=np.uint8) # Camera Image
        self.grabbed = False # Camera video status
        self.disconnected = False # Camera connection status
        
        # Thread variables
        self.run_event = Event()
        self.run_event.set()
        self.daemon = True
        
        # Start camera port
        self.init_video_handler() 

    def init_video_handler(self):
        """ InitializeS video handler
        Args:
        Returns:
        """

        # Open camera port
        self.video_handler = cv2.VideoCapture(self.cam_device_path)

        # opens successfully the camera
        if self.video_handler.isOpened(): 

            # Set desired size to camera handler
            # Some OpenCV versions requirers assing the size twice -don't know why
            self.set_properties()

            try: # Grab frame from camera and assing properties                
                self.grabbed, self.image = self.video_handler.read()

                if self.grabbed:
                    self.video_format = decode_fourcc(self.video_handler.get(cv2.CAP_PROP_FOURCC))
                    self.debugger(DEBUG_LEVEL_0, "(GOT VIDEO) {}: DEVICE:{} - SIZE:{}X{} - FPS:{}/{} - PROP_MODE:{} - EXPOSURE:{}".format(
                        self.cam_label, self.cam_device_number, 
                        int(self.video_handler.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        int(self.video_handler.get(cv2.CAP_PROP_FRAME_HEIGHT)), 
                        int(self.cam_config["FPS"]),
                        int(self.video_handler.get(cv2.CAP_PROP_FPS)), 
                        self.video_format, 
                        self.video_handler.get(cv2.CAP_PROP_AUTO_EXPOSURE)
                        ), log_type="info")
                    
                    # If camera was disconnected
                    if self.disconnected:
                        self.disconnected=False 
                        self.debugger(DEBUG_LEVEL_0, "{}:{} has been re-connected".format(
                            self.cam_label, self.cam_device_number), log_type="info")

                    # # Print info if video size can not be setted
                    # if self.image.shape[0]!=self.video_height or self.image.shape[1]!=self.video_width:
                    #     self.image=cv2.resize(self.image, (self.video_width, self.video_height), interpolation=cv2.INTER_LINEAR)
                    #     self.debugger(DEBUG_LEVEL_0, "{}: CANT NOT SET VIDEO SIZE, DEFAULT IS {}x{}, IMAGE WILL BE RESIZED TO {}x{}".format(
                    #         self.cam_label, 
                    #         int(self.video_handler.get(cv2.CAP_PROP_FRAME_WIDTH)), 
                    #         int(self.video_handler.get(cv2.CAP_PROP_FRAME_HEIGHT)), 
                    #         self.video_width,
                    #         self.video_height),
                    #         log_type="warn")
                            
                    # Print warning if desired fps are not supported
                    if self.cam_config["FPS"]>float(self.video_handler.get(cv2.CAP_PROP_FPS)):
                        self.debugger(DEBUG_LEVEL_0, "{}: FRAME RATE AS {} NOT SUPPORTED, SETTED AS {}".format(
                            self.cam_label, int(self.cam_config["FPS"]), int(self.video_handler.get(cv2.CAP_PROP_FPS))),
                            log_type="warn")
                        self.cam_config["FPS"]=self.video_handler.get(cv2.CAP_PROP_FPS)

                else:
                    self.debugger(DEBUG_LEVEL_0, "{}:{} did not capture first image, possibly not space left on device".format(
                        self.cam_label, self.cam_device_number), log_type="warn")
                    self.set_error_image("{} NO SPACE ON DEVICE".format(self.cam_label))

            # If not possible open camera port print exception 
            except Exception as e:
                self.debugger(DEBUG_LEVEL_0,
                    "Something error ocurred reading frames from camera {} (device {}) -> {}".format(
                        self.cam_label, self.cam_device_number, e), log_type = 'err')
                self.grabbed = False
                self.set_error_image("{} READING ERROR".format(self.cam_label))
        else:
            if not self.disconnected:
                self.debugger(DEBUG_LEVEL_0, "{}: {} not recognized".format(
                    self.cam_label, self.cam_device_number), log_type="warn")
            self.set_error_image("{} {}".format(self.cam_label, "NO RECOGNIZED" if not self.disconnected else "DISCONNECTED"))
            self.video_handler = None
    
    def set_properties(self):
        """ Assigns video handler's properties
        Args:
        Returns:
        """
        for cam_prop_key in self.cam_config:
            if type(self.cam_config[cam_prop_key]) is list:
                self.video_handler.set(self.cam_config[cam_prop_key][0], self.cam_config[cam_prop_key][1])
                prop_val = self.video_handler.get(self.cam_config[cam_prop_key][0])
                if prop_val!= self.cam_config[cam_prop_key][1]:
                    if cam_prop_key=="FOURCC":
                        self.debugger(DEBUG_LEVEL_0, "{} - DEVICE:{} CAN NOT SET PIXEL FORMAT AS {}".format(
                            self.cam_label, self.cam_device_number, decode_fourcc(self.cam_config[cam_prop_key][1])), log_type="warn")
                    elif cam_prop_key!="WIDTH" and cam_prop_key!="HEIGHT":
                        self.debugger(DEBUG_LEVEL_0, "{} - DEVICE:{} CAN NOT SET {} AS {}".format(
                            self.cam_label, self.cam_device_number, cam_prop_key, 
                            self.cam_config[cam_prop_key][1]), log_type="warn")

    def set_error_image(self, error_msg):
        """ Set in camera thread image a default error image
        Args:
            error_msg: `string` error message to print on image
        Returns:
        """
        self.image = np.zeros((self.video_height, self.video_width, 3), dtype=np.uint8)
        font_size = 0.0015625*self.video_width + 0.5 #empirical formula so that: when w=6400 -> font=1.5 and w=1920 -> font=3.5
        cv2.putText(self.image, error_msg, (self.video_width//8, self.video_height//2),
            cv2.FONT_HERSHEY_SIMPLEX, font_size, (255,255,255), 4)

    def __str__(self):

        _str="CAM:{},VIDEO:{},SIZE:{}X{},FPS:{},PROP_MODE:{}".format(
            self.cam_label, 
            self.cam_device_number,
            self.video_width, 
            self.video_height,
            self.fps,
            self.video_format)
        return _str

    def run(self):
        tries_limit = 0 # NUmber of tries if a camera is disconnected
        time_prev = 0.; time_wait=0.; time_elapsed=0.; # Times to capture frame
        desired_fps = float(self.cam_config["FPS"]); time_fps=float(1./desired_fps)
        fps_cnt = 0; fps_tic=time.time() # FPS count variable
        
        if self.video_handler is not None: # If camera was openned 
            while self.run_event.is_set(): # Start thread loop
                
                if self.grabbed: # If camera got image capture new frame 
                    try: # Grab a frame
                        self.grabbed, image = self.video_handler.read()
    
                        # Flip the image if option is enable
                        if self.video_flip is not None:
                            image=cv2.flip(src=image, flipCode=-1)

                        # Resize image
                        self.image=image
                        # self.image=cv2.resize(image, (self.video_width, self.video_height), interpolation=cv2.INTER_LINEAR
                        #         ) if image.shape[0]!=self.video_height or image.shape[1]!=self.video_width else image
                            
                        if LOCAL_RUN: # DEBUG - DEBUG - DEBUG - DEBUG - DEBUG -
                            cv2.putText(image, "{}:{}".format(self.cam_label, self.fps), 
                                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, 3)
                            cv2.putText(image, "{}:{}".format(self.cam_label, self.fps), 
                                (10, image.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, 3)
                            cv2.putText(image, "{}:{}".format(self.cam_label, self.fps), 
                                (image.shape[1]-90, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, 3)
                            cv2.putText(image, "{}:{}".format(self.cam_label, self.fps), 
                                (image.shape[1]-90, image.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, 3)

                    except Exception as e: # Error reading image
                        self.set_error_image("{} ERROR WHILE READING".format(self.cam_label))
                        self.debugger(DEBUG_LEVEL_0, "Camera: {} error while reading".format(
                            self.cam_label), log_type = 'err')
                        self.disconnected = True
                        self.fps = 0

                    if not self.grabbed: # If camera was disconnected
                        self.debugger(DEBUG_LEVEL_0, "Camera: {} HOT Unplugged!".format(
                            self.cam_label), log_type = 'err')
                        self.video_handler.release()
                        self.video_handler = None
                        self.disconnected = True
                        self.fps = 0   

                elif self.disconnected: # If camera was disconnected 
                    self.init_video_handler()
                    time.sleep(1) # wait as if a camera is disconnected
                    # find_cameras(self.debugger)
                    tries_limit+=1
                    self.set_error_image("RECONNECTING")
                    if tries_limit>self.video_idle: # in [seconds]
                        self.debugger(DEBUG_LEVEL_0, "Camera: {} has reached the limit of reconnecting trials".format(
                            self.cam_label), log_type = 'warn')
                        self.set_error_image("{} LOST".format(self.cam_label))
                        self.fps = 0
                        self.set_error_image("LOST")
                        break
                        
                # wait as if a frame were read
                time_elapsed = float(time.time() - time_prev)
                time_wait = float(time_fps-time_elapsed)

                # suspends execution of the current thread in seconds.
                if time_wait>=0.0: 
                    time.sleep(time_wait)
                time_prev = time.time()

                # Count FPS
                fps_cnt = fps_cnt + 1
                fps_elap = time.time() - fps_tic
                if fps_elap > 1.0:
                    self.fps = fps_cnt
                    fps_tic = time.time()
                    fps_cnt = 0

            if self.video_handler is not None:
                self.video_handler.release() # Release video variable
                self.video_handler = None # Make handler thread process as None

class CamerasSupervisorBase(Debugger):

    def __init__(self, cams_config, CameraClass=CameraHandler):

        # start legacy components
        super(CamerasSupervisorBase, self).__init__()
        
        # Get available connected cameras
        self.usb_ports_cameras = find_cameras(self.debugger)

        # Print some info
        self.debugger(DEBUG_LEVEL_0, "Video device numbers detected: {}".format(
            len(self.usb_ports_cameras)), log_type="warn" if not len(self.usb_ports_cameras) else "info")
        
        cams_ports=[cam_dic["PORT"] for cam_dic in cams_config.values()]
        self.video_numbers = [self.usb_ports_cameras[str(port)] if port in self.usb_ports_cameras 
            else None for port in cams_ports]

        # Initializes camera handler threads objects
        self.camera_handlers = dict(
            (camera_label, CameraClass(device_number=device_number, cam_config=cams_config[camera_label], cam_label=camera_label) ) 
            for device_number, camera_label in zip(self.video_numbers, cams_config.keys()) )

        # Check cameras status
        self.cameras_status = ["{}:{}".format(cam_key, int(self.camera_handlers[cam_key].grabbed)
            ) for cam_key in self.camera_handlers.keys()]

        # Create services to report cameras status
        rospy.Service('video_mapping/cameras_status', CamerasStatus, 
            self.get_cameras_status_verbose)
        
    def get_cameras_status_verbose(self, data):
        return CamerasStatusResponse(self.cameras_status)

class CamerasSupervisor(CamerasSupervisorBase):

    def __init__(self, cams_config):

        # Create handlers for each specified camera
        super(CamerasSupervisor, self).__init__(cams_config=cams_config)

        # Start video capture of handlers
        map(lambda o: o.start(), self.camera_handlers.values())

# =============================================================================