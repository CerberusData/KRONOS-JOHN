#!/usr/bin/env python
# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import sys
import os

import numpy as np
import rospy
import cv2

from extended_rospylogs import Debugger
from extended_rospylogs import DEBUG_LEVEL_0
from easy_memmap import MultiImagesMemmap, EasyMemmap

from calibration.calibration import Calibrator
from local_gui.high_gui_class import high_gui

from utils.cameras import CamerasSupervisor
from utils.cameras import read_cams_configuration

LOCAL_RUN = int(os.getenv(key="LOCAL_LAUNCH", default=0)) 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from python_utils.suscribers import ChassisSuscriber
from geometry_msgs.msg import TwistStamped
import time

# =============================================================================
def setProcessName(name):
    if sys.platform in ['linux2', 'linux']:
        import ctypes
        libc = ctypes.cdll.LoadLibrary('libc.so.6')
        libc.prctl(15, name, 0, 0, 0)
    else:
        raise Exception("Can not set the process name on non-linux systems: "+str(
            sys.platform))

def show_local_gui(imgs_dic, win_name="LOCAL_VIDEO"):
    """     
        Show a local video with the current cameras, deping on the configuration
        and the camera streamings given by imgs_dic the distribution of images 
        in the window will change.
        Args:
            imgs_dic: `dictionary` dictionary of images with key as the camera 
                label and value as the image streaming of that camera
            win_name: `string` name of window to create local gui window
        Returns:
    """

    for img in imgs_dic.values(): # Draw images margins
        cv2.rectangle(img=img, pt1=(0, 0), pt2=(img.shape[1]-1, img.shape[0]-1), 
            color=(150, 150, 150), thickness=1) 

    if "C" not in imgs_dic.keys(): 
        return
    elif set(imgs_dic.keys()) == set(["C", "B", "LL", "RR", "P"]):
        stream = np.concatenate((np.concatenate((imgs_dic["C"], imgs_dic["B"]), axis=0), 
            np.concatenate((imgs_dic["LL"], imgs_dic["RR"]), axis=0)), axis=1)
        stream[int((stream.shape[0] - imgs_dic["P"].shape[0])*0.5): 
            int((stream.shape[0] - imgs_dic["P"].shape[0])*0.5) + imgs_dic["P"].shape[0],
            int((stream.shape[1] - imgs_dic["P"].shape[1])*0.5): 
            int((stream.shape[1] - imgs_dic["P"].shape[1])*0.5) + imgs_dic["P"].shape[1]] = imgs_dic["P"]
    elif set(imgs_dic.keys()) == set(["C", "LL", "RR", "P"]):
        stream = (np.concatenate((imgs_dic["LL"], imgs_dic["P"], imgs_dic["RR"]), axis=1))
    elif set(imgs_dic.keys()) == set(["C", "LL", "P"]):
        stream = (np.concatenate((imgs_dic["LL"], imgs_dic["P"]), axis=1))
    elif set(imgs_dic.keys()) == set(["C", "RR", "P"]):
            stream = (np.concatenate((imgs_dic["P"], imgs_dic["RR"]), axis=1))
    else:
        stream = imgs_dic["P"]
    cv2.imshow(win_name, stream); key = cv2.waitKey(1) # Show video and capture key
    if key==113 or key==81: # (Q) If press q then quit
        exit()
    elif key!=-1: # No key command
        print("Command or key action no found: {}".format(key))

# =============================================================================
class streaming_subs(object):
    
    def __init__(self, cam_topic, Debugger):
        """     
            Creates and manages a camera image subscriber to receive and show 
            the camera streaming image that is sent to a third party service 
            through topic message
        Args:
            cam_topic: `type` description
        Returns:
            arg_name: `type` description
        """

        self.img_bridge = CvBridge()
        self.cam_topic = cam_topic
        self._subs = rospy.Subscriber(name=cam_topic,
            data_class=Image, callback=self.img_show_callback_fn, 
            queue_size=1)
        self.cv_image = None

    def img_show_callback_fn(self, data):
        """     
            When called the variable self.cv_image will be assigned with the 
            incoming data
        Args:
            data: `sensor_msgs.msg/Image` image 
        Returns:
        """

        try:
            self.cv_image = self.img_bridge.imgmsg_to_cv2(
                img_msg=data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.debugger.debugger(DEBUG_LEVEL_0, 
                "subscriber for camera in topic {}, {}".format(
                    self.cam_topic, e), log_type="err")

class streaming_pub(object):

    def __init__(self, cams_config, Debugger, show_imgs_msg=True, queue_size=5):
        """     
            Creates and manages cameras streamings publishres to send the image 
            to a third party service using a topic message
        Args:
            cams_config: `dict` dictionary with cameras configurations
            Debugger: `Debugger` debugger to report information, warnings and errors
            show_imgs_msg: `bool` Enable/Disable the creation of subscribers to
                show images sent through camera images topic
            queue_size: `int` queue size for images topics
        Returns:
        """

        self.img_bridge = CvBridge()
        
        self.img_subscribers = {}
        self.img_publishers = {}
        self.show_img_topics = show_imgs_msg
        self.optimizer =  streaming_optimizer()

        self._FR_STREAMING_OPTIMIZER = int(os.getenv(key="FR_STREAMING_OPTIMIZER", default=1)) 
        self._VIDEO_TOPICS = int(os.getenv(key="VIDEO_TOPICS", default=1)) 
        if not self._VIDEO_TOPICS:
            return

        self._supervisors_img_topic = "/kiwibot/streaming/supervisors_img"

        self.img_publishers = {cam_key: rospy.Publisher(name=cam_params["TOPIC"], data_class=Image, 
            queue_size=queue_size) for cam_key, cam_params in cams_config.items() 
            if cam_params["TOPIC"] != "None"}
        self.img_publishers["P"] = rospy.Publisher(
            name=self._supervisors_img_topic, data_class=Image, 
            queue_size=queue_size) 
        self.debugger = Debugger

        if LOCAL_RUN and self.show_img_topics:
            self.img_subscribers= {cam_key: streaming_subs(
                    cam_topic=cam_params["TOPIC"], Debugger=Debugger) 
                for cam_key, cam_params in cams_config.items() 
                if cam_params["TOPIC"] != "None"}
            self.img_subscribers["P"] = streaming_subs(
                cam_topic=self._supervisors_img_topic, 
                Debugger=Debugger) 

    def pub(self, images_dict):
        """     
            Publishes all images in their topics
        Args:
            imgs_dic: `dictionary` dictionary of images with keys as the camera 
                labels and value as the image streaming of the camera
        Returns:
        """

        # Optimize images
        if self._FR_STREAMING_OPTIMIZER:
            images_dict = self.optimize_imgs(images_dict=images_dict)

        # Send supervisor's image message to topic
        for cam_key, _ in self.img_publishers.items():
            if cam_key not in images_dict.keys():
                continue
            try:
                img_msg = self.img_bridge.cv2_to_imgmsg(
                    cvim=images_dict[cam_key], 
                    encoding="bgr8")
                img_msg.header.stamp = rospy.Time.now()
                self.img_publishers[cam_key].publish(img_msg)
            except CvBridgeError as e:
                self.debugger.debugger(DEBUG_LEVEL_0, 
                    "publishing CAM{} image in topic, {}".format(
                    cam_key, e), log_type="err")

    def show_imgs(self):
        """     
            Shows all current images in the list of streaming_subs objects
        Args:
        Returns:
        """
        if self.show_img_topics:
            for cam_label, img_pub in self.img_subscribers.items():
                if img_pub.cv_image is not None:
                    cv2.imshow(winname="CAM{}:{}".format(
                        cam_label, img_pub.cam_topic), mat=img_pub.cv_image)

    def optimize_imgs(self, images_dict):   
        """     
            given a dictionary of images, these will be optimized to be sent to
            third party services changeing the color space and video dimensions.
        Args:
            imgs_dic: `dictionary` dictionary of images with keys as the camera 
                labels and value as the image streaming of the camera
        Returns:
        """     
        return {cam_label: self.optimizer.optimize_img(cam_img, cam_label) 
            for cam_label, cam_img in images_dict.items()}

class streaming_optimizer(object):

    def __init__(self):
        """     
            Creates video optimizer object for third party services to get the 
            parameters with images quality are reduced to be sent.
        Args:
        Returns:
        """

        self._subs_chassis = ChassisSuscriber()

        # Read local variables of video streaming configuration
        self.IDLE_TIME = int(os.environ.get("FR_STREAMING_IDLE_TIME", 20))
        self.STREAMING_FACTOR = float(os.getenv("FR_STREAMING_FACTOR", 0.2)) 
        self.STREAMING_IDLE_FACTOR = float(os.getenv("FR_STREAMING_IDLE_FACTOR", 0.4))  
        
        self.inactive_timer = 0
        self._time_tick = time.time()

        self._subs_web_control = rospy.Subscriber(
            name="/motion_control/speed_controller/reference_FR", 
            data_class=TwistStamped, callback=self._actuator_action_cb, 
            queue_size=2)

    def optimize_img(self, img, cam_label=None):
        """     
            reduces the images quality to be sent.
        Args:
            img: `cv2.math` image to reduce quality
        Returns:
        """

        if self.inactive_timer < self.IDLE_TIME + 1:
            self.inactive_timer = time.time() - self._time_tick

        if cam_label is not None:  
            hfc = 0.2 if img.shape[1] < 500 else 0.4
            org = (int(img.shape[1]*hfc), int(img.shape[0]*0.90))      
            img = cv2.putText(img=img, text="CAMERA_{}".format(cam_label), org = org, 
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, 
                color=(0, 0, 0), thickness = 4, lineType = cv2.LINE_AA)
            img = cv2.putText(img=img, text="CAMERA_{}".format(cam_label), org = org, 
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, 
                color=(0, 255, 255), thickness = 1, lineType = cv2.LINE_AA)

        if self._subs_chassis.armed:
            img = cv2.resize(src=img, dsize=(
                int(img.shape[1] * self.STREAMING_FACTOR), 
                int(img.shape[0] * self.STREAMING_FACTOR)))
        elif self.inactive_timer>=self.IDLE_TIME or not self._subs_chassis.armed:
            img = cv2.resize(src=cv2.cvtColor(src=img, code=cv2.COLOR_BGR2GRAY), 
                dsize=(int(img.shape[1] * self.STREAMING_IDLE_FACTOR), 
                       int(img.shape[0] * self.STREAMING_IDLE_FACTOR)))
            img = cv2.cvtColor(src=img, code=cv2.COLOR_GRAY2BGR)

        return img

    def _actuator_action_cb(self, data):
        """     
            Reset inactive timer when a action coming from actuator reference
            control is received
        Args:
        Returns:
        """

        self._time_tick = time.time()
    
##################################### Main ####################################
def main():

    # -------------------------------------------------------------------------
    rospy.init_node('video_mapping_node', anonymous=True) # Node initialization
    rospy.set_param('/video_mapping/debug', 0) # Set node debugger
    setProcessName("video_mapping_node") # Set video mapping process name
    main_debugger = Debugger() # Debugger that logs properly in stdout

    VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360)) 
    VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640)) 
    FR_AGENT = int(os.getenv(key="FR_AGENT", default=0)) 

    # -------------------------------------------------------------------------
    # Initiate CameraSupervisors Class that handles the threads that reads the 
    # cameras
    cams_config = read_cams_configuration(
        FILE_NAME="cams_config_local.yaml" if LOCAL_RUN else "cams_config.yaml")
    if cams_config is None : # Validate cameras status
        main_debugger.debugger(DEBUG_LEVEL_0, 
            "No cameras were configured in configuration file, video mapping node stopped", 
            log_type="err")
        rospy.spin()
        return  
    cameras_supervisor = CamerasSupervisor(cams_config=cams_config)
    
    cams_labels = cameras_supervisor.camera_handlers.keys() # Get camera labels
    cams_labels.insert(0, "P") # Add supervisors image label to camera labels
    if "C" not in cams_config: # Validate camera C
        main_debugger.debugger(DEBUG_LEVEL_0, 
            "Camera C always should be setted, video mapping node stopped", 
            log_type="err")
        rospy.spin()
        return

    # -------------------------------------------------------------------------
    # Freedom robotics variables for image message topic
    if FR_AGENT:
        imgs_publishers = streaming_pub(
            cams_config=cams_config, 
            Debugger=main_debugger)

    # -------------------------------------------------------------------------
    # Node rate is the maximum fps of cameras configuration
    rosnode_rate = max(list(map(lambda o: int(o.cam_config["FPS"]), 
        cameras_supervisor.camera_handlers.values())))
    r = rospy.Rate(hz=rosnode_rate)  # Set node rate
 
    # video mapping for raw images
    video_map = MultiImagesMemmap(memmap_path=os.getenv(key="VIDEO_MEMMAP_PATH", 
        default="/tmp"), labels=cams_labels, name="main_stream", axis=1, mode="w")
    main_debugger.debugger(DEBUG_LEVEL_0, "Cameras labels order: {}".format(
        ', '.join(cams_labels)),  log_type="info")

    robot_gui = high_gui(cam_labels=cams_labels) # object to draw gui components
    calibrator = Calibrator() # Initiate calibration class object
    
    # Start ros loop 
    main_debugger.debugger(debug_level=DEBUG_LEVEL_0, 
        msg="Initiating main loop", log_type="info")
    while not rospy.is_shutdown():
        
        # ---------------------------------------------------------------------
        # Read into a list all images read from the threads
        images_dict = dict(map(lambda o: (o.cam_label, o.image), 
            cameras_supervisor.camera_handlers.values()))
        images_dict_cp = dict(map(lambda o: (o.cam_label, o.image.copy()), 
            cameras_supervisor.camera_handlers.values()))

        # ---------------------------------------------------------------------
        # Check image dimensions for gui issues        
        for cam_label in images_dict.keys():
            if (images_dict[cam_label].shape[0] != VIDEO_HEIGHT
                or images_dict[cam_label].shape[1] != VIDEO_WIDTH):
                images_dict[cam_label]= cv2.resize(src=images_dict[cam_label], 
                    dsize=(VIDEO_WIDTH, VIDEO_HEIGHT))

        # ---------------------------------------------------------------------
        # GUI COMPONENTS - GUI COMPONENTS - GUI COMPONENTS - GUI COMPONENTS - G
        images_dict["P"] = robot_gui.draw_components(imgs_dict=images_dict)
        images_dict_cp["P"] = images_dict["P"] 

        # ---------------------------------------------------------------------
        # Publicate images topics
        if "imgs_publishers" in locals():
            imgs_publishers.pub(images_dict=images_dict_cp)
            if LOCAL_RUN: imgs_publishers.show_imgs()
                
        # ---------------------------------------------------------------------
        # CALIBRATION - CALIBRATION - CALIBRATION - CALIBRATION - CALIBRATION -
        
        # after calibration, show only resulting image for a couple of seconds
        if calibrator.show_calibration_result:
            images_dict["P"] = calibrator.calibration_image.copy()

        # Concatenate all list images in one big 3D matrix and map them in memory
        video_map.write(np.concatenate(images_dict.values(), axis=1))
        r.sleep() # Suspend execution of R expressions for time interval

        # ---------------------------------------------------------------------
        # Visual and local debuggin - Visual and local debuggin - Visual and lo 
        if LOCAL_RUN:
            show_local_gui(imgs_dic=images_dict, win_name="LOCAL_VIDEO")
 
# =============================================================================
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# =============================================================================