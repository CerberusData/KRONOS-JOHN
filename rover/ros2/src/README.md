# Video mapping Node

## Short Description

This node is in charge of: 
    
    John Alberto Betancourt Gonzalez 
    john@kiwicampus.com 

For diagrams and more node, topics, services, parameters and information go to:
- [Environment variables: KIWIBOT_APOLLO/configs/local_env_vars.sh](https://drive.google.com/open?id=1QrTXTuZKveOrdQR2NBoVC5LDX_zD9jKv)
- [Documentation of environment variables](https://www.notion.so/kiwi/Defining-Environment-Variables-729fdef098a44b4e8e4c7cfef9a47abe)
- [Control Robot's architecture](https://drive.google.com/open?id=1bcWFONAZZEr0n4XVKkfeyaNNOUsVbfTm)
- [Video issues and calibration](https://drive.google.com/open?id=1Jx6hgiNxutLpTTFu-5XRtsBfVMoP4D84WgqcYMMZMJw) 
- [Vision system architecture](https://drive.google.com/open?id=1tBCa44rpLfaVWX-ZdUtJhPdU60rea3jl)
- [Ai development and tutorials](https://drive.google.com/open?id=12Iz5gZrEemtlcY20pNstr67NzYA0xxen)

## Node Responsibilities

This node is shown as **video_mapping**, and is in charge of:

* Opening and reading the video streaming of cameras connected through a USB hub. Also handling errors of video streaming readings.
* Publish the streaming images through topics for third party services.
* Modifying the video stream (or images) that are going to be streamed with the functionalities shown in the supervisor's console video:
    * Overlaying different cameras images: rear camera in the top right and left on superior corner, zoomed image in the bottom left corner, virtual pan with left and right cameras, stitching mode.
    * Drawing insightful information: object detection bounding boxes, the clickable region for waypoint-mode, waypoint trajectory, debug messages, warning labels, messages, and other GUI objects.
    * Calibration routines for the cameras (RR, C, LL).


## Node Functionality Description

### *Local Mode*

If you are running the stack of the robot in your computer, you can activate the local GUI using the environment variable **LOCAL_LAUNCH**, don't forget to check the file `./configs/local_env_vars.sh` for more information about.

### *Cameras Initialization and Threads*

The cameras threads are executed in the class **cameras_supervisor** in the package `vision.utils.cam_handler`, which receives a dictionary with all cameras configuration parameters, you can check these settings in the file `cams_config.yaml`. The file is well explained by itself, so no more explanations here. You can change, comment, remove or add parameters supported by OpenCV and FFmpeg for each camera, but the most interesting thing about this topic is that you can start a camera with different video size and frame rate from others, also don't launch a specific camera if you want. For third party services, the node creates publishers using the class **streaming_pub** to share the cameras streaming through topics, also defined in `cams_config.yaml`. The state of cameras is reported by the service `video_mapping/cameras_status` all the time.

Remember the configuration and algorithms are dynamic when start, I mean if you remove a camera in the file `cams_config.yaml`, the topics, memory, process, classes, and others for that camera won't be created or considered in the code execution. 

The cameras images in video mapping are taken from the **cameras_supervisor** class as a dictionary where the keys are the labels of the cameras, for example, **C** for central camera, **RR** for right camera, **LL** for left camera, **B** for rear camera, and **P** for the supervisors image which is also stored in this dictionary, but is not a raw camera data, it will be the image with all components draw and the data to be streamed.  

### *Drawing Everything on Supervisors Console Image*

The drawing logic is huge, due that we don't have frontend engineers or a console to receive data fast enough, we decided to draw directly in the supervisor's image the most relevant information to be notified to supervisors (also is less laggy). The **high_gui** class comes to save us from disorder, the code is well structured in that class, in the main code of the **video_mapping_node**, you will see only a call of the function *draw_components()* of the object **high_gui** class. Here the logic to draw a component or not is performed with subscribers and environment variables. Add, remove or change GUI components is quite easy. The call of these functions will return the supervisor's console image, and the next step is to stream it.

### *When a Camera is Calibrated*

Calibration is a huge topic by itself, you can check the file [Video issues and calibration](https://drive.google.com/open?id=1Jx6hgiNxutLpTTFu-5XRtsBfVMoP4D84WgqcYMMZMJw) for more information, no more explanation here. When a camera is calibrated we want to see the results, so video mapping has an object of the class **Calibrator**, this class is also huge, and requires another document to explain what it does, but the only thing you have to know here is when a camera is calibrated a variable of this class changes for some seconds to indicate that the calibration was performed successfully, in this case, the supervisors image **P** will take the result of the calibrator class and the image will be shown on supervisors console.




## Topics/Services/Parameters/Environment_variables

This section describes the topics, services, parameters and environment variables that are used by the node, and its classes, modules, and other scripts associated.

**WARNING**: All subscribers couldn't be listed here, if you see one, which is not here, please add it and make a pull request, thank you!

### Subscribers

The node uses many classes to keep the main node code simple and clean, giving the right responsibilities to each python module. The node is declared in `video_mapping_node.py`, and the most important Python modules are:

<!-- * Calibrator: handles the calibration processes and results
* high_gui: handle printings and drawings with the video cameras streaming, creates the final image which is streamed into the supervisor's console.

the *high_gui* object use many classes of the module called *python_util/subscribers* which is shared with other nodes to get information about other topics in the stack of the robot. The following subscribers are used by this class: -->

<!-- * *ActuatorReferenceSuscriber*
    - (subscriber) - **/kiwibot/actuator_reference** [`geometry_msgs/TwistStamped`]: Information of steering reference  -->

### Publishers     

The node publishes to the following topics:

* *Video Mapping Node* (class *streaming_pub*):
    * /streaming/cam_central [`sensor_msgs/Image`]
    * /streaming/cam_rear [`sensor_msgs/Image`]
    * /streaming/cam_left [`sensor_msgs/Image`] 
    * /streaming/cam_right [`sensor_msgs/Image`]

    the name of topics is specified in the file `cams_config.yaml`. the convention needs sense, so don't change it. If a camera is omitted, removed or added, the respective topic will be created or not. These topics are used to send the images to third parties agents, programs or scripts associated with the ROS master core.
    
### Services

The node exposes the following services:

- (subscriber) - */video_mapping/cams_status* [`usr_srv/srv/CamerasStatus`]: It checks the status of each of the camera connected to the robot. Doesn't receive any input param. Returns an array of String values with the notation "CAMLABEL:STATE" indicating whether a camera is working or not. CAMLABEL is the camera label keeping the notation explained before and STATE is 0 is something is wrong with the camera (disconnected, reading error, no space left on device) or if it's working properly.

### Parameters

This node doesn't use any specific parameters

## Technical/Implementation details

### Threads usage

The structure of the node is the following:

The main process spawns different threads (one per camera) to read continuously the cameras stream. These threads are in the same context of the parent process, so this one can access their variables, i.e. the variables where the images are stored of the camera are stored. By default, each thread also flips the image read because the cameras are physically upside down (Trolls!).
