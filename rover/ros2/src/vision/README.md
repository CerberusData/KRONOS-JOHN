# Video mapping Node

## Short Description

This node is in charge of: 
    
    John Alberto Betancourt Gonzalez 
    john@kiwicampus.com 
    +57 350 283 5122 

For diagrams and more node, topics, services, parameters and information go to:
- [Environment variables: KIWIBOT_APOLLO/configs/local_env_vars.sh](https://drive.google.com/open?id=1QrTXTuZKveOrdQR2NBoVC5LDX_zD9jKv)
- [Documentation of environment variables](https://www.notion.so/kiwi/Defining-Environment-Variables-729fdef098a44b4e8e4c7cfef9a47abe)
- [Control Robot's architecture](https://drive.google.com/open?id=1bcWFONAZZEr0n4XVKkfeyaNNOUsVbfTm)
- [Video issues and calibration](https://drive.google.com/open?id=1Jx6hgiNxutLpTTFu-5XRtsBfVMoP4D84WgqcYMMZMJw) 
- [Vision system architecture](https://drive.google.com/open?id=1tBCa44rpLfaVWX-ZdUtJhPdU60rea3jl)
- [Ai development and tutorials](https://drive.google.com/open?id=12Iz5gZrEemtlcY20pNstr67NzYA0xxen)

## Node Responsibilities

This node is shown as **video_mapping_node**, and is in charge of:

* Opening and reading the video streaming of cameras connected through the USB hub. Also handling errors of video streaming readings.
* Using the instant images from the video streams to map them into memory as a huge 3D array (see detailed description for this).
* Publish the streaming images through topics for third party services.
* Modifying the video stream (or images) that are going to be streamed with the functionalities shown in the supervisor's console video:
    * Overlaying different cameras images: rear camera in the top right and left on superior corner, zoomed image in the bottom left corner, virtual pan with left and right cameras, stitching mode.
    * Drawing insightful information: object detection bounding boxes, the clickable region for waypoint-mode, waypoint trajectory, debug messages, warning labels, messages, and other GUI objects.
    * Calibration routines for the cameras (RR, C, LL).


## Node Functionality Description

### *Local Mode*

If you are running the stack of the robot in your computer, you can activate the local GUI using the environment variable **LOCAL_LAUNCH**, don't forget to check the file `KIWIBOT_APOLLO/configs/local_env_vars.sh` for more information.

### *Cameras Initialization and Threads*

The node starts reading the video size from the environment variables **VIDEO_HEIGHT** and **VIDEO_WIDTH** , to check that all images mapped in memory have the same size, otherwise, images will be resized because you can configure each camera with a different configuration, but to mapping them in memory with **MultiImagesMemmap**  a fixed array size is required (this video size is the size of the image to be streamed and used by other nodes, others like third party services will get the images with their original size values). 

The cameras threads are executed in the class **cameras_supervisor**, which receives a dictionary with all cameras configuration parameters, you can check these settings in the file `cams_config.yaml`. The file is well explained by itself, so no more explanations here. You can change, comment, remove or add parameters supported by OpenCV and FFmpeg for each camera, but the most interesting thing about this topic is that you can start a camera with different video size and frame rate from others, also don't launch a specific camera if you want. For third party services, the node creates publishers using the class **streaming_pub** to share the cameras streaming though topics, also defined in `cams_config.yaml`. The state of cameras is reported by the service `video_mapping/cameras_status`.

Remember the configuration and algorithms are dynamic when start, I mean if you remove a camera in the file `cams_config.yaml`, the topics, memory, process, classes, and others for that camera won't be created or considered in the code execution. 

The cameras images in video mapping are taken from the **cameras_supervisor** class as a dictionary where the keys are the labels of the cameras, for example, **C** for central camera, **RR** for right camera, **LL** for left camera, **B** for rear camera, and **P** for the supervisors image which is also stored in this dictionary, but is not a raw camera data, it will be the image with all components draw and the data to be streamed.  

### *Drawing Everything on Supervisors Console Image*

The drawing logic is huge, due that we don't have frontend engineers or a console to receive data fast enough, we decided to draw directly in the supervisor's image the most relevant information to be notified to supervisors. The **high_gui** class comes to save us from disorder, the code is well structured in that class, in the main code of the **video_mapping_node**, you will see only a call of the function *draw_components()* of the object **high_gui** class. Here the logic to draw a component or not is performed with subscribers and environment variables. Add, remove or change GUI components is quite easy. The call of these functions will return the supervisor's console image, and the next step is to stream it.

### *When a Camera is Calibrated*

Calibration is a huge topic by itself, you can check the file [Video issues and calibration](https://drive.google.com/open?id=1Jx6hgiNxutLpTTFu-5XRtsBfVMoP4D84WgqcYMMZMJw) for more information, no more explanation here. When a camera is calibrated we want to see the results, so video mapping has an object of the class **Calibrator**, this class is also huge, and requires another document to explain what it does, but the only thing you have to know here is when a camera is calibrated a variable of this class changes for some seconds to indicate that the calibration was performed successfully, in this case, the supervisors image **P** will take the result of the calibrator class and the image will be shown on supervisors console.

### *Streaming the Image*

This node is not in charge of the stream of the video to any web service, but here comes a short explanation of how it works. The package **vision** has the node to optimize the quality and features of the video to be streamed. Depending on the robots state `webrtc_optimizer_node.py` creates a configuration to stream the video, one for an operative state and other when the robot is in standby. We use a node because the script who is in charge of video streaming is in Java and in the moment an integration with ROS looked quite hard to code, so the video configuration is sent with a JSON file through an endpoint, and the image is given by the `websocketserver.py` script to the **webrtc** script using a WebSocket.


## Topics/Services/Parameters/Environment_variables

This section describes the topics, services, parameters and environment variables that are used by the node, and its classes, modules, and other scripts associated.

**WARNING**: All subscribers couldn't be listed here, if you see one, which is not here, please add it and make a pull request, thank you!

### Subscribers

The node uses many classes to keep the main node code simple and clean, giving the right responsibilities to each python module. The node is declared in `video_mapping_node.py`, and the most important Python modules are:

* EasyMemmap: Map video streaming images to memory for fast reading in other packages or nodes.
* Calibrator: handles the calibration processes and results
* high_gui: handle printings and drawings with the video cameras streaming, creates the final image which is streamed into the supervisor's console.

the *high_gui* object use many classes of the module called *python_util/subscribers* which is shared with other nodes to get information about other topics in the stack of the robot. The following subscribers are used by this class:

* *ActuatorReferenceSuscriber*
    - (subscriber) - **/kiwibot/actuator_reference** [`geometry_msgs/TwistStamped`]: Information of steering reference 

* *ActuatorControlSuscriber*
    - (subscriber) - **/kiwibot/actuator_control** [`geometry_msgs/TwistStamped`]: Information of steering and throttle reference 

* *DataCaptureSuscriber*
    - (subscriber) - **/data_capture/status** [`data_capture/Status`] as DataCaptureStatus: Information if data capture node is recording data or not, how much space left in USB device, and the number of images recorded.

* *WanNetworksSuscriber*
    - (subscriber) - **/pepwave/wan_status** [`pepwave/WanStatusArray`]: Description and information of wan connection
    - (subscriber) - **/pepwave/wan_speed** [`pepwave/NetSpeed`]: Description and information about download and upload speeds.
    - (subscriber) - **/pepwave/gps_location** [`pepwave/GpsLocation`]: Description, information, and status of router's GPS.

* *NavigationSuscriber*
    - (subscriber) - **/position_control/error** [*geometry_msgs/PoseStamped`]: Position control error/references to move the robot.

* *WebClientSuscriber*
    - (subscriber) - **web_client/control** [`web_client/WebControl`]: Information of webclient's pan, throttle, and tilt.
    - (subscriber) - **web_client/control** [`web_client/WebStatus`]: Information of webclient's pilot state, and latency. 

* *WaypointSuscriber*
    - (subscriber) - **/vision/video_position** [`geometry_msgs/PointStamped`]: Get waypoint clic XY position in video 
    - (subscriber) - **/position_control/error** [`geometry_msgs/PoseStamped`]: Get Position control error/references to move the robot.
    - (publishers) - **/desired_pose** [`geometry_msgs/PoseStamped`]: Reports the desired pose of robot which is the position error
    - (publishers) - **/video_mapping/message** [`web_client/Messages`]: Reports message to supervisors console

* *OdometrySuscriber*
    - (subscriber) - **/kiwibot/odometry/estimation** [`odometry/EncoderOdom`]: Odometry description and information of robot, roll , pitch, and yaw

* *KiwiBotSuscriber*
    - (subscriber) - **/web_client/configure** [`std_msgs/Bool`]: Signal to update kiwibot parameters
    - (subscriber) - **/rover_teleop/driveMode** [`std_msgs/String`]: Get robot's driving mode
    - (subscriber) - **/video_mapping/zoom** [*std_msgs/Bool*]: Signal to activate/deactivate zoom in image.
    - (subscriber) - **/skynet/autopilot** [`std_msgs/Bool`]: Signal to activate/deactivate autopilot node.
    - (subscriber) - **/video_mapping/stitch** [`std_msgs/Bool`]: Signal to activate/deactivate stitched image.

* *BatterySuscriber*
    - (subscriber) - **/kiwibot/sensors/battery** [`sensor_msgs/BatteryState`]: Get robot's battery voltage level.

* *import ChassisSuscriber*
    - (subscriber) - **/kiwibot/motor/status** [`socket_can_ros/Motors`]: Get  state of each robot's motors.
    - (subscriber) - **/kiwibot/driveController/status** [`mavros_msgs/State`]: Get driver controller status, if is working/connected or having some error.

* *import CameraSuscriber*
    - (subscriber) - **/video_mapping/rear_camera** [`std_msgs/Bool`]: Get activation state of rear camera to be shown on supervisors image or not.
    - (subscriber) - **/video_mapping/switch_rear_camera** [`std_msgs/Bool`]: Get activation state to switch between frontal and rear camera on supervisors image or not.

* *import PWMoutSuscriber*
    - (subscriber) - **/kiwibot/pwm/out** [`socket_can_ros/PwmOut`]: Receives the signal when an action (open/close) in the robot's door is executed. 

* *SkynetSuscriber*
    - (subscriber) - **/skynet/config** [`skynet/Status`]: Get object detection status, if is activated or not.
    - (subscriber) - **/skynet/object_detection** [`std_msgs/String`]: Get the detection by the 

* *VisualDebugger*
    - (subscriber) - **/video_mapping/message** [`web_client/Messages`]: get log messages when something fails or there's something to report in the supervisors console image.
    - (subscriber) - **/web_client/message** [`web_client/Messages`]: get log messages when something fails or there's something to report from the web_client (previously it also showed something in supervisors console)

* *CliffSensorSuscriber*
    - (subscriber) - **/kiwibot/cliff_sensorx** [`sensor_msgs/Range`]: get the value of the cliff sensor.

* *DistanceSensorSuscriber*
    - (subscriber) - **/kiwibot/distance_sensor?** [`sensor_msgs/Range`]: get the value of the distance sensor. 

* `video_mapping/calibration/calibration.py`
    - (subscriber) - **video_mapping/calibrate** [`std_msgs/String`]: signal to activate process of camera calibration, to calibrate a specific camera the label should be given as a string argument.
    - (publisher) - **video_mapping/calibrate/done** [`std_msgs/Bool`]: signal when the calibration was performed successfully.

### Publishers     

The node publishes to the following topics:

* *VisualDebugger*
    - (Publisher) -  **/web_client/message** [`web_client/Messages`]: Publishes log messages when something fails to web_client (previously it also showed something in pilots console)

* *WaypointSuscriber*
    - (Publisher) - **/desired_pose** [`geometry_msgs/PoseStamped`]: description

* *Video Mapping Node* (class *streaming_pub*):
    * /kiwibot/streaming/supervisors_img [`sensor_msgs/Image`]
    * /kiwibot/streaming/left_camera [`sensor_msgs/Image`]
    * /kiwibot/streaming/right_camera [`sensor_msgs/Image`] 
    * /kiwibot/streaming/back_camera [`sensor_msgs/Image`]

    the name of topics is specified in the file `cams_config.yaml`. the convention needs sense, so don't change it. If a camera is omitted, removed or added, the respective topic will be created or not. These topics are used to send the images to third parties agents, programs or scripts associated with the ROS master core.
    
### Services

The node exposes the following services:

* `video_mapping/utils/calibration.py`
    - (subscriber) - */video_mapping/get_cameras_status* [`video_mapping/CamerasStatus`]: It checks the status of each of the camera connected to the robot. Doesn't receive any input param. Returns an array of String values with the notation "CAMLABEL:STATE" indicating whether a camera is working or not. CAMLABEL is the camera label keeping the notation explained before and STATE is 0 is something is wrong with the camera (disconnected, reading error, no space left on device) or if it's working properly.

### Parameters

This node doesn't use any specific parameters

### Environment variables

The node uses or is associated with next environment variables notation:
* VIDEO_XXXXXX_XXXXX
* GUI_XXXXXXXX_XXXXX
* VISION_XXXXX_XXXXX
* STREAMING_XX_XXXXX
* STITCHER_XXX_XXXXX
* PARTICLE_TRACKER_X
* PEPWAVE_XXXX_XXXXX
* DATA_CAPTURE_XXXXX

## Technical/Implementation details

### Threads usage

The structure of the node is the following:

The main process spawns different threads (one per camera) to read continuously the cameras stream. These threads are in the same context of the parent process, so this one can access their variables, i.e. the variables where the images are stored of the camera are stored. By default, each thread also flips the image read because the cameras are physically upside down (Trolls!).

**Implementation detail:** When the central camera is read in HD quality (1929x1080), it does not flip the image because it slows down a lot the thread loop. This case is handled in the main process by resizing first that image first and then flipping it.

In the same manner, a different thread is used for the heavy stitching process. The parent process tells the thread when to stitch the images passed by shared variables. And the result also can be read using a variable from the stitcher thread class.

### Mapping into memory

Why mapping into memory? What does that even mean?
Sharing many images between processes is computationally intensive, and adds many unwanted delays. That is why we decided to share images using shared memory between processes. That means that different processes can access a part of the memory that has the image arrays. That is done with `mmap` [Unix utility](https://en.wikipedia.org/wiki/Mmap). It maps a file into memory, and other processes that have the specific route to that file can also access that mapping.

It is relatively easy to use memory mapping on structured arrays. For that reason, we concatenate each BGR array (from each camera) into a big 3D array. We also concatenate the modified image that is going to be streamed to pilots console. The big array is then memory mapped using a library that internally uses `numpy.memmap`, which uses the Unix `mmap` seamless.

After the mapping of that big array, each image can be read from memory in any process. For that, this node identifies each image within that 3D array with a label (see next section). We use a library for this for reading images in python, but there's also a C++ version of that in this repo, in utils/lib/easy_memmap.cpp.

### External libraries

The main library used to handle the memory mapping is [easy_memmap](https://github.com/charlielito/easy_memmap) that uses under the hood `numpy.memmap`. For pretty logging, the library [extended_rospylogs](https://github.com/charlielito/extended_rospylogs) is used, basically just for printing the `[ROS NODE NAME]` before each print sentence in order to be able to discriminate between print messages from different nodes. It also has other features that you can check on the library's page.

#### easy_memmap

This module exposes a class named `MultiImagesMemmap` that assumes the use of RGB image arrays, and each of them can be identified with a label. In our case, we use the following convention for naming: "LL", "C", "RR", "B", "L", "R", "P", that correspond to the most left camera, the center camera, the back camera, the left camera, the right camera and the image that is going to be streamed to supervisors console. The big 3D array that has all BGR images concatenated needs to be also in the same order of the label array.

The constructor have the followig signature (an example is provided):

```python
video_map = MultiImagesMemmap(mode = "w", name = "main_stream", labels = ["LL", "C", "RR", "B", "L", "R", "P"], memmap_path = "/tmp)
```

where the `name` is a predefined name that identifies that memory-mapped array. (In our case, "main_stream" is used for this memmap sharing)

Remember that depending on which cameras are initialized or not in the `cams_config.yaml` file, the keys of this dictionary will be generated automatically.
