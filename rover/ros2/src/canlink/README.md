# **Canlink**
Socket CAN is implemented in this node to achieve the CAN communication between the CMC and Jetson TX2

## **Dependencies**
Dependencies needes for this module are already included within the Dockerfile.
`can_utils`

## **Libraries**
Socket CAN library is required for reading and writing to the defined socket.

## **Publishers**

| Topic name                      | Message type                    | Module   |
|---------------------------------|:-------------------------------:|---------:|
| /canlink/chassis/status         | usr_msgs::msg::ChassisStatus    | Chassis  |
| /canlink/chassis/motors_out     | usr_msgs::msg::PWMOut           | Chassis  |
| /canlink/chassis/motors_status  | usr_msgs::msg::Motors           | Chassis  |
| /canlink/chassis/test_response  | usr_msgs::msg::TestMotors       | Chassis  |
| /canlink/chassis/current        | std_msgs::msg::Float32          | Chassis  |
| /canlink/battery/data           | sensor_msgs::msg::BatteryState  | Battery  |
| /canlink/battery/status         | std_msgs::msg::String           | Battery  |
| /canlink/battery/voltage_FR     | std_msgs::msg::Float32          | Battery  |



## **Subscribers**

| Topic name                                | Message type                      | Module   |
|-------------------------------------------|:---------------------------------:|---------:|
| /canlink/chassis/configuration            | usr_msgs::msg::Configuration      | Chassis  |
| /canlink/chassis/sleep                    | std_msgs::msg::Bool               | Chassis  |
| /canlink/chassis/test                     | std_msgs::msg::Bool               | Chassis  |
| /canlink/chassis/restart                  | std_msgs::msg::Bool               | Chassis  |
| /wheel_odometry/global_odometry           | nav_msgs::msg::Odometry           | Chassis  |
| /motion_control/speed_controller/output   | geometry_msgs::msg::TwistStamped  | Chassis  |
| /motion_control/speed_controller/reference| geometry_msgs::msg::TwistStamped  | Chassis  |

## **Services**

| Service name          | Service type              | Module   |
|-----------------------|:-------------------------:|---------:|
| /canlink/chassis/arm  | std_srvs::srv::SetBool    | Chassis  |

## **Socket CAN configuration**
Socket CAN configurations is added in the balena_start.sh script. Kernel loading is not required (i.e. modprobe: mttcan, can, can_raw) as we are working inside a container and the Kernel and its modules are not included inside the container.

> ip link set can0 type can bitrate 500000
> ip link set up can0
> ip link set can1 type can bitrate 500000 sjw 4 dbitrate 5000000 dsjw 4 berr-reporting on fd on
> ip link set up can1

## **Debugging**
Canlink Node (Including Chassis and Cabin) is the base of the Kiwibot, so it is critical to debug the problems and issues in the ndoe. Here you will find some important information regarding direcctions and messages to execute specific functions.

* **PID CONFIGURATION:** 45A [0C 05 7F 00 59 00 00 00] 
* **CONFIGURATION:** 45A [01 15 02 00 00 04 00 01] 
* **WHEELSCMD:** 45A [04 05 25 25 25 25]
* **SOFTBRAKE:** 45A [0D 01]
* **LEDS:** 45A [0B 01 01]
* **DISARM:** 45A [02 00]
* **ARM:** 45A [02 01] 

### **Commands**
Use candump `ca_id`n to hear the desired Socket CAN.
> candump can0

Use cansend `ca_id` `dir#data`
> cansend can0 45a#0C057F0059000000


Links and notion image 

To Do:
QoS definition for ROS2

Last updated: 