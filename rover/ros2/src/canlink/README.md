
Publishers

| Topic name                      | Message type              | Module   |
|---------------------------------|:-------------------------:|---------:|
| /canlink/chassis/status         | usr_msgs::msg::State      | Chassis  |
| /canlink/chassis/motors_out     | usr_msgs::msg::PWMOut     | Chassis  |
| /canlink/chassis/motors_status  | usr_msgs::msg::Motors     | Chassis  |
| /canlink/chassis/test_response  | usr_msgs::msg::TestMotors | Chassis  |

Subscribers

|  Topics  |      Message type     |   Module  |
|----------|:---------------------:|----------:|
| col 1 is |  left-aligned         | $1600     |
| col 1 is |  left-aligned         | $1600     |
| col 1 is |  left-aligned         | $1600     |
| col 1 is |  left-aligned         | $1600     |
| col 1 is |  left-aligned         | $1600     |

Services




CONFIGURATION: 45A [01 15 02 00 00 04 00 01] 
PID CONFIGURATION: 45A [0C 05 7F 00 59 00 00 00] 
ARM: 45A [02 01] 
DISARM: 45A [02 00]
SOFTBRAKE: 45A [0D 01]
LEDS: 45A [0B 01 01]
WHEELSCMD: 45A [04 05 25 25 25 25]

Socket CAN configuration at balena_start.sh

# CAN Interfaces setup and initialization
ip link set can0 type can bitrate 500000
ip link set can1 type can bitrate 500000 sjw 4 dbitrate 5000000 dsjw 4 berr-reporting on fd on
ip link set up can0
ip link set up can1

modprobe mttcan
modprobe can_raw

can-utils
candump can0
cansend can0 45a#0C057F0059000000 - Configuration message
