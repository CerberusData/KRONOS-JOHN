modprobe can
modprobe can-raw
modprobe can_dev
modprobe mttcan
ip link set can0 type can bitrate 500000
ip link set can1 type can bitrate 500000 sjw 4 dbitrate 5000000 dsjw 4 berr-reporting on fd on
ip link set up can1
ip link set up can0
sleep 5


# modprobe can
# modprobe can-raw
# modprobe mttcan
# ip link set can0 type can bitrate 500000
# ip link set can1 type can bitrate 500000 sjw 4 dbitrate 5000000 dsjw 4 berr-reporting on fd on
# ip link set up can0
# ip link set up can1

# sleep 5