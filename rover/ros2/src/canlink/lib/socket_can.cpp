/* 
    - File name: socket_can.cpp
    - This library defines functions to communicate via a CAN interface with external can devices
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/

#include "canlink/socketCAN.hpp"
 
CANDriver::CANDriver(const char *interface_name)
{
    /* Opening the socket */
    const char *ifname_ = interface_name;
    if ((sckt_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while opening socket");
    }
    
    strcpy(ifr_.ifr_name, ifname_);
    ioctl(sckt_, SIOCGIFINDEX, &ifr_);

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    printf("%s at index %d\n", ifname_, ifr_.ifr_ifindex);
    
    if (bind(sckt_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
    {
        perror("Error in socket bind.");
    }
}
 
void CANDriver::ListenSocket()
{
    struct can_frame rxmsg;
    bool read_can_port = true;
    while (read_can_port)
    {
        read(sckt_, &rx_frame_, sizeof(rxmsg));
        new_msg_rx = true;
    }
}
 
struct can_frame *CANDriver::ReadMsg()
{
    if (new_msg_rx)
    {
        new_msg_rx = false;
        return &rx_frame_;
    }
    return NULL;
}
 
struct can_frame *CANDriver::ReadSocket()
{
    struct can_frame rxmsg;
    read(sckt_, &rx_frame_, sizeof(rxmsg));
    return &rx_frame_;
}
 
 
int CANDriver::CANWrite(int can_id, int can_data_length, uint8_t *data)
{
    frame_.can_id = can_id;
    frame_.can_dlc = can_data_length;
    for (int i = 0; i < can_data_length; ++i)
    {
        frame_.data[i] = data[i];
    }
    nbytes_ = write(sckt_, &frame_, sizeof(struct can_frame));
    return 0;
}

