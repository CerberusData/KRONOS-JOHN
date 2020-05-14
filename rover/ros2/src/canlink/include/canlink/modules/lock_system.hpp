/*
    - File name: lock_system.h
    - This library defines members and member functions for the battery module's communication through CAN
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#ifndef CAN_LOCK_SYSTEM_H_INCLUDED
#define CAN_LOCK_SYSTEM_H_INCLUDED
 
/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>

/* Custom libraries */
#include "canlink/socketCAN.hpp"
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
 
#define LOCK_SYSTEM_ADDRESS 0x0D4
#define KIWIBOT_ADDRESS 0x469
#define CONFIGURATION_CMD_ID 0x01
#define OPEN_LOCK_ID 0x02
#define LOCK_STATUS_ID 0x03
#define TOGGLE_DOOR_ID 0x04
#define REQUEST_LOCK_STATUS 0x01

using std::placeholders::_1;

  
class LockSystem : public rclcpp::Node
{   
    public:
        LockSystem(const rclcpp::NodeOptions & options, CANDriver *can_driver);
        ~LockSystem(){};

        /* Functions */
        void PublishLockStatus(struct can_frame *frame);
        void PublishToggleDoor(void);

    private:
        /* Publishers */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_system_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_system_toggle_door_pub_;

        /* Subscribers */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_system_open_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_system_request_status_sub_;

        /* Member functions */
        void OpenLockCb(const std_msgs::msg::Bool::SharedPtr msg);
        void RequestLockStatusCb(const std_msgs::msg::Bool::SharedPtr msg); 

        /* Objects */
        CANDriver *can_driver_;

};
#endif //CAN_LOCK_SYSTEM_H_INCLUDED
 
 
