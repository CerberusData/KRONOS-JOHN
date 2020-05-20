#ifndef CAN_TEST_H_INCLUDE
#define CAN_TEST_H_INCLUDE

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "can_test/socketCAN.hpp"

#define CHASSIS_ADDRESS 0x45A
#define KIWIBOT_ADDRESS 0x469
#define CONFIGURATION_CMD_ID 0x01
#define ARM_CMD_ID 0x02
#define WHEELS_INFO_ID 0x03
#define WHEELS_CONTROL_RPM_ID 0x04
#define MOTORS_CONTROL_RAW_ID 0x05
#define MOTORS_CURRENT_ID 0x06
#define ERROR_CMD_ID 0x07
#define TEST_REPORT_CMD_ID 0x08
#define RESET_CMD_ID 0x09
#define STATUS_CMD_ID 0x0A
#define PID_CMD_ID 0x0C
#define SOFT_BRAKE_CMD_ID 0x0D
 
#define DISABLE_INTEGRAL_LIMIT 0X00
#define WHEELS_BAUDRATE_DEFAULT 0x15
#define BATTERY_STATUS_BAUDRATE_DEFAULT 0x02
#define WHEEL_CONTROL_MODE_RPM 0x00
#define WHEEL_CONTROL_MODE_RAW 0x01
#define OPERATION_MODE_NORMAL 0x00
#define OPERATION_MODE_TEST 0x01
#define OPERATION_MODE_PROGRAMMING 0x02
#define OPERATION_MODE_RESET 0x03
 
#define NUMBER_BATTERY_CELLS_DEFAULT 0x04
#define MOTORS_MODEL_DEFAULT 0x00
#define MOTORS_CURRENT_DEFAULT 0x01
 
#define RIGHT_FRONT_WHEEL 0x01
#define RIGHT_REAR_WHEEL 0x02
#define LEFT_REAR_WHEEL 0x04

#define LEDS_ON 0x01
#define LEDS_OFF 0x01

using std::placeholders::_1;

class CANTest : public rclcpp::Node
{
    public:
        CANTest(const rclcpp::NodeOptions & options, CANDriver* can_driver);
        ~CANTest(){};
    
    private:
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leds_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_sub_;

        void LedsCb(const std_msgs::msg::Bool::SharedPtr msg);
        void BatteryCb(const std_msgs::msg::Bool::SharedPtr msg);
        void ChassisCb(const std_msgs::msg::Bool::SharedPtr msg);

        void Configuration();
        void SendLeds();
        void SendBattery();
        void SendChassis();

        CANDriver *can_dvr_;
        
};



#endif
