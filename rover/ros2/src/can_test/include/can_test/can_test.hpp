#ifndef CAN_TEST_H_INCLUDE
#define CAN_TEST_H_INCLUDE

#include <memory>
#include <utility>
#include <stdio.h>
#include <math.h>

/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>

/* ROS2 Messages */
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

/* ROS2 Services */
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

#define LIGHTS_ADDRESS 0x0B
#define LEDS_ON 0x01
#define LEDS_OFF 0x01

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


class CANTest : public rclcpp::Node
{
    public:
        // CANTest(const rclcpp::NodeOptions & options, CANDriver* can_driver);
        CANTest(const rclcpp::NodeOptions & options, std::shared_ptr<CANDriver> can_driver);

        ~CANTest(){};

        /* Public functions */
        void StartCANBusRead();
    
    private:
        /* Subscribers */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leds_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr config_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_sub_;

        /* Services */
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;

        /* Timers */
        rclcpp::TimerBase::SharedPtr sent_tmr_;

        /* Member functions */
        void LedsCb(const std_msgs::msg::Bool::SharedPtr msg);
        void ChassisCb(const std_msgs::msg::Bool::SharedPtr msg);
        void ConfigurationCb(const std_msgs::msg::Bool::SharedPtr msg);
        bool ArmCb(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        void TimerCb();


        void Configuration();
        void PIDConfiguration();
        void SendLeds();
        void SendBattery();
        void SendChassis();

        // CANDriver *can_dvr_;
        std::shared_ptr<CANDriver> can_dvr_;

        // std::shared_ptr<CANDriver> driver_can;
        
        /* Variables */
        bool chassis_flag_ = false;

        /* Socket CAN thread */
        std::thread read_thread_;
};

#endif
