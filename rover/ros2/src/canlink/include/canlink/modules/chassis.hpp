/*
    - File name:chassis.h
    - This library defines members and member functions for the CAN communication with the chassis
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#ifndef CAN_CHASSIS_H_INCLUDED
#define CAN_CHASSIS_H_INCLUDED

#include <memory>
#include <utility>
#include <vector>
#include <math.h>

/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/* Custom libraries */
#include "canlink/socketCAN.hpp"
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

/* Custom Messages */
#include "usr_msgs/msg/configuration.hpp"
#include "usr_msgs/msg/motors.hpp"
#include "usr_msgs/msg/test_motors.hpp"
#include "usr_msgs/msg/pwm_out.hpp"
#include "usr_msgs/msg/state.hpp"
#include "usr_msgs/msg/messages.hpp"

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
#define LEFT_FRONT_WHEEL 0x08
 
using std::placeholders::_1;

class Chassis : public rclcpp::Node
{
    public:
        /* Constructor */
        Chassis(const rclcpp::NodeOptions & options, CANDriver *can_driver);
        ~Chassis(){};

        /* Public functions */
        void PublishChassisStatus(struct can_frame* frame);
        void PublishMotorStatus(struct can_frame* frame);
        void PublishTestReport(struct can_frame* frame);
        void SetErrorStatus(struct can_frame* frame);
        void SetMotorsCurrent(struct can_frame* frame);
        bool GetConnected();

    private:
        /* Publishers */
        rclcpp::Publisher<usr_msgs::msg::State>::SharedPtr motors_dvr_status_pub_;
        rclcpp::Publisher<usr_msgs::msg::PWMOut>::SharedPtr motors_out_pub_;
        rclcpp::Publisher<usr_msgs::msg::Motors>::SharedPtr motors_status_pub_;
        rclcpp::Publisher<usr_msgs::msg::TestMotors>::SharedPtr test_motors_pub_;
        rclcpp::Publisher<usr_msgs::msg::Messages>::SharedPtr msg_pub_;
        std::vector<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> current_pub_;

        /* Publishers messages */
        usr_msgs::msg::PWMOut motors_out_msg_;
        usr_msgs::msg::Motors motors_status_;
        usr_msgs::msg::TestMotors test_motors_response_;
        usr_msgs::msg::State motors_dvr_status_;

        /* Subscribers */
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_control_out_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_control_ref_sub_;
        rclcpp::Subscription<usr_msgs::msg::Configuration>::SharedPtr chassis_config_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motors_sleep_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_test_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_restart_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        /* Subscribers callbacks */
        void ActuatorControlCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void ActuatorReferenceCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void ChassisConfigCb(const usr_msgs::msg::Configuration::SharedPtr msg);
        void MotorsSleepCb(const std_msgs::msg::Bool::SharedPtr msg);
        void ChassisTestCb(const std_msgs::msg::Bool::SharedPtr msg);
        void ChassisRestartCb(const std_msgs::msg::Bool::SharedPtr msg); 
        void OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);

        /* Timers */
        rclcpp::TimerBase::SharedPtr heartbeat_tmr_;
        rclcpp::TimerBase::SharedPtr moon_tmr_;
        rclcpp::TimerBase::SharedPtr current_tmr_;

        /* Timers callbacks */
        void HeartbeatTimerCb();
        void MoonTimerCb();
        void CurrentTimerCb();

        /* Objects */
        CANDriver *can_driver_;

        /* Member Messages */
        usr_msgs::msg::Configuration chassis_cfg_;  /* To Do: Use as a shared pointer */

        /* Member Functions */
        void ConfigurePID();
        void InitialConfig();
        void SendChassisConfiguration();
        bool SendMotorsCmd();
        uint8_t RadsToDigital(float control);

        /* Environment variables */
        float kp_ = getEnv("CANLINK_CHASSIS_KP_WHEELS", 5.5f);
        float ki_ = getEnv("CANLINK_CHASSIS_KI_WHEELS", 0.35f);
        float kd_ = getEnv("CANLINK_CHASSIS_KD_WHEELS", 0.0f);
        int overcur_tmr_duration_ = getEnv("CANLINK_CHASSIS_MOTOR_ERROR_DURATION", 4000);
        bool stop_on_motor_anomaly_ = getEnv("CANLINK_CHASSIS_STOP_ON_MOTOR_ANOMALY", false);
        float current_slope_ = getEnv("CANLINK_CHASSIS_CURRENT_SLOPE", 0.00555067f);
        float current_intercept_ = getEnv("CANLINK_CHASSIS_CURRENT_INTERCEPT", -1.8746168f);
        float min_current_ = getEnv("CANLINK_CHASSIS_MIN_CURRENT", 5.0f);
        float max_allowed_current_ = getEnv("CANLINK_CHASSIS_MAX_CURRENT", 16.0f);
        float wheel_max_rpm_ = getEnv("CANLINK_CHASSIS_WHEEL_MAX_RPM", 165.0f);
        float robot_length_ = getEnv("CANLINK_CHASSIS_ROBOT_TRACK", 0.4f);
        float radius_ = getEnv("CANLINK_CHASSIS_WHEEL_RADIUS", 0.074f);
        float max_allowed_pitch_ = getEnv("CANLINK_CHASSIS_MAX_PITCH", 40.0f) * 3.1416f/180.0f;
        
        /* Constants */
        bool publish_currents_separately_ = false;

        /* Variables */
        int moon_view_ = 0;
        int motor_error_state_ = 0;
        float throttle_prev_ = 0.0f;
        float throttle_current_ = 0.0f;
        float speed_pitch_factor_ = 100.0f;
        double pitch_ = 0.0f;
        bool moving_ = true;
        bool moon_first_time_ = false;
        bool motors_current_ok_ = true;
        bool current_timer_started_ = false;
        bool accelerating_ = false;
        bool soft_brake_ = true;
        std::vector<float> controls_ = {0.0f, 0.0f, 0.0f};
        std::vector<uint16_t> raw_motors_out_;
        uint16_t motor_error_[4] = {0, 0, 0, 0};  /* Double check if it makes sense */

};
#endif  /* End of CAN_CHASSIS_H_INCLUDED */ 
 
 
