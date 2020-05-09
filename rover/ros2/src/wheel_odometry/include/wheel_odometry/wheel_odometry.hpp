#ifndef WHEEL_ODOMETRY_CAN_H_INCLUDED
#define WHEEL_ODOMETRY_CAN_H_INCLUDED

#include <memory>
#include <utility>
#include <vector>
#include <math.h>

/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* Custom libraries */
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

/* Custom Messages */
#include "usr_msgs/msg/motors.hpp"

using std::placeholders::_1;

class WheelOdometry : public rclcpp::Node
{
    public:
        /* Constructor */
        WheelOdometry(const rclcpp::NodeOptions & options);
        ~WheelOdometry(){};
        
        /* Public functions */
        void CalculateOdometry();
        void PubTimerCb();

    private:
        /* Publishers */
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_global_pub_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_check_pub_;

        /* Subscribers */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movement_sub_;
        rclcpp::Subscription<usr_msgs::msg::Motors>::SharedPtr motor_status_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        /* Timers */
        rclcpp::TimerBase::SharedPtr pub_timer_;

        /* Member functions */
        void MovementCb(const std_msgs::msg::Bool::SharedPtr msg);
        void MotorStatusCb(const usr_msgs::msg::Motors::SharedPtr msg);
        void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);

        std::vector<double> motors_rpm_{0.0f, 0.0f, 0.0f, 0.0f};
        std::vector<double> motors_curr_{0.0f, 0.0f, 0.0f, 0.0f};

        rclcpp::Time prev_time_;

        nav_msgs::msg::Odometry wheel_odom_msg_;
        
        /* Kiwibot parameters */
        double wheel_rad_ = getEnv("CANLINK_CHASSIS_WHEEL_RADIUS", 0.074f) * 1.08f;
        double bot_track_ = getEnv("CANLINK_CHASSIS_ROBOT_TRACK", 0.392f); 

        double imu_roll_ = 0.0f;
        double imu_pitch_ = 0.0f;
        double imu_yaw_ = 0.0f;
        double imu_yaw_offset_ = 0.0f;
        double imu_omega_ = 0.0f;
        bool imu_state_ = false;
        bool imu_published_ = false;
};

#endif