#ifndef SPEED_CONTROLLER_H_INCLUDED
#define SPEED_CONTROLLER_H_INCLUDED

#include <memory>
#include <utility>
#include <vector>
#include <math.h>

/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>

/* Custom libraries */
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

/* TF2 Transformations */
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

/* ROS2 Services */
#include "std_srvs/srv/set_bool.hpp"

/* Custom Messages */
#include "usr_msgs/msg/motors.hpp"

using std::placeholders::_1;

class SpeedController : public rclcpp::Node
{
    public:
        SpeedController(rclcpp::NodeOptions & options);
        ~SpeedController(){};

        void Controller();

    private:
        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_cmd_pub_;

        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr driving_cmd_fr_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

        // Timers
        rclcpp::TimerBase::SharedPtr pub_timer_;

        // Member functions
        void CommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
        void OdometryCb(const nav_msgs::msg::Odometry::SharedPtr msg);
        float ThrottlePID(double ref_vx, double cur_vx, double dt);

        // Environment variables
        bool throttle_ctrl_ = getEnv("SPEED_CONTROLLER_THROTTLE_CONTROL_ENABLE", true);
        double kp_thr_ = getEnv("SPEED_CONTROLLER_KP_THROTTLE", 0.2f);
        double ki_thr_ = getEnv("SPEED_CONTROLLER_KI_THROTTLE", 0.2f);
        double kd_thr_ = getEnv("SPEED_CONTROLLER_KD_THROTTLE", 0.0f);
        double kff_thr_ = getEnv("SPEED_CONTROLLER_FF_THROTTLE", 1.0f);
        
        /* -- */
        geometry_msgs::msg::TwistStamped robot_twist_;
        geometry_msgs::msg::TwistStamped reference_cmd_;

        rclcpp::Time prev_time_;

        double int_error_ = 0.0f;
        double e_k1_ = 0.0f;
        double prev_ref_vx_ = 0.0f;

        double yaw_set_point_ = 0.0f;
        double bot_yaw_ = 0.0f;
        bool first_yaw_value = false;

};
#endif
