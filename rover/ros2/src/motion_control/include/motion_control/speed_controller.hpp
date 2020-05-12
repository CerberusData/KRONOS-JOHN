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
#include "geometry_msgs/msg/twist_stamped.hpp"

/* ROS2 Services */
#include "std_srvs/srv/set_bool.hpp"

/* Custom Messages */
#include "usr_msgs/msg/motors.hpp"

using std::placeholders::_1;

class SpeedController : public rclcpp::Node
{
    public:
        /* Constructor */
        SpeedController(rclcpp::NodeOptions & options);
        ~SpeedController(){};

    private:
        /* Subscribers */
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr driving_cmd_fr_;

        /* Timers */
        rclcpp::TimerBase::SharedPtr pub_timer_;

        /* Member functions */
        void CommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

};
#endif
