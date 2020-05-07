#include <memory>
#include <utility>
#include <vector>
#include <math.h>

/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>

/* Custom libraries */
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

    private:
        /* Publishers */
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_global_pub_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr radio_check_pub_;

        rclcpp::TimerBase::SharedPtr pub_timer_;

        void PubTimerCb();
};