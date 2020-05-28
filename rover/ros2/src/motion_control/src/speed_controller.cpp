#include "motion_control/speed_controller.hpp"

SpeedController::SpeedController(rclcpp::NodeOptions & options)
: Node("speed_controller", options)
{
    RCLCPP_INFO(this->get_logger(), "Speed Controller constructor");

    /* Publishers */
    output_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/motion_control/speed_controller/output", 10);

    /* Subscribers */ 
    driving_cmd_fr_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/freedom_client/joystick_commands", 10, std::bind(&SpeedController::CommandsCb, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry/global_odometry", 50, std::bind(&SpeedController::OdometryCb, this, _1));

}
void SpeedController::OdometryCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{

    /*
        Odometry callback to extract the current velocity and pose comming from Wheel odometry
        We are using global wheel odometry, but I need to check if we should use the local one.
    */
    robot_twist_->twist = msg->twist.twist;

    double roll = 0.0f; 
    double pitch = 0.0f;
    tf2::Quaternion q(msg->pose.pose.orientation.x, 
                    msg->pose.pose.orientation.y, 
                    msg->pose.pose.orientation.z, 
                    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, bot_yaw_);

    if (first_yaw_value == false)
    {
        yaw_set_point = bot_yaw_;
        first_yaw_value = true;
    }
}

void SpeedController::CommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    auto commands_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    /* Control commands assignation */
    commands_msg->header.stamp = this->now();
    commands_msg->twist.linear.x = msg->twist.linear.x;
    commands_msg->twist.linear.y = 0.0f; 
    commands_msg->twist.linear.z = 0.0f;
    commands_msg->twist.angular.x = 0.0f;
    commands_msg->twist.angular.y = 0.0f; 
    commands_msg->twist.angular.z = msg->twist.angular.z;

    output_cmd_pub_->publish(std::move(commands_msg));
}

float SpeedController::ThrottlePID(float vx_ref)
{
    if (vx_ref == 0.0f)
    {
        return 0.0f;
    }
    // error = ref - current velocity

    return 0.0f;
}

// float SpeedController::PIDCmd()
// {
//     /*
//         This function will calculate and return the PID command for the given 
//         integral, derivative and proportional errors

//         The velocity representation (For PID discretization) is used here.
//     */

// }




void SpeedController::Controller()
{
    RCLCPP_INFO(this->get_logger(), "Controller function");
}