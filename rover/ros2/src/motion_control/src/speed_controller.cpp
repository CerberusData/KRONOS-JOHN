/*
    - File name: speed_controller.cpp.
    - By: Camilo Andrès Alvis and Juan David Galvis
    - Email: camiloalvis@kiwibot.com
*/

#include "motion_control/speed_controller.hpp"

SpeedController::SpeedController(rclcpp::NodeOptions & options)
: Node("speed_controller", options)
{
    RCLCPP_DEBUG(this->get_logger(), "Speed Controller Init");

    // Publishers
    output_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/motion_control/speed_controller/output", 10);

    // Subscribers
    fr_cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/freedom_client/cmd_vel", 10, std::bind(&SpeedController::CommandsCb, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry/global_odometry", 50, std::bind(&SpeedController::OdometryCb, this, _1));
    imu_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/imu/status", 1, std::bind(&SpeedController::ImuStatusCb, this, _1));

    // Time to detect the dt inside the control loop
    prev_time_ = this->now();

    // Soft Speed object
    linear_soft_spline = std::make_shared<SoftSpeedSpline>(0.8f);
}

void SpeedController::ImuStatusCb(const std_msgs::msg::String::SharedPtr msg)
{
    /*
        - Topic: /imu/status
        - Type: std_msgs::msg::String
        - Action: Checks the current IMU State
    */
    bool status_update = (msg->data.compare("OK") == 0);
    if (status_update == false && imu_status_ == true)
    {
        RCLCPP_ERROR(this->get_logger(), "IMU not available for Speed Control");
    }

    imu_status_ = status_update;
}


void SpeedController::OdometryCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    /*
        - Topic: /wheel_odometry/global_odometry
        - Type: nav_msgs::msg::Odometry
        - Actions: 
            * Extracts current Kiwibot Pose and Velocities
            * It provides the Yaw angle for control purposes 
    */
    robot_twist_.twist = msg->twist.twist;

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
        yaw_set_point_ = bot_yaw_;
        first_yaw_value = true;
    }
}

void SpeedController::CommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    /*
        - Topic: /freedom_client/cmd_vel
        - Type: geometry_msgs::msg::TwistStamped
        - Action: Receives velocity commands from Freedom Robotics console
    */
    reference_cmd_.header.stamp = this->now();
    reference_cmd_.twist = msg->twist;
}

float SpeedController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    /*
        Args:
            * ref_wz = Angular reference velocity (Z-axis)
            * cur_wz = Current Kiwibot angular velocity (Z-axis)
            * dt = Delta time
        Returns: Angular velocity control command 
        Desc: A PID controller (Feedforward + Feedback) calculate the control 
            command for the specified velocity.
    */

    if (ref_wz == 0.0f)
    {
        wz_int_error_  = 0.0;
        return 0.0f;
    }

    if(steering_ctrl_ == true)
    {
        float prop_error = ref_wz - cur_wz;

        float der_error = (dt != 0.0f) 
            ? ((prop_error - wz_prop_ek1_) / dt) : 0.0f;
        wz_int_error_ += prop_error * dt;
        wz_prop_ek1_ = prop_error;

        // PID command with predictive term (FF) and reactive term (FB)
        float u_cmd = (kff_str_ * ref_wz) 
            + (kp_str_ * (prop_error + (ki_str_ * vx_int_error_) + (kd_str_ * der_error)));

        return u_cmd;
    }

    else
    {
        return ref_wz;
    }
}


float SpeedController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /*
        Args:
            * ref_vx = Linear reference velocity (X-axis)
            * cur_vx = Current Kiwibot linear velocity (X-axis)
            * dt = Delta time
        Returns: Linear velocity control command 
        Desc: A PID controller (Feedforward + Feedback) calculate the control 
            command for the specified velocity.
    */
    
    if (ref_vx == 0.0f)
    {
        // Condition used to avoid small movements in the robot
        vx_int_error_ = 0.0;
        return 0.0f;
    }

    if (throttle_ctrl_ == true)
    {
        float smooth_ff = 1.0f;
        // Proportional, Integral and Derivative errors for PID implementation
        float prop_error = (ref_vx - cur_vx) < 0.05f ? (ref_vx - cur_vx) : 0.0f;
        float der_error = (dt != 0.0f)
            ? ((prop_error - vx_prop_ek1_) / dt) : 0.0f;
        vx_int_error_ += prop_error * dt;
        vx_prop_ek1_ = prop_error;

        // Smoothing the predictive term when changing direction (Dynamic)
        if (((prev_ref_vx_ < 0.0f) && (ref_vx - prev_ref_vx_ > 0.0f)) 
            || ((prev_ref_vx_ > 0.0f) && (ref_vx - prev_ref_vx_ < 0.0f)))
        {   
            smooth_ff = 0.75f;
        }

        // PID command with predictive term (FF) and reactive term (FB)
        float u_cmd = (smooth_ff * kff_thr_ * ref_vx) 
            + (kp_thr_ * (prop_error + (ki_thr_ * vx_int_error_) + (kd_thr_ * der_error)));

        prev_ref_vx_ = ref_vx;

        // Anti Wind-Up (To avoid increasing the error)
        if (std::abs(ki_thr_ * vx_int_error_) > 0.25f)
        {
            vx_int_error_ -= prop_error * dt;
        }

        return u_cmd;
    }

    else
    {
        return ref_vx;
    }
}

void SpeedController::Controller()
{
    /*
        Args: NaN
        Returns: NaN 
        Desc: Main function which performs velocity control (Linear and angular).
    */
    auto output_cmd_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    float lin_vx = reference_cmd_.twist.linear.x;
    float ang_wz = reference_cmd_.twist.angular.z;

    rclcpp::Time curr_time = this->now();
    double dt = (curr_time - prev_time_).seconds();
    prev_time_ = this->now();

    // Arguments: (Reference, Acceleration Factor)
    float vx_ref = linear_soft_spline->CalculateSoftSpeed(lin_vx, 1.0f);
    output_cmd_msg->header.stamp = this->now();

    // Arguments: (Reference, Current, dt)
    output_cmd_msg->twist.angular.z = SteeringPID(ang_wz, robot_twist_.twist.angular.z, dt); 
    output_cmd_msg->twist.linear.x = ThrottlePID(vx_ref, robot_twist_.twist.linear.x, dt); 
    output_cmd_pub_->publish(std::move(output_cmd_msg));
}

// ToDo: Smart break conditions for handling the speed scaling factor
