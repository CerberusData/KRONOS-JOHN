#include "wheel_odometry/wheel_odometry.hpp"
#define PI 3.1516

WheelOdometry::WheelOdometry(const rclcpp::NodeOptions & options)
: Node("odometry", options)
{
    RCLCPP_INFO(this->get_logger(), "Odometry constructor");

    // Publishers
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wheel_odometry/local_odometry", 50);
    wheel_odom_global_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wheel_odometry/global_odometry", 50);

    // Subscribers
    motor_status_sub_ = this->create_subscription<usr_msgs::msg::Motors>(
        "/canlink/chassis/motors_status", 10, std::bind(&WheelOdometry::MotorStatusCb, this, _1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_bot", 10, std::bind(&WheelOdometry::ImuCb, this, _1));
    movement_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/imu/move_state", 10, std::bind(&WheelOdometry::MovementCb, this, _1));

    // Services
    restart_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/wheel_odometry/restart", std::bind(&WheelOdometry::RestartCb, this, _1, _2, _3));

    prev_time_ = this->now();
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), 
        std::bind(&WheelOdometry::PubTimerCb, this));

    local_wheel_odom_msg_.header.frame_id = "local_odom";
    local_wheel_odom_msg_.child_frame_id = "base_link";    

    global_wheel_odom_msg_.header.frame_id = "global_odom";
    global_wheel_odom_msg_.child_frame_id = "base_link";
}

/* ------------------------------------------------------------------------- */
// Callbacks
bool WheelOdometry::RestartCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    /* 
        This service is in charge of restarting the odometry values and set the
        IMU offset for the new movement.
    */
    (void) request_header;
    (void) request;

    local_wheel_odom_msg_.pose.pose.position.x = 0.0f;
    local_wheel_odom_msg_.pose.pose.position.y = 0.0f;
    local_wheel_odom_msg_.pose.pose.position.z = 0.0f;
    
    imu_yaw_offset_ = imu_yaw_;

    // Quaternion representation
    tf2::Quaternion quat;
    quat.setRPY(0.0f, 0.0f, 0.0f);

    // Quaternion assignation
    local_wheel_odom_msg_.pose.pose.orientation.x = quat[0];
    local_wheel_odom_msg_.pose.pose.orientation.y = quat[1];
    local_wheel_odom_msg_.pose.pose.orientation.z = quat[2];
    local_wheel_odom_msg_.pose.pose.orientation.w = quat[3];
    wheel_odom_pub_->publish(local_wheel_odom_msg_);

    // Service response
    response->success = true;
    response->message = "Odometry Restarted";
    
    return true;
} 

void WheelOdometry::MovementCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    /* 
        Callback for the topic /imu/move_state. It represents if the Kiwibot is
        moving or not based on the IMU data.
    */
    imu_state_ = msg->data;
}

void WheelOdometry::MotorStatusCb(const usr_msgs::msg::Motors::SharedPtr msg)
{
    /*
        Callback for the topic canlink/chassis/motors_status. Custom message con-
        taining the following elements:
            - header
            - rpm
            - current
            - error_status
        
        Located at usr_msgs/msg/canlink
    */

    motors_rpm_ = msg->rpm;
    motors_curr_ = msg->current;
}

void WheelOdometry::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    /*
        Callback for the topic /imu/data_bot. 
        It updates a Quaternion to extract the current euler angles.
    */

    tf2::Quaternion q(msg->orientation.x, 
                    msg->orientation.y,
                    msg->orientation.z, 
                    msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(imu_roll_, imu_pitch_, imu_yaw_);
    imu_omega_ = msg->angular_velocity.z;
    
    if (imu_published_ == false)
    {
        // IMU state must be OK
        imu_published_ = true;
    }

    RCLCPP_DEBUG(this->get_logger(), 
        "Roll: %0.4f, Pitch: %0.4f, Yaw: %0.4f", 
        imu_roll_, imu_pitch_, imu_yaw_);

}

/* ------------------------------------------------------------------------- */
// Functions
float WheelOdometry::CalculateSlipFactor(float kin_omega, float imu_omega)
{
    /*
        Difference calculation between the kinematic and IMU angular speed.
    */
    RCLCPP_INFO(this->get_logger(), 
        "Kinematic: %0.4f, IMU: %0.4f", 
        kin_omega, imu_omega);

    float alpha = 1.0f;
    if (kin_omega != 0.0f)
    {
        alpha = 1.0f - ((kin_omega - imu_omega) / 2.0f); 
        alpha = alpha < 0.0f ? 0.0f : alpha;
        alpha = alpha > 1.0f ? 1.0f : alpha;
    }

    return alpha;
}

void WheelOdometry::CalculateOdometry()
{
    /*
        Function to calculate the Wheel Odometry (Local and Global)
    */

    // Wheels linear velocities
    float FR_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[0]) / 60.0f; 
    float RR_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[1]) / 60.0f; 
    float RL_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[2]) / 60.0f; 
    float FL_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[3]) / 60.0f; 

    // Left and Right linear velocities
    float R_vel = -(FR_vel + RR_vel) / 2.0f;  
    float L_vel = (FL_vel + RL_vel) / 2.0f;
    // Lineal and angular velocities
    float X_vel = (R_vel + L_vel) / 2.0f;
    float kin_omega = (R_vel - L_vel) / bot_track_;

    rclcpp::Time curr_time = this->now();
    double dt = (curr_time - prev_time_).seconds();
    prev_time_ = this->now();

    // Filling a Quaternion with the current euler angles
    tf2::Quaternion quat;
    quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_ - imu_yaw_offset_);

    // Header assignation
    local_wheel_odom_msg_.header.stamp = this->now();
    // Quaternion assignation
    local_wheel_odom_msg_.pose.pose.orientation.x = quat[0];
    local_wheel_odom_msg_.pose.pose.orientation.y = quat[1];
    local_wheel_odom_msg_.pose.pose.orientation.z = quat[2];
    local_wheel_odom_msg_.pose.pose.orientation.w = quat[3];
    // Speeds in X and Y axes
    float X_dot = X_vel * cos(imu_yaw_ - imu_yaw_offset_);
    float Y_dot = X_vel * sin(imu_yaw_ - imu_yaw_offset_);
    // Adding displacement in [m] to the current message
    local_wheel_odom_msg_.pose.pose.position.x += X_dot * dt;
    local_wheel_odom_msg_.pose.pose.position.y += Y_dot * dt;
    local_wheel_odom_msg_.pose.pose.position.z = 0.0f;
    local_wheel_odom_msg_.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                        0, 0.1, 0, 0, 0, 0,
                                        0, 0, 0.2, 0, 0, 0,
                                        0, 0, 0, 0.001, 0, 0,
                                        0, 0, 0, 0, 0.001, 0,
                                        0, 0, 0, 0, 0, 0.002};
    // Velocities assignation
    local_wheel_odom_msg_.twist.twist.linear.x = X_vel;
    local_wheel_odom_msg_.twist.twist.linear.y = 0.0f;
    local_wheel_odom_msg_.twist.twist.linear.z = 0.0f;
    local_wheel_odom_msg_.twist.twist.angular.x = 0.0f;
    local_wheel_odom_msg_.twist.twist.angular.y = 0.0f;
    local_wheel_odom_msg_.twist.twist.angular.z = imu_omega_;

    // Testing purposes
    local_wheel_odom_msg_.twist.covariance = {0.0000001, 0, 0, 0, 0, 0,
                                        0, 0.000000001, 0, 0, 0, 0,
                                        0, 0, 0.00000002, 0, 0, 0,
                                        0, 0, 0, 0.01, 0, 0,
                                        0, 0, 0, 0, 0.01, 0,
                                        0, 0, 0, 0, 0, 0.002};

    /* Twist covariance tunning (Velocities) */
    // if ((imu_state_ == true) && X_vel == 0.0)
    // { //Robot is moving but wheels not (e.g. robot is being carried)
    //     /* 
    //         When robot is moving, but wheels not, hence we should avoid adding
    //         values to the linear components.
    //     */
    //     local_wheel_odom_msg_.twist.covariance = {999, 0, 0, 0, 0, 0,
    //                                         0, 999, 0, 0, 0, 0,
    //                                         0, 0, 999, 0, 0, 0,
    //                                         0, 0, 0, 0.001, 0, 0,
    //                                         0, 0, 0, 0, 0.001, 0,
    //                                         0, 0, 0, 0, 0, 0.002};
    // }
    // else
    // {
    //     /* 
    //         When robot is moving and wheel are moving, so we trust the readings
    //         in all the components.
    //     */
    //     local_wheel_odom_msg_.twist.covariance = {0.0000001, 0, 0, 0, 0, 0,
    //                                         0, 0.000000001, 0, 0, 0, 0,
    //                                         0, 0, 0.00000002, 0, 0, 0,
    //                                         0, 0, 0, 0.01, 0, 0,
    //                                         0, 0, 0, 0, 0.01, 0,
    //                                         0, 0, 0, 0, 0, 0.002};
    // }

    /* Pose covariance tunning (Positions) */
    

    // Calculations for Global Wheel Odometry
    float alpha = CalculateSlipFactor(kin_omega, imu_omega_);
    float X_vel_corrected = X_vel * alpha;

    // Filling a global Quaternion with the current euler angles
    tf2::Quaternion quat_global;
    quat_global.setRPY(imu_roll_, imu_pitch_, imu_yaw_);
    // Header assignation
    global_wheel_odom_msg_.header.stamp = this->now();
    // Quaternion assignation
    global_wheel_odom_msg_.pose.pose.orientation.x = quat_global[0];
    global_wheel_odom_msg_.pose.pose.orientation.y = quat_global[1];
    global_wheel_odom_msg_.pose.pose.orientation.z = quat_global[2];
    global_wheel_odom_msg_.pose.pose.orientation.w = quat_global[3];
    // Speeds in X and Y axes
    float X_dot_global = X_vel_corrected * cos(imu_yaw_ - imu_yaw_offset_);
    float Y_dot_global = X_vel_corrected * sin(imu_yaw_ - imu_yaw_offset_);
    // Adding displacement in [m] to the global message
    global_wheel_odom_msg_.pose.pose.position.x += X_dot_global * dt;
    global_wheel_odom_msg_.pose.pose.position.y += Y_dot_global * dt;
    global_wheel_odom_msg_.pose.pose.position.z = 0.0f;
    global_wheel_odom_msg_.pose.covariance = local_wheel_odom_msg_.pose.covariance;
    // Velocities assignation
    global_wheel_odom_msg_.twist.twist.linear.x = X_vel_corrected;
    global_wheel_odom_msg_.twist.twist.linear.y = 0.0f;
    global_wheel_odom_msg_.twist.twist.linear.z = 0.0f;
    global_wheel_odom_msg_.twist.twist.angular.x = 0.0f;
    global_wheel_odom_msg_.twist.twist.angular.y = 0.0f;
    global_wheel_odom_msg_.twist.twist.angular.z = imu_omega_;
    global_wheel_odom_msg_.twist.covariance = local_wheel_odom_msg_.twist.covariance;

    if (imu_published_ == true)
    {
        wheel_odom_pub_->publish(local_wheel_odom_msg_);
        wheel_odom_global_pub_->publish(global_wheel_odom_msg_);
    } 
}

void WheelOdometry::PubTimerCb()
{
    CalculateOdometry();
}