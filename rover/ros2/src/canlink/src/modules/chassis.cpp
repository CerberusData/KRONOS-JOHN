/*
    - File name: chassis.cpp.
    - This library defines members and member functions for the CAN communication with the chassis
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
#include "canlink/modules/chassis.hpp"

Chassis::Chassis(const rclcpp::NodeOptions & options, std::shared_ptr<CANDriver>can_driver)
: Node("chassis", options)
{
    
    RCLCPP_INFO(this->get_logger(), "Chassis init");
    if (can_driver)
    {
        can_driver_ = can_driver;

        /* Chassis and Motors status initialization */
        motors_out_msg_.info = motors_out_msg_.DEFAULT;  /* Double check*/
        motors_dvr_status_.mode = "JETHAWK";  /* Double check*/
        motors_dvr_status_.armed = false;
        motors_dvr_status_.connected = true;

        /* Publishers */
        motors_dvr_status_pub_ = this->create_publisher<usr_msgs::msg::State>(
            "/canlink/chassis/status", 10);
        motors_out_pub_ = this->create_publisher<usr_msgs::msg::PWMOut>(
            "/canlink/chassis/motors_out", 10);
        motors_status_pub_ = this->create_publisher<usr_msgs::msg::Motors>(
            "/canlink/chassis/motors_status", 10);
        test_motors_pub_ = this->create_publisher<usr_msgs::msg::TestMotors>(
            "/canlink/chassis/test_response", 10);
        msg_pub_ = this->create_publisher<usr_msgs::msg::Messages>(
            "/web_client/message", 512); // This message can be removed

        if(publish_currents_separately_)  /* Separate current Publishers */
        {
            for(int i = 0; i < 4; ++i)
            {
                std::string topic_name = "/canlink/chassis/current" + std::to_string(i + 1);
                rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_pub =
                    this->create_publisher<std_msgs::msg::Float32>(topic_name, 10);
                current_pub_.push_back(current_pub);
            }
        }

        /* Subscribers */
        speed_control_out_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/motion_control/speed_controller/output", 10, std::bind(&Chassis::ActuatorControlCb, this, _1));
        speed_control_ref_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/motion_control/speed_controller/reference", 10, std::bind(&Chassis::ActuatorReferenceCb, this, _1));
        chassis_config_sub_ = this->create_subscription<usr_msgs::msg::Configuration>(
            "/canlink/chassis/configuration", 10, std::bind(&Chassis::ChassisConfigCb, this, _1));
        motors_sleep_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/canlink/chassis/sleep", 10, std::bind(&Chassis::MotorsSleepCb, this, _1));
        chassis_test_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/canlink/chassis/test", 10, std::bind(&Chassis::ChassisTestCb, this, _1));
        chassis_restart_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/canlink/chassis/restart", 10, std::bind(&Chassis::ChassisRestartCb, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wheel_odometry/global_odometry", 10, std::bind(&Chassis::OdomCb, this, _1));

        /* Initial chassis setup */
        ConfigurePID();
        sleep(1.0);
        InitialConfig();

        // Timers
        moon_tmr_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&Chassis::MoonTimerCb, this));
        moon_tmr_->cancel();
        current_tmr_ = this->create_wall_timer(
            std::chrono::milliseconds(overcur_tmr_duration_), 
            std::bind(&Chassis::CurrentTimerCb, this));
        current_tmr_->cancel();

        // Services
        arm_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/canlink/chassis/arm", std::bind(&Chassis::ArmCb, this, _1, _2, _3));
        
        raw_motors_out_.reserve(4);
    }
    
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Chassis is not connected");
        motors_dvr_status_.connected = false;

        // Publishers
        motors_dvr_status_pub_ = this->create_publisher<usr_msgs::msg::State>(
            "/canlink/chassis/status", 10);
        motors_dvr_status_pub_->publish(motors_dvr_status_);
    }

}

/* ------------------------------------------------------------------------- */
// Services callbacks 
bool Chassis::ArmCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    (void) request_header;
    bool arm_req = request->data;
    uint8_t data_arm[2];
    data_arm[0] = ARM_CMD_ID;

    if ((arm_req == true) && (motors_dvr_status_.armed == false))
    {
        /*
            Arming command
        */
        data_arm[1] = 0x01; 
        if (chassis_cfg_.operation_mode == OPERATION_MODE_NORMAL)
        {
            RCLCPP_INFO(this->get_logger(), "----- Arming chassis -----");
            can_driver_->CANWrite(CHASSIS_ADDRESS, 2, data_arm);
            response->success = true;
            response->message = "Armed";
        }
    }

    if ((arm_req == false) && (motors_dvr_status_.armed == true))
    {
        /*
            Disarming command
        */
        data_arm[1] = 0x00; 
        if (chassis_cfg_.operation_mode == OPERATION_MODE_NORMAL)
        {
            RCLCPP_INFO(this->get_logger(), "----- Disarming Chassis -----");
            can_driver_->CANWrite(CHASSIS_ADDRESS, 2, data_arm);
            response->success = true;
            response->message = "Disarmed";
        }
    }        

    return true;
}

/* ------------------------------------------------------------------------- */
// Subscribers callbacks
void Chassis::ActuatorControlCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    float throttle = msg->twist.linear.x;

    // Throttle limitation depending on the inclination to avoid moon launch position
    if ((throttle > 0.0f) && (pitch_ < 0.0f))
    {
        throttle = throttle > speed_pitch_factor_ ?  speed_pitch_factor_ : throttle;
    }
    else if ((throttle < 0.0f) && (pitch_ > 0.0f))
    {
        throttle = throttle < -speed_pitch_factor_ ?  -speed_pitch_factor_ : throttle;
    }

    // Downhill
    if(moon_view_ == 1)
    {
        controls_.at(0) = 0.0f;  /* Angular velocity - Steering */
        controls_.at(1) = throttle >= 0.0f ? 0.0f : throttle;  /* Linear velocity - Throttle */
    }
    
    // Moonlaunch
    else if (moon_view_ == -1) 
    {
        controls_.at(0) = 0.0f; /* Angular velocity - Steering */
        controls_.at(1) = throttle <= 0.0f ? 0.0f : throttle;  /* Linear velocity - Throttle */
    }

    // Uphill
    else if (moon_view_ == 2)
    {
        throttle = pitch_ <= 0.2 ? 0.0f : throttle;
        controls_.at(0) = 0.0f;  /* Angular velocity - Steering */
        controls_.at(1) = throttle <= 0.0f ? 0.0f : throttle;  /* Linear velocity - Throttle */
    }

    // Normal
    else
    {
        controls_.at(0) = msg->twist.angular.z;  /* Angular velocity - Steering */
        controls_.at(1) = throttle;  /* Linear velocity - Throttle */
    }

    // Send by CAN the motors commands
    SendMotorsCmd();
}

void Chassis::ActuatorReferenceCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    throttle_current_ = msg->twist.linear.x;
}

void Chassis::ChassisConfigCb(const usr_msgs::msg::Configuration::SharedPtr msg)
{
    chassis_cfg_ = *msg;
    SendChassisConfiguration();
    RCLCPP_INFO(this->get_logger(), "----- Chassis Configured -----");
}

void Chassis::MotorsSleepCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        uint8_t data[2] = {ARM_CMD_ID, 0x02};
        can_driver_->CANWrite(CHASSIS_ADDRESS, 2, data);
        sleep(0.5);
    }
    RCLCPP_INFO(this->get_logger(), "----- Motors sleep command -----");
}

void Chassis::ChassisTestCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "----- Chassis configured in Test mode -----");
    chassis_cfg_.operation_mode = OPERATION_MODE_TEST;
    SendChassisConfiguration();
}

void Chassis::ChassisRestartCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "----- Restarting Chassis -----");
    /* Restarting motor status */ 
    for(int i = 0; i < 4; ++i)
    {
        motor_error_[i] = 0;
    }
    motor_error_state_ = 0;
    motors_current_ok_ = true;
    
    current_tmr_->cancel();
    
    /* Restarting chassis board */
    chassis_cfg_.operation_mode = OPERATION_MODE_RESET;
    SendChassisConfiguration();
    chassis_cfg_.operation_mode = OPERATION_MODE_NORMAL;
    sleep(0.5);
}

void Chassis::OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    
    double roll = 0.0f;
    double yaw = 0.0f;

    tf2::Quaternion q(msg->pose.pose.orientation.x, 
                    msg->pose.pose.orientation.y, 
                    msg->pose.pose.orientation.z, 
                    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch_, yaw);

    // Robot is uphill
    if (std::abs(pitch_) > 0.3f && std::abs(pitch_) <= max_allowed_pitch_)
    {
        speed_pitch_factor_ = (1.0 - std::abs(pitch_) * 2.5f);
        speed_pitch_factor_ = speed_pitch_factor_ < 0.0f 
            ? (1.5 * speed_pitch_factor_ - 0.7) : speed_pitch_factor_;
        if (speed_pitch_factor_ < 0.0)
        {
            moon_view_ = 2;
            moon_tmr_->reset();

            /* Deploying warning message to the console */
            if (!moon_first_time_)  /* False */
            {
                moon_first_time_ = true;
                auto info_msg = std::make_unique<usr_msgs::msg::Messages>();
                info_msg->type = usr_msgs::msg::Messages::WARNING;
                info_msg->data = "Deteniendo robot para evitar que se voltee";
                msg_pub_->publish(std::move(info_msg));
            }
        }
    }

    // Robot is downhill
    else if ((pitch_) < -max_allowed_pitch_)
    {
        moon_view_ = 1;
        moon_tmr_->reset();
        /* Deploying warning message to the console */
        if (!moon_first_time_)  /* False */
        {
            moon_first_time_ = true;
            auto info_msg = std::make_unique<usr_msgs::msg::Messages>();
            info_msg->type = usr_msgs::msg::Messages::WARNING;
            info_msg->data = "Deteniendo robot para evitar que se voltee";
            msg_pub_->publish(std::move(info_msg));
        }
    }

    // Moontime position
    else if ((pitch_) > (max_allowed_pitch_))
    {
        moon_view_ = -1;
        moon_tmr_->reset();
        /* Deploying warning message to the console */
        if(!moon_first_time_)  // False
        {
            moon_first_time_ = true;
            auto info_msg = std::make_unique<usr_msgs::msg::Messages>();
            info_msg->type = usr_msgs::msg::Messages::WARNING;
            info_msg->data = "Deteniendo robot para evitar que se voltee";
            msg_pub_->publish(std::move(info_msg));
        }
    }

    // Normal (Flat land)
    else
    {
        speed_pitch_factor_ = 100.0f;
        // Strong inclination
        if (std::abs(pitch_) > 0.15f)
        { 
            // Checking the soft brake flag and Throttle value (Linear velocity)
            if (soft_brake_ && controls_.at(1) == 0.0f)
            {
                if (chassis_cfg_.operation_mode == OPERATION_MODE_NORMAL)
                {
                    soft_brake_ = false;
                }
            }
        }

        // False
        else if (!soft_brake_)
        {
            // Soft brake activation
            if (chassis_cfg_.operation_mode == OPERATION_MODE_NORMAL)
            {
                soft_brake_ = true;
            }
        }
    }
}


/* ------------------------------------------------------------------------- */
// Timers Callbacks 
void Chassis::HeartbeatTimerCb()
{
    auto info_message = std::make_unique<usr_msgs::msg::Messages>();
    info_message->type = usr_msgs::msg::Messages::ERROR;
    info_message->data = "Chassis desconectado!. Robot detenido";
    msg_pub_->publish(std::move(info_message));

    RCLCPP_INFO(this->get_logger(), "------ CAN Link Heartbeat Timeout ------");

    SendChassisConfiguration();
    motors_dvr_status_.connected = false;
    controls_.at(0) = 0.0f;
    controls_.at(1) = 0.0f;

    // On Heartbeat timer, the Robot must be stopped
    for (int i = 0; i < 4; ++i)
    {
        motors_status_.rpm[i] = 0.0f;
    }

    motors_status_.header.stamp = this->now();
    motors_status_pub_->publish(motors_status_);
    motors_dvr_status_pub_->publish(motors_dvr_status_);
}

void Chassis::MoonTimerCb()
{
    moon_view_ = 0;
    moon_first_time_ = false;
    moon_tmr_->cancel();
    RCLCPP_INFO(this->get_logger(), "------ Moon timer stopped, going back to normal ------");
}

void Chassis::CurrentTimerCb()
{
    if(stop_on_motor_anomaly_)
    {
        motors_current_ok_ = false;
    }
    current_timer_started_ = false;
    auto info_message = std::make_unique<usr_msgs::msg::Messages>();
    info_message->type = usr_msgs::msg::Messages::ERROR;
    info_message->data = "Parar cuanto antes: Codigo de error en motores:"
                        + std::to_string(motor_error_[0]) + ", "
                        + std::to_string(motor_error_[1]) + ", "
                        + std::to_string(motor_error_[2]) + ", "
                        + std::to_string(motor_error_[3]);

    msg_pub_->publish(std::move(info_message));
    motors_status_.error_status = {motor_error_[0], motor_error_[1], motor_error_[2], motor_error_[3]};
    RCLCPP_INFO(this->get_logger(), "------ Motor Current Stall ------");
}


/* ------------------------------------------------------------------------- */
// Functions
void Chassis::SendChassisConfiguration()
{
    /*
    - Sends the initial chassis configuration
        + Values defined within *InitialConfig()*
    - Writes the data configuration through the *CANWrite()* (Def at socket_can.cpp)
    */
    uint8_t data[8] = {CONFIGURATION_CMD_ID,
                    chassis_cfg_.wheels_baudrate,
                    chassis_cfg_.battery_status_baudrate,
                    chassis_cfg_.wheel_control_mode,
                    chassis_cfg_.operation_mode,
                    chassis_cfg_.number_battery_cells,
                    chassis_cfg_.motors_model,
                    chassis_cfg_.motors_current_baudrate};
    can_driver_->CANWrite(CHASSIS_ADDRESS, 8, data);
}

void Chassis::ConfigurePID()
{
    /*
        - Configures the Chassis PID variables (For the Wheels)
        - Writes the data through the *CANWrite()* (Def at socket_can.cpp)
    */
    uint8_t int_kp = floor(kp_);
    uint8_t dec_kp = (uint8_t)((kp_ - int_kp) * 255.0);
    uint8_t int_ki = floor(ki_);
    uint8_t dec_ki = (uint8_t)((ki_ - int_ki) * 255.0);
    uint8_t int_kd = floor(kd_);
    uint8_t dec_kd = (uint8_t)((kd_ - int_kd) * 255.0);

    RCLCPP_INFO(this->get_logger(), "kp: %0.4f - int: %i - dec: %i", kp_, int_kp, dec_kp);
    RCLCPP_INFO(this->get_logger(), "ki: %0.4f - int: %i - dec: %i", ki_, int_ki, dec_ki);
    RCLCPP_INFO(this->get_logger(), "kd: %0.4f - int: %i - dec: %i", kd_, int_kd, dec_kd);

    uint8_t data[8] = {PID_CMD_ID, int_kp, dec_kp, int_ki, dec_ki, int_kd, dec_kd, DISABLE_INTEGRAL_LIMIT};
    can_driver_->CANWrite(CHASSIS_ADDRESS, 8, data);
}

void Chassis::InitialConfig()
{
    /*
        - Sets the initial chassis configuration
        - Softbrake activation - Writes the data through the *CANWrite()* 
        (Def at socket_can.cpp)
    */
    chassis_cfg_.wheels_baudrate = WHEELS_BAUDRATE_DEFAULT;
    chassis_cfg_.battery_status_baudrate = BATTERY_STATUS_BAUDRATE_DEFAULT;
    chassis_cfg_.wheel_control_mode = WHEEL_CONTROL_MODE_RPM;
    chassis_cfg_.operation_mode = OPERATION_MODE_NORMAL;
    chassis_cfg_.number_battery_cells = NUMBER_BATTERY_CELLS_DEFAULT;
    chassis_cfg_.motors_model = MOTORS_MODEL_DEFAULT;
    chassis_cfg_.motors_current_baudrate = MOTORS_CURRENT_DEFAULT;
    SendChassisConfiguration();

    // Motors state initialization
    motors_status_.rpm = {0.0, 0.0, 0.0, 0.0};
    motors_status_.current = {0.0, 0.0, 0.0, 0.0};
    motors_status_.error_status = {0, 0, 0, 0};
    test_motors_response_.status = {0, 0, 0, 0, 0, 0};

    // Heartbeat Timer
    heartbeat_tmr_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&Chassis::HeartbeatTimerCb, this));
    heartbeat_tmr_->reset();

    // Soft brake activation
    sleep(0.5);
    uint8_t data[2] = {SOFT_BRAKE_CMD_ID, 0x01};
    can_driver_->CANWrite(CHASSIS_ADDRESS, 2, data);
    sleep(0.5);
}

uint8_t Chassis::RadsToDigital(float rads)
{
    float rpm = rads * 60.0f / (2 * M_PI);
    float digital_value = ((rpm / wheel_max_rpm_) * 255.0f);
    digital_value = digital_value > 255.0f ? 255.0f : digital_value;
    digital_value = digital_value < -255.0f ? -255.0f : digital_value;
    return (uint8_t)std::abs(digital_value);
}

bool Chassis::SendMotorsCmd()
{
    float w_left = (2.0f * controls_.at(1) - controls_.at(0) * robot_length_) / (2.0f * radius_);
    float w_right = (2.0f * controls_.at(1) + controls_.at(0) * robot_length_) / (2.0f * radius_);
    if(motors_current_ok_ == false)
    {
        w_left = 0.0f;
        w_right = 0.0f;
    }

    uint8_t w_left_dig = RadsToDigital(w_left);
    uint8_t w_right_dig = RadsToDigital(w_right);
    uint8_t directions = 0xAA;  // Brake on all motors
   
    // Positive Left wheel speed
    if (w_left > 0)
    {
        if (w_right > 0)
        {
            directions = 0x05;  // Left: CCW - Right: CW - (Forwards)
        }
        else if (w_right < 0)
        {
            directions = 0x00;  // Left: CCW - Right: CCW - (Turn CW)
        }
    }

    // Negative Left wheel speed
    else if (w_left < 0)
    {
        if (w_right > 0)
        {
            directions = 0x55;  // Right: CW - Left: CW - (Turn CCW) 
        }

        else if (w_right < 0)
        {
            directions = 0x50;  // Right: CCW - Left: CW  - (Backwards)
        }
    }

    if ((motors_dvr_status_.connected == true) 
        && (motors_dvr_status_.armed == true) 
        && (chassis_cfg_.operation_mode == OPERATION_MODE_NORMAL))
    {
        /* 
        - This conditional checks three things
            + Chassis connection (True when Can Driver is working)
            + If the Robot is armed
            + Current operation mode for the chassis
        */
        if (((controls_.at(0) == 0.0f) && (controls_.at(1) == 0.0f)))
        {
            /*
            - Controls:
                + Position 0: Angular speed (Steering)
                + Position 1: Linear speed (Throttle)
            */
            if (moving_ == true)
            {
                uint8_t data[1] = {RESET_CMD_ID};
                can_driver_->CANWrite(CHASSIS_ADDRESS, 1, data);
                sleep(0.1);
                moving_ = false;
                return true;
            }
        }

        else
        {
            moving_ = true;
        }

        /* Decceleration checking */
        if((((throttle_prev_ < 0.0f) && (throttle_current_ - throttle_prev_ > 0.1f)) 
            || ((throttle_prev_ > 0.0f) && (throttle_current_ - throttle_prev_ < -0.1f))))
        {
            if (accelerating_ == true)
            {
                accelerating_ = false;
            }
        }

        else
        {
            if (accelerating_ == false)
            {
                accelerating_ = true;
            }      
        }

        /* Checking for Errors in the motors (If error, send 0) */
        uint8_t w_dig_1 = motors_status_.error_status[0] != 0 ? 0 : w_right_dig;
        uint8_t w_dig_2 = motors_status_.error_status[1] != 0 ? 0 : w_right_dig;
        uint8_t w_dig_3 = motors_status_.error_status[2] != 0 ? 0 : w_left_dig;
        uint8_t w_dig_4 = motors_status_.error_status[3] != 0 ? 0 : w_left_dig;
        
        raw_motors_out_ = {w_dig_1, w_dig_2, w_dig_3, w_dig_4};

        // Checking the current chassis wheel control mode 
        if (chassis_cfg_.wheel_control_mode == WHEEL_CONTROL_MODE_RPM)
        {
            uint8_t data_cmd[6] = {
                WHEELS_CONTROL_RPM_ID, 
                directions, 
                w_dig_1, 
                w_dig_2, 
                w_dig_3, 
                w_dig_4};
            
            can_driver_->CANWrite(CHASSIS_ADDRESS, 6, data_cmd);
        }

        else if (chassis_cfg_.wheel_control_mode == WHEEL_CONTROL_MODE_RAW)
        {
            uint8_t data_cmd[6] = {
                MOTORS_CONTROL_RAW_ID, 
                directions, 
                w_dig_1, 
                w_dig_2, 
                w_dig_3, 
                w_dig_4};
            
            can_driver_->CANWrite(CHASSIS_ADDRESS, 6, data_cmd);
        }
    }

    /* Publishing wheels velocities */
    motors_out_msg_.channels = raw_motors_out_;
    motors_out_pub_->publish(motors_out_msg_);
    throttle_prev_ = throttle_current_;
    return true;
}

bool Chassis::GetConnected()
{
    return motors_dvr_status_.connected;
}

void Chassis::SetMotorsCurrent(struct can_frame* frame)
{
    for (int i = 0; i < 2; ++i)
    {
        uint8_t lsbs = frame->data[2 * i + 1];
        uint8_t msbs = frame->data[2 * i + 2];
        uint16_t motor_current = msbs;
        motors_status_.current[i] = (float)((motor_current << 8) | (lsbs));
    }

    uint8_t lsbs = frame->data[5];
    uint8_t msbs = (frame->data[6]) & 0x0F;
    uint16_t motor_current = msbs;
    motors_status_.current[2] = (float)((motor_current << 8) | (lsbs));

    lsbs = frame->data[7];
    msbs = (frame->data[6]) & 0xF0;
    motor_current = msbs >> 4;
    motors_status_.current[3] = (float)((motor_current << 8) | (lsbs));

    // Current units conversion (A)
    for( int i = 0; i < 4; i++)
    {
        motors_status_.current[i] = motors_status_.current[i] * current_slope_ + current_intercept_;
    }
    motor_error_state_ = 0;

    // Checking the current limits and status for each motor
    for(int i = 0; i < 4; i++)
    {
        // Locked wheel
        if((motors_status_.current[i] >= min_current_) 
            && (std::abs(motors_status_.rpm[i]) < 3.0f))
        {
            motor_error_[i] = 2;
            motor_error_state_ -= 1;
            if((motors_status_.current[i] >= max_allowed_current_))
            {
                motor_error_[i] = 3;
            }
        }

        // Overcurrent
        else if((motors_status_.current[i] >= max_allowed_current_))
        {
            motor_error_[i] = 1;
            motor_error_state_ -= 1;
        }

        // No problem at all
        else if((motors_status_.current[i] < max_allowed_current_) 
                && (std::abs(motors_status_.rpm[i]) > 3.0f) 
                && (motors_status_.current[i] >= min_current_))
        {
            motor_error_[i] = 0;
        }
    }

    if (motor_error_state_ == 0)  /* Case: Current is OK */
    {
        current_tmr_->cancel();
        current_timer_started_ = false;
        if(!stop_on_motor_anomaly_)
        {
            motors_current_ok_ = true;
        }
    }

    else  /* Case: Current errors */
    {
        if(!current_timer_started_)
        {
            current_tmr_->reset();
            current_timer_started_ = true;
        }
    }
}

void Chassis::SetErrorStatus(struct can_frame* frame)
{
    for (int i = 1; i < frame->can_dlc; ++i)
    {
        motors_status_.error_status[i - 1] = frame->data[i];
    }
}

void Chassis::PublishMotorStatus(struct can_frame* frame)
{
    if (!motors_dvr_status_.connected)
    {
        RCLCPP_INFO(this->get_logger(), "----- Chassis restarted -----");
        InitialConfig();
    }
    uint8_t dir_b = frame->data[1];
    bool directions_bool[4] = {((dir_b & RIGHT_FRONT_WHEEL) == 0x00), ((dir_b & RIGHT_REAR_WHEEL) == 0x00), ((dir_b & LEFT_REAR_WHEEL) == 0x00), ((dir_b & LEFT_FRONT_WHEEL) == 0x00)};
    int directions[4];
    for (int i = 0; i < 4; ++i)
    {
        directions[i] = directions_bool[i] ? 1 : -1;
    }

    // Motors RPMs 
    for (int i = 0; i < 4; ++i)
    {
        motors_status_.rpm[i] = ((float)directions[i] * (float)frame->data[i + 2]) * wheel_max_rpm_ / 255.0f;
    }

    motors_status_.header.stamp = this->now();
    motors_status_pub_->publish(motors_status_);

    // Publishing current values separately
    if(publish_currents_separately_ == true)
    {
        for(int i = 0; i < 4; ++i)
        {
            auto current_msg = std::make_unique<std_msgs::msg::Float32>();
            current_msg->data = motors_status_.current[i];
            current_pub_[i]->publish(std::move(current_msg));
        }
    }
}

void Chassis::PublishTestReport(struct can_frame* frame)
{
    RCLCPP_INFO(this->get_logger(), "----- Test completed -----");
    for (int i = 1; i < frame->can_dlc; ++i)
    {
        RCLCPP_DEBUG(this->get_logger(), "Test: %i, Result: %x", i, frame->data[i]);
        test_motors_response_.status[i - 1] = frame->data[i];
    }
    test_motors_pub_->publish(test_motors_response_);
    chassis_cfg_.operation_mode = OPERATION_MODE_NORMAL;
    SendChassisConfiguration();
    sleep(0.5);
    moving_ = true;
}

void Chassis::PublishChassisStatus(struct can_frame* frame)
{

    /* Check for Microcontroller status to restart and send the initial configuration */
    if (!motors_dvr_status_.connected)  /* False - Disconnected */
    {
        RCLCPP_INFO(this->get_logger(), "----- Chassis restarted -----"); // Check this message
        InitialConfig();
    }

    if (frame->data[1] == 0x01)  /* Status: Armed */
    {
        motors_dvr_status_.armed = true;
    }

    else if (frame->data[1] == 0x00)  /* Status: Disarmed */
    {
        motors_dvr_status_.armed = false;
    }

    motors_dvr_status_.connected = true;
    motors_dvr_status_pub_->publish(motors_dvr_status_);
    heartbeat_tmr_->reset();
}