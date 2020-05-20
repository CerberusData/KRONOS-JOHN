#include "can_test/can_test.hpp"

CANTest::CANTest(const rclcpp::NodeOptions & options, CANDriver* can_driver)
: Node("can_test", options)
{
    can_dvr_ = can_driver;

    leds_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/test/leds", 10, std::bind(&CANTest::LedsCb, this, _1));
    config_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/test/config", 10, std::bind(&CANTest::ConfigurationCb, this, _1));
    chassis_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/test/chassis", 10, std::bind(&CANTest::ChassisCb, this, _1));

    sent_tmr_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&CANTest::TimerCb, this));

    PIDConfiguration();
    sleep(0.5);
    Configuration();

    read_thread_ = std::thread(&CANTest::StartCANBusRead, this);
}

void CANTest::TimerCb()
{
    // RCLCPP_WARN(this->get_logger(), "Publisher Callback");
    SendChassis();
}

void CANTest::LedsCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Leds Callback");
    SendLeds();
}

void CANTest::ConfigurationCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Configuration Callback");
    PIDConfiguration();
    sleep(0.5);
    Configuration();
}

void CANTest::ChassisCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Chassis Callback");
    chassis_flag_ = !chassis_flag_;
}

void CANTest::Configuration()
{
    RCLCPP_INFO(this->get_logger(), "Initial configuration");

    /*
        Configuration: 45A [01 15 02 00 00 04 00 01] 
    */
    uint8_t data_cfg[8] = {
        CONFIGURATION_CMD_ID,  /* 01 */
        WHEELS_BAUDRATE_DEFAULT,  /* 15 */
        BATTERY_STATUS_BAUDRATE_DEFAULT,  /* 02 */
        WHEEL_CONTROL_MODE_RPM,  /* 00 */
        OPERATION_MODE_NORMAL,  /* 00 */
        NUMBER_BATTERY_CELLS_DEFAULT,  /* 04 */
        MOTORS_MODEL_DEFAULT,  /* 00 */
        MOTORS_CURRENT_DEFAULT  /* 01 */
    };

    can_dvr_->CANWrite(CHASSIS_ADDRESS, 8, data_cfg);
    RCLCPP_INFO(this->get_logger(), "Initial configuration sent");
    sleep(0.5);

    /*
        ARM: 45A [02 01] 
        DISARM: 45A [02 00] 
    */
    uint8_t data_arm[2] = {
        ARM_CMD_ID,  /* 02 */
        0x01
    };

    can_dvr_->CANWrite(CHASSIS_ADDRESS, 2, data_arm);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
    sleep(0.1);

    /*
        SOFTBRAKE: 45A [0D 01] 
    */
    uint8_t data_soft_brake[2] = {
        SOFT_BRAKE_CMD_ID,  /* 0D */ 
        0x01
    };
    can_dvr_->CANWrite(CHASSIS_ADDRESS, 2, data_soft_brake);
    RCLCPP_INFO(this->get_logger(), "Soft Brake sent");
    sleep(0.1);

}

void CANTest::PIDConfiguration()
{
    /*
        PID Configuration: 45A [0C 05 7F 00 59 00 00 00] 
    */
    float kp = 5.5f;
    float ki = 0.35f;
    float kd = 0.0f;
    uint8_t int_kp = floor(kp);
    uint8_t dec_kp = (uint8_t)((kp - int_kp) * 255.0f);
    uint8_t int_ki = floor(ki);
    uint8_t dec_ki = (uint8_t)((ki - int_ki) * 255.0f);
    uint8_t int_kd = floor(kd);
    uint8_t dec_kd = (uint8_t)((kd - int_kd) * 255.0f);

    uint8_t data_PID[8] = {
        PID_CMD_ID, 
        int_kp, 
        dec_kp, 
        int_ki, 
        dec_ki, 
        int_kd, 
        dec_kd, 
        DISABLE_INTEGRAL_LIMIT
    };
    can_dvr_->CANWrite(CHASSIS_ADDRESS, 8, data_PID);
    RCLCPP_INFO(this->get_logger(), "PID configuration sent");
}

void CANTest::SendLeds()
{
    /* 
        LEDS: 45A [0B 01 01]
    */
    uint8_t data_leds[3] = {
        LIGHTS_ADDRESS,  /* 0B */
        LEDS_ON, /* 01 */
        LEDS_ON  /* 01 */
    };
    can_dvr_->CANWrite(CHASSIS_ADDRESS, 3, data_leds);
    RCLCPP_INFO(this->get_logger(), "Lights sent");
}

void CANTest::SendChassis()
{
    if(chassis_flag_ == true)
    {
        /*
            WHEELSCMD: 45A [04 05 25 25 25 25]
        */
        uint8_t w_right = 25;
        uint8_t w_left = 25;
        uint8_t data_wheels[6] = {
            WHEELS_CONTROL_RPM_ID,
            0x05,
            w_right,
            w_right,
            w_left,
            w_left
        };
        can_dvr_->CANWrite(CHASSIS_ADDRESS, 6, data_wheels);
        RCLCPP_INFO(this->get_logger(), "Wheels sent");
    }
}

void CANTest::StartCANBusRead()
{
    RCLCPP_INFO(this->get_logger(), "CAN Thread");
    struct can_frame *frame;
    while (true)
    {
        frame = can_dvr_->ReadSocket();
        RCLCPP_WARN(this->get_logger(), "Not blocked");

        if (frame)
        {
            if ((frame->can_id & 0xFFF) == KIWIBOT_ADDRESS)
            {
                RCLCPP_WARN(this->get_logger(), "ID: '%x'", frame->data[1]);
            }
        }
    }
}