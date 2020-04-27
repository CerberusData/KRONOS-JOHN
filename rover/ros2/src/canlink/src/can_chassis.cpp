/*
    - File name: can_chassis.cpp
    - This library defines members and member functions for the CANnode
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#include "canlink/CANChassis.hpp"
 
CANChassis::CANChassis(const rclcpp::NodeOptions & options, CANDriver* can_driver)
: Node("can_chassis", options)
{
    RCLCPP_INFO(this->get_logger(), "CAN Chassis Initializer");
    can_dvr_status_msg_.data = "OK";
    
    /* CAN Driver Instantiation - Open socket */
    try
    {
        can_driver_ = can_driver;
        
        /* Module instantiation */
        battery_ = new Battery(options, can_driver_);
        chassis_ = new Chassis(options, can_driver_);
        lights_ = new Lights(options, can_driver_);

        /* Publishers */
        can_dvr_status_pub_ = this->create_publisher<std_msgs::msg::String>("/canlink/chassis/connection_status", 10);
    }
    /* Nullptr instantiation when CAN not found*/
    catch (const std::system_error &error)
    {
        /* Module instantiation */
        battery_ = new Battery(options, nullptr);
        chassis_ = new Chassis(options, nullptr);
        lights_ = new Lights(options, nullptr);
        
        /* Publishers */
        can_dvr_status_pub_ = this->create_publisher<std_msgs::msg::String>("/canlink/chassis/connection_status", 10);

        char buff[100];
        snprintf(buff, sizeof(buff), "ERROR reading CAN device: %s", error.what());
        std::string errorStr(buff);
        can_dvr_status_msg_.data = errorStr;
    }
    can_dvr_status_pub_->publish(can_dvr_status_msg_);

    /* Start a new thread to read continuously the CAN Socket data */
    read_thread_ = std::thread(&CANChassis::StartCANBusRead, this);  

    /* pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CANChassis::Pub_Cb_, this)); */
}

/*
void CANChassis::Pub_Cb_()
{
    RCLCPP_WARN(this->get_logger(), "Timer Chassis! ");
    struct can_frame* frame;
    // can_dvr_status_pub_->publish(can_dvr_status_msg_); 
    chassis_->PublishChassisStatus(frame);
    chassis_->PublishTestReport(frame);
}
*/

void CANChassis::PublishCANInfo(struct can_frame *frame)
{
    /* 
        - Depending on the data in the frame publishes the corresponding data
    */
    switch (frame->data[0])
    {
        case WHEELS_INFO_ID:
            chassis_->PublishMotorStatus(frame);
            break;
    
        case MOTORS_CURRENT_ID:
            chassis_->SetMotorsCurrent(frame);
            break;
    
        case ERROR_CMD_ID:
        {
            chassis_->SetErrorStatus(frame);
            break;
        }    
    
        case TEST_REPORT_CMD_ID:
        {
            chassis_->PublishTestReport(frame);   
            break;
        }
        case STATUS_CMD_ID:
        {
            if(chassis_->GetConnected())
            {
                can_dvr_status_msg_.data = "OK";
            }
            else if(can_dvr_status_msg_.data.compare("OK") == 0)
            {
                can_dvr_status_msg_.data = "CAN Heartbeat timeout";
            }
            can_dvr_status_pub_->publish(can_dvr_status_msg_);
            chassis_->PublishChassisStatus(frame);
            battery_->PublishBatteryStatus(frame->data[2], frame->data[3], 0.0f);
            break;
        }
        default:
        {
            RCLCPP_INFO(this->get_logger(), "Invalid message");
            break;
        }
    }
}

void CANChassis::StartCANBusRead()
{
    /* 
        - Calls *ReadSocket()* (Def at socket_can.cpp) to read the Socket which returns the memory addres to a frame (Reference)
        - If the CAN Id addres matches the Kiwibot addres it calls *PublishCANInfo()* 
    */
    RCLCPP_INFO(this->get_logger(), "CAN bus thread initialization");
    can_dvr_status_pub_->publish(can_dvr_status_msg_);
    struct can_frame *frame;
    while (true)
    {
        frame = can_driver_->ReadSocket();
        if (frame)
        {
            if ((frame->can_id & 0xFFF) == KIWIBOT_ADDRESS)
            {
                PublishCANInfo(frame);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Impossible to read CAN bus");
        }
    }
}

int main(int argc, char * argv[])
{
    /* 
        - Nodes spinning:
          + CAN Chassis - Main module
          + Chassis - Submodule
        
        Note: Ligths and battery are not spinned as we are calling their functions in
              can_chassis and they are not executing any timmer or callback.
    */

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor; 


    /* CAN Driver creation */
    std::string Mystr = "can0";
    const char *interface_name_ = Mystr.c_str();    
    auto can_dvr_ = new CANDriver(interface_name_);

    /* Filling executor */     
    auto node_CANChassis = std::make_shared<CANChassis>(options, can_dvr_);  /* CAN Chassis */
    executor.add_node(node_CANChassis);
    auto node_chassis = std::make_shared<Chassis>(options, can_dvr_);  /* Chassis */
    executor.add_node(node_chassis);

    RCLCPP_WARN(node_CANChassis->get_logger(), "Init Chassis! ");

    /* Spinning */
    //RCLCPP_WARN(node_CANChassis->get_logger(), "Init Spin! ");
    executor.spin();
    //RCLCPP_WARN(node_CANChassis->get_logger(), "End Spin! ");

    rclcpp::shutdown();
    return 0;
}