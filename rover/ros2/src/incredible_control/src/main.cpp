#include "incredible_control/definitions.hpp"

Control::Control(const rclcpp::NodeOptions & options, double angle) 
: Node("incredible_control", options), m_angle(angle)
{
    /*
        Define here both publishers publishers
        
        Find more informations:
            https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/

        Example:
            pub_name = this->create_publisher<msg_type>("/control_node/robot_position", 10)
    */

    // Timer definitions - 500 ms
    pub_tmr_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Control::PubTimerCb, this));
    
}

void Control::PubTimerCb()
{
    /*
        Use this timer callback to publish your messages.
        
        Example for message definition: 
            auto msg = std::make_unique<msg_type>();
            msg->data = value;
            pub_name->publish(std::move(msg));
        
        Note: For filling the timestamp you can use this->now()
    */
    
    RCLCPP_WARN(this->get_logger(),
        "Publishing from an incredible_control Node!!!");

    m_pos = CalculatePosition(m_angle);

    RCLCPP_INFO(this->get_logger(),
        "Pos X: %0.2f, Pos Y: %0.2f", m_pos[0], m_pos[1]);

    m_angle += 0.1047;
    m_angle = m_angle >= 2*M_PI ? 0.0 : m_angle;
}

std::vector<double> Control::CalculatePosition(double angle)
{
    double x = cos(angle);
    double y = sin(angle);

    std::vector<double> pos = {x, y};
    
    return pos;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Control>(options, 1.57);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}