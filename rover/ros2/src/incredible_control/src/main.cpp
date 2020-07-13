#include "incredible_control/definitions.hpp"

Control::Control(const rclcpp::NodeOptions & options) 
: Node("incredible_control", options)
{
    /*
        Define here both publishers publishers

        Example:
            https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/
            pub_name = this->create_publisher<msg_type>("topic_name", queue_size)
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
        
        Example: 
            auto msg = std::make_unique<msg_type>();
            pub_name->publish(msg);

    */
    
    RCLCPP_INFO(this->get_logger(),
        "Publishing from an incredible_control Node!!!");

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Control>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}