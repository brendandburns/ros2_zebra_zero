#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node_name")
    {
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
