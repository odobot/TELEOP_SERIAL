#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class subscriberNode : public rclcpp::Node
{
public:
    subscriberNode() : Node("sub") // Create a node with name stated
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "sub/topic", 10,
            std::bind(&subscriberNode::sub_callBack, this, std::placeholders::_1));
        // the code above subscribes to the topic sub/topic

        RCLCPP_INFO(this->get_logger(), "WE have subscribed");
    }

private:
    void sub_callBack(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_; // subscriber object 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // initialize ROS 2 communication
    auto node = std::make_shared<subscriberNode>(); // creating a shared ponter to the node
    rclcpp::spin(node);                             // loop as node is executed
    rclcpp::shutdown();                             // shutdown the communication, stop spinning
    return 0;
}