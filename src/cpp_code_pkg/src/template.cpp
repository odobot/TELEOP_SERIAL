#include "rclcpp/rclcpp.hpp"


class MyCustomNode: public rclcpp::Node
{
    public:
        MyCustomNode():Node("node_name") //Create a node with name stated
        {
            // output 
        }
    private:
};



int main(int argc, char **argv)
{
    rclcpp::init(argc ,argv);  //initialize ROS 2 communication
    auto node = std::make_shared<MyCustomNode>();// creating a shared ponter to the node
    rclcpp::spin(node);//loop as node is executed
    rclcpp::shutdown(); // shutdown the communication, stop spinning
    return 0;
}  