#include "sub_node/sub_node.hpp"

void sub_callback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("sub_node"),"I heard: '%s'",msg->data.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto sub_node=std::make_shared<rclcpp::Node>("sub_node");
    // auto sub=sub_node->create_subscription<std_msgs::msg::String>("keyboard_input",10,
    //     [sub_node](std_msgs::msg::String::SharedPtr msg){
    //         RCLCPP_INFO(sub_node->get_logger(),"I heard: '%s'",msg->data.c_str());
    //     });
    auto sub=sub_node->create_subscription<std_msgs::msg::String>("keyboard_input",10,sub_callback);
    rclcpp::spin(sub_node);
    rclcpp::shutdown();
    return 0;
}