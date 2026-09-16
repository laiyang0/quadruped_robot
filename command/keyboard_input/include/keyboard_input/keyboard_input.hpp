#ifndef __KEYBOARD_INPUT_HPP__
#define __KEYBOARD_INPUT_HPP__

#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include <std_msgs/msg/string.hpp>

class KeyboardInput final:public rclcpp::Node
{
    public:
        KeyboardInput();
        ~KeyboardInput();
    private:
        void TimeCallback();
        bool kbhit();
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keyboard_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        termios old_tio_{}, new_tio_{};

};

#endif