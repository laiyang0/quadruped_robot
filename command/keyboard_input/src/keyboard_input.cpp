#include "keyboard_input/keyboard_input.hpp"

KeyboardInput::KeyboardInput():Node("keyboard_input_node")
{
    RCLCPP_INFO(this->get_logger(), "Keyboard input node has been started.");
    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
    keyboard_publisher_ = this->create_publisher<std_msgs::msg::String>("keyboard_input", 10); //10表示网络的服务质量qos，也就是最大缓存的消息数
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyboardInput::TimeCallback, this));
}
KeyboardInput::~KeyboardInput()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_); //将键盘状态恢复为原来的状态
}

void KeyboardInput::TimeCallback()
{
    std::string input; 
    if(kbhit())
    {
        input=getchar();
    }
    if(!input.empty())
    {
        auto message = std_msgs::msg::String();
        message.data = input;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        keyboard_publisher_->publish(message);
    }
}
bool KeyboardInput::kbhit() {
    timeval tv = {0L, 0L};          // 1. 设置超时时间为 0 秒 0 微秒（立即返回）
    fd_set fds;                     // 2. 定义文件描述符集合
    FD_ZERO(&fds);                  // 3. 清空集合
    FD_SET(STDIN_FILENO, &fds);     // 4. 将标准输入（文件描述符 0）加入集合
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
    // 5. select 检查是否有可读文件描述符，tv=0 表示不等待，立即返回
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto keyboard_node=std::make_shared<KeyboardInput>();
    rclcpp::spin(keyboard_node);    
    rclcpp::shutdown();
    return 0;
}