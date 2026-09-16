#ifndef __MASSCENTER_DES_HPP
#define __MASSCENTER_DES_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>

class MasscenterDes final: public rclcpp::Node
{
    public:
        MasscenterDes();
        ~MasscenterDes();
    private:
        void sub_callback(std_msgs::msg::String::SharedPtr msg);
        void timer_callback();
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Path path_msg;  //质心轨迹消息

        Eigen::Vector3d pos_w=Eigen::Vector3d(0.0,0.0,0.0); //当前world系下的质心位置
        Eigen::Vector3d angle_w=Eigen::Vector3d(0.0,0.0,0.0); //当前world系下的质心角度

        Eigen::Vector3d vel_w=Eigen::Vector3d(0.0,0.0,0.0); //当前world系下的质心速度
        Eigen::Vector3d w_w=Eigen::Vector3d(0.0,0.0,0.0);   //当前world系下的角速度 

        Eigen::Vector3d vel_b=Eigen::Vector3d(0.0,0.0,0.0); //当前body系下的质心速度
        Eigen::Vector3d w_b=Eigen::Vector3d(0.0,0.0,0.0);   //当前body系下的角速度
        
        double angle_z_init=0.0;     //当前b与o的初始偏航角  可由IMU获取
        double pos_z_init=1.0;       //当前b与o的初始期望高度

        const double t=0.002;   //递增周期，跟MPC控制周期一致或者500Hz

};


#endif