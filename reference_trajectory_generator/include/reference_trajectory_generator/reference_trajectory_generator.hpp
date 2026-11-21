#ifndef __REFERENCE_TRAJECTORY_GENERATOR_HPP__
#define __REFERENCE_TRAJECTORY_GENERATOR_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>

class ReferenceTrajectoryGenerator final: public rclcpp::Node
{
    public:
        ReferenceTrajectoryGenerator();
        ~ReferenceTrajectoryGenerator();
    private:
        void sub_callback(std_msgs::msg::String::SharedPtr msg);
        void timer_callback();
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajector_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        nav_msgs::msg::Path path_msg;  //质心轨迹消息

        Eigen::Vector3d pos_w=Eigen::Vector3d(0.0,0.0,0.0); //当前world系下的质心位置
        Eigen::Vector3d angle_w=Eigen::Vector3d(0.0,0.0,0.0); //当前world系下的质心角度

        Eigen::Vector3d vel_w=Eigen::Vector3d(0.0,0.0,0.0); //当前world系下的质心速度
        Eigen::Vector3d w_w=Eigen::Vector3d(0.0,0.0,0.0);   //当前world系下的角速度 

        Eigen::Vector3d vel_b=Eigen::Vector3d(0.0,0.0,0.0); //当前body系下的质心速度
        Eigen::Vector3d w_b=Eigen::Vector3d(0.0,0.0,0.0);   //当前body系下的角速度
        
        const double init_yaw=0.0;                   //body与world的初始偏航角  可由IMU获取
        const double init_center_height=0.048;       //body与world的初始期望高度
        const double stand_center_height=0.348;      //站立时的期望高度
        const double stand_yaw=0.0;                  //站立时的期望偏航角

        const double dt=0.02;   //递增周期，跟MPC控制周期一致50Hz或者500Hz

};
#endif
