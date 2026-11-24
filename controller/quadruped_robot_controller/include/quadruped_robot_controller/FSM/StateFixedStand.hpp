#ifndef QUADRUPED_ROBOT_CONTROLLER_STATE_FIXED_STAND_HPP_
#define QUADRUPED_ROBOT_CONTROLLER_STATE_FIXED_STAND_HPP_

#include <array>
#include <vector>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "robot_leg.hpp"
#include "CtrlInterfaces.h"


class StateFixedStand
{
    public:

        explicit StateFixedStand(std::array<RobotLeg, 4> &robot_legs,CtrlInterfaces &ctrl_interfaces);
        ~StateFixedStand() = default;
        void enter(std::vector<Eigen::Vector3d> &joint_start);
        void run();
        void exit();

        //body坐标系下的足端初始位置
        std::vector<Eigen::Vector3d>  default_start_leg_bodycoordinate={
            Eigen::Vector3d(0.19344,-0.16259,-0.059062),  //FR
            Eigen::Vector3d(0.19344,0.16259,-0.059062), //FL
            Eigen::Vector3d(-0.19344,-0.17148,-0.036688),//RR
            Eigen::Vector3d(-0.19344,0.17148,-0.036688)   //RL
        };
        //body坐标系下的足端位置
        std::vector<Eigen::Vector3d>  default_target_leg_bodycoordinate={
            Eigen::Vector3d(0.19344,-0.16259,-0.059062),  //FR
            Eigen::Vector3d(0.19344,0.16259,-0.059062), //FL
            Eigen::Vector3d(-0.19344,-0.17148,-0.036688),//RR
            Eigen::Vector3d(-0.19344,0.17148,-0.036688)   //RL
        };
        ////单腿坐标系下的足端起始和期望位置
        std::vector<Eigen::Vector3d>  default_start_leg_legcoordinate;
        std::vector<Eigen::Vector3d>  default_target_leg_legcoordinate;
        double default_stand_height=0.3;    //默认站立高度
        double default_stand_time=10;        //从初始状态到站立时间
        double default_foot_torque=37.7425; //默认足端力矩

        


    private:
        // 引用控制器拥有的腿对象；控制器中的容器先构造、后析构。
        std::array<RobotLeg, 4> &robot_legs_;
                
        CtrlInterfaces &ctrl_interfaces_;//控制接口

        std::array<Eigen::Vector4d,12> A;    //关节3次线性插值系数矩阵
        std::array<double,12>  angle;   //插值过程中的角度
        std::array<double,12>  speed;   //插值过程中的速度
        std::vector<Eigen::Vector3d> joint_start_angles = std::vector<Eigen::Vector3d>(4);
        std::vector<Eigen::Vector3d> joint_target_angles = std::vector<Eigen::Vector3d>(4);
        //FL默认参数
        const double leg_offset_x=0.193400;   //leg相对于trunk的x offset
        const double leg_offset_y=0.046500;   //leg相对于trunk的y_offset

};

#endif  // QUADRUPED_ROBOT_CONTROLLER_STATE_FIXED_STAND_HPP_
