#ifndef QUADRUPED_ROBOT_CONTROLLER_STATE_FIXED_STAND_HPP_
#define QUADRUPED_ROBOT_CONTROLLER_STATE_FIXED_STAND_HPP_

#include <vector>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "robot_leg.hpp"
class StateFixedStand
{
    public:
        explicit StateFixedStand(const std::vector<RobotLeg*> &robot_legs_ptr);
        ~StateFixedStand() = default;
        void enter();
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
        double default_stand_time=5;        //从初始状态到站立时间
        double default_foot_torque=37.7425; //默认足端力矩
    private:
        // Non-owning pointers. The RobotLeg objects are owned by
        // QuadrupedRobotController and outlive this state object.
        RobotLeg * robot_leg_fr{nullptr};
        RobotLeg * robot_leg_fl{nullptr};
        RobotLeg * robot_leg_rr{nullptr};
        RobotLeg * robot_leg_rl{nullptr};

        std::vector<Eigen::Vector3d> joint_start_angles;
        std::vector<Eigen::Vector3d> joint_target_angles;
        //FL默认参数
        const double leg_offset_x=0.193400;   //leg相对于trunk的x offset
        const double leg_offset_y=0.046500;   //leg相对于trunk的y_offset

};

#endif  // QUADRUPED_ROBOT_CONTROLLER_STATE_FIXED_STAND_HPP_
