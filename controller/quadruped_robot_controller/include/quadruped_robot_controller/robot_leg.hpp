#ifndef QUADRUPED_ROBOT_CONTROLLER_ROBOT_LEG_HPP_
#define QUADRUPED_ROBOT_CONTROLLER_ROBOT_LEG_HPP_

#include <array>
#include <cstdint>
#include <cmath> 
#include <string>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

enum class robot_leg_type : std::uint8_t
{
    FRONT_RIGHT = 1,
    FRONT_LEFT = 2,
    REAR_RIGHT = 3,
    REAR_LEFT = 4,
};

//目前将hip,thigh,calf参数固定在类里面
class RobotLeg
{
public:
    explicit RobotLeg(robot_leg_type leg_type);
    ~RobotLeg() = default;
    Eigen::Vector3d forward_kinematic(Eigen::Vector3d &joint_position_);
    Eigen::Vector3d  backward_kinematic(Eigen::Vector3d leg_end_leg_);
    Eigen::Matrix3d calJacobin();
    Eigen::Matrix3d calJacobinT();
    void setJointPosition(Eigen::Vector3d &Joints);
    void setLegPosition(Eigen::Vector3d &Leg);
    void setJointVelocity(Eigen::Vector3d &Joints);
    void setLegVelocity(Eigen::Vector3d &Leg);
    
    Eigen::Vector3d FootEnd_Leg2Body(Eigen::Vector3d &foot_end_leg);
    Eigen::Vector3d FootEnd_Body2Leg(Eigen::Vector3d &foot_end_body);
    void logName();
    Eigen::Vector3d getLegVelocityleg();

private:
    // 腿的身份在构造后保持不变，避免已有数据被重新归属到另一条腿。
    const robot_leg_type leg_type_;
    Eigen::Matrix<double, 4, 1> position;

    const double math_pi=3.1415926;
    //FL默认参数
    const double leg_offset_x=0.193400;   //leg相对于trunk的x offset
    const double leg_offset_y=0.046500;   //leg相对于trunk的y_offset

    const double thigh_offset=0.095500;   //thigh相对于hip关节的offset     l1
    const double thigh_length=0.21300;    //calf相对于thigh关节的offset    l2
    const double calf_length=0.21300;     //calf下的足端长度               l3

    // //关节实际位置(rad)
    Eigen::Vector3d joint_position={0,0,0};
    //关节实际速度(rad/s)
    Eigen::Vector3d joint_velocity={0,0,0};

    //FL:-0.17552,0.039145,0.1308     joint:0.873,2.280,-0.941    --- -0.013903,0.342708,-0.168299
    //FR:0.26872,-0.075799,-0.14003   joint:0.525,0.644,-2.519
    //RL:-0.48924,-0.070185,-0.22557  joint:-0.863,1.378,-0.958
    //RR:0.0497,-0.18317,-0.026756    joint:-0.622,-0.272,-1.809
    // Eigen::Vector3d leg_end_leg={-0.48924+leg_offset_x,-0.070185-leg_offset_y,-0.22557};    //单腿坐标系下的腿部末端位置,RL
    // Eigen::Vector3d leg_end_leg={0.0497+leg_offset_x,-0.18317+leg_offset_y,-0.026756};    //单腿坐标系下的腿部末端位置,RR   对应thigh:-math_pi
    Eigen::Vector3d leg_end_leg={0,0,0};    //单腿坐标系下的腿部末端位置
    Eigen::Vector3d leg_end_body={0,0,0};   //body系下的腿部末端位置

    Eigen::Vector3d leg_end_vel_leg={0,0,0};    //单腿坐标系下的腿部末端速度
    Eigen::Vector3d leg_end_vel_body={0,0,0};   //body系下的腿部末端速度

    std::string leg_prefix_;    //leg类型标识

};

#endif  // QUADRUPED_ROBOT_CONTROLLER_ROBOT_LEG_HPP_
