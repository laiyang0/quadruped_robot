#include "quadruped_robot_controller/robot_leg.hpp"

#include <stdexcept>

RobotLeg::RobotLeg(robot_leg_type leg_type)
    : leg_type_(leg_type)
{
    switch (leg_type_)
    {
        case robot_leg_type::FRONT_RIGHT:
            leg_prefix_ = "FR";
            break;
        case robot_leg_type::FRONT_LEFT:
            leg_prefix_ = "FL";
            break;
        case robot_leg_type::REAR_RIGHT:
            leg_prefix_ = "RR";
            break;
        case robot_leg_type::REAR_LEFT:
            leg_prefix_ = "RL";
            break;
        default:
            throw std::invalid_argument(
                "Invalid robot leg type: " + std::to_string(static_cast<unsigned int>(leg_type_)));
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotLeg"),"RobotLeg Initial:%s",leg_prefix_.c_str());
    position << 0.0, 0.0, -1.0 * calf_length, 1.0;  //初始足端位置向量赋值

}
//计算得出单腿系下的足端坐标
Eigen::Vector3d  RobotLeg::forward_kinematic(Eigen::Vector3d &joint_position_)
{
    Eigen::Vector3d leg_position_={0,0,0};
    std::int8_t leg_offset_x_polarity=1;
    std::int8_t leg_offset_y_polarity=1;
    double l1=thigh_offset;
    double l2=-thigh_length;
    double l3=-calf_length;
    switch(leg_type_)
    {
        case robot_leg_type::FRONT_RIGHT:
            l1=-thigh_offset;
            leg_offset_x_polarity=1;
            leg_offset_y_polarity=-1;
            break;
        case robot_leg_type::FRONT_LEFT:
            l1=thigh_offset;
            leg_offset_x_polarity=1;
            leg_offset_y_polarity=1;

            break;
        case robot_leg_type::REAR_RIGHT:
            l1=-thigh_offset;
            leg_offset_x_polarity=-1;
            leg_offset_y_polarity=-1;

            break;
        case robot_leg_type::REAR_LEFT:
            l1=thigh_offset;
            leg_offset_x_polarity=-1;
            leg_offset_y_polarity=1;

            break;
    }

    // leg_position[0]=-1*calf_length*std::sin(thigh_joint_position+calf_joint_position)+leg_offset_x_polarity*leg_offset_x;
    // leg_position[1]=calf_length*std::sin(hip_joint_position)*std::cos(thigh_joint_position+calf_joint_position)+thigh_length*std::sin(hip_joint_position)+thigh_offset*hip_polarity+leg_offset_y_polarity*leg_offset_y;
    // leg_position[2]=-1*calf_length*std::cos(hip_joint_position)*std::cos(thigh_joint_position+calf_joint_position)-thigh_length*std::cos(hip_joint_position);

    leg_position_[0]=l3*std::sin(joint_position_[1]+joint_position_[2])+l2*std::sin(joint_position_[1])+leg_offset_x_polarity*leg_offset_x;
    leg_position_[1]=-l3*std::sin(joint_position_[0])*std::cos(joint_position_[1]+joint_position_[2])+l1*std::cos(joint_position_[0])-l2*std::cos(joint_position_[1])*std::sin(joint_position_[0])+leg_offset_y_polarity*leg_offset_y;
    leg_position_[2]=l3*std::cos(joint_position_[0])*std::cos(joint_position_[1]+joint_position_[2])+l1*std::sin(joint_position_[0])+l2*std::cos(joint_position_[0])*std::cos(joint_position_[1]);
    // leg_end_x=leg_position_[0];
    // leg_end_leg[1]=leg_position_[1];
    // leg_end_leg[2]=leg_position_[2];
    return leg_position_;
}
//根据单腿系下的足端坐标求逆运动学解算，求出关节值
Eigen::Vector3d  RobotLeg::backward_kinematic(Eigen::Vector3d leg_end_leg_)
{
    Eigen::Vector3d joint_position_={0,0,0};
    double l1=thigh_offset;
    double l2=-thigh_length;
    double l3=-calf_length;
    if(leg_type_==robot_leg_type::FRONT_LEFT||leg_type_==robot_leg_type::REAR_LEFT)//左边的腿
    {
        l1=thigh_offset;
    }
    else{
        l1=-thigh_offset;
    }
    const double L=std::sqrt(leg_end_leg_[1]*leg_end_leg_[1]+leg_end_leg_[2]*leg_end_leg_[2]-l1*l1);  //求解hip角时，将坐标投影到yz平面
    const double L_AP=std::sqrt(leg_end_leg_[0]*leg_end_leg_[0]+leg_end_leg_[1]*leg_end_leg_[1]+leg_end_leg_[2]*leg_end_leg_[2]-thigh_offset*thigh_offset);//求解calp角时，不进行坐标投影，利用hip与tight关节垂直
 
    joint_position_[0]= std::atan2((leg_end_leg_[2]*l1+leg_end_leg_[1]*L),(leg_end_leg_[1]*l1-leg_end_leg_[2]*L));      
    joint_position_[2]=std::acos((thigh_length*thigh_length+calf_length*calf_length-L_AP*L_AP)/(2.0*thigh_length*calf_length))-math_pi;
    const double a1=leg_end_leg_[1]*std::sin(joint_position_[0])-leg_end_leg_[2]*std::cos(joint_position_[0]);
    const double a2=leg_end_leg_[0];
    const double m1=l3*std::sin(joint_position_[2]);
    const double m2=l3*std::cos(joint_position_[2])+l2;

    joint_position_[1]=std::atan2(a1*m1+a2*m2,(a2*m1-a1*m2));  
    // RCLCPP_ERROR(rclcpp::get_logger("RobotLeg"),"ik joint over the limit");
    return joint_position_;
}
Eigen::Matrix3d RobotLeg::calJacobin()
{
    Eigen::Matrix3d Jacobin;
    double l1=thigh_offset;
    double l2=-thigh_length;
    double l3=-calf_length;
    if(leg_type_==robot_leg_type::FRONT_LEFT||leg_type_==robot_leg_type::REAR_LEFT)//左边的腿
    {
        l1=thigh_offset;
    }
    else{
        l1=-thigh_offset;
    }
    Jacobin(0,0)=0;
    Jacobin(0,1)=l3*std::cos(joint_position[1]+joint_position[2])+l2*std::cos(joint_position[1]);
    Jacobin(0,2)=l3*std::cos(joint_position[1]+joint_position[2]);

    Jacobin(1,0)=-l1*std::sin(joint_position[0])-l2*std::cos(joint_position[0])*std::cos(joint_position[1])-l3*std::cos(joint_position[0])*std::cos(joint_position[1]+joint_position[2]);
    Jacobin(1,1)=l2*std::sin(joint_position[0])*std::sin(joint_position[1])+l3*std::sin(joint_position[0])*std::sin(joint_position[1]+joint_position[2]);
    Jacobin(1,2)=l3*std::sin(joint_position[0])*std::sin(joint_position[1]+joint_position[2]);

    Jacobin(2,0)=l1*std::cos(joint_position[0])-l2*std::sin(joint_position[0])*std::cos(joint_position[1])-l3*std::sin(joint_position[0])*std::cos(joint_position[1]+joint_position[2]);
    Jacobin(2,1)=-l2*std::cos(joint_position[0])*std::sin(joint_position[1])-l3*std::cos(joint_position[0])*std::sin(joint_position[1]+joint_position[2]);
    Jacobin(2,2)=-l3*std::cos(joint_position[0])*std::sin(joint_position[1]+joint_position[2]); 

    leg_end_vel_leg=Jacobin*joint_velocity;
    return Jacobin;
}
Eigen::Matrix3d RobotLeg::calJacobinT()
{
    Eigen::Matrix3d Jacobin;
    Jacobin=calJacobin().transpose();
    return Jacobin;
}
void RobotLeg::setJointPosition(Eigen::Vector3d &Joints)
{
    joint_position[0]=Joints[0];
    joint_position[1]=Joints[1];
    joint_position[2]=Joints[2];
}
void RobotLeg::setJointVelocity(Eigen::Vector3d &Joints)
{
    joint_velocity[0]=Joints[0];
    joint_velocity[1]=Joints[1];
    joint_velocity[2]=Joints[2];
}
Eigen::Vector3d RobotLeg::getLegVelocityleg()
{
    return leg_end_vel_leg;
}
//设置body系下的足端坐标，并自动转换单腿系下的足端坐标
void RobotLeg::setLegPosition(Eigen::Vector3d &Leg)
{
    leg_end_body[0]=Leg[0];
    leg_end_body[1]=Leg[1];
    leg_end_body[2]=Leg[2];
    std::int8_t leg_offset_x_polarity=1;
    std::int8_t leg_offset_y_polarity=1;
    switch(leg_type_)
    {
        case robot_leg_type::FRONT_RIGHT:
            leg_offset_x_polarity=1;
            leg_offset_y_polarity=-1;
            break;
        case robot_leg_type::FRONT_LEFT:
            leg_offset_x_polarity=1;
            leg_offset_y_polarity=1;
            break;
        case robot_leg_type::REAR_RIGHT:
            leg_offset_x_polarity=-1;
            leg_offset_y_polarity=-1;
            break;
        case robot_leg_type::REAR_LEFT:
            leg_offset_x_polarity=-1;
            leg_offset_y_polarity=1;
            break;
    }
    leg_end_leg[0]=leg_end_body[0]-leg_offset_x_polarity*leg_offset_x;
    leg_end_leg[1]=leg_end_body[1]-leg_offset_y_polarity*leg_offset_y;
    leg_end_leg[2]=leg_end_body[2];
}
void RobotLeg::setLegVelocity(Eigen::Vector3d &Leg)
{
    leg_end_vel_body[0]=Leg[0];
    leg_end_vel_body[1]=Leg[1];
    leg_end_vel_body[2]=Leg[2];
    std::int8_t leg_offset_x_polarity=1;
    std::int8_t leg_offset_y_polarity=1;
    switch(leg_type_)
    {
        case robot_leg_type::FRONT_RIGHT:
            leg_offset_x_polarity=1;
            leg_offset_y_polarity=-1;
            break;
        case robot_leg_type::FRONT_LEFT:
            leg_offset_x_polarity=1;
            leg_offset_y_polarity=1;
            break;
        case robot_leg_type::REAR_RIGHT:
            leg_offset_x_polarity=-1;
            leg_offset_y_polarity=-1;
            break;
        case robot_leg_type::REAR_LEFT:
            leg_offset_x_polarity=-1;
            leg_offset_y_polarity=1;
            break;
    }
    leg_end_vel_leg[0]=leg_end_vel_body[0];
    leg_end_vel_leg[1]=leg_end_vel_body[1];
    leg_end_vel_leg[2]=leg_end_vel_body[2];
}


//足底从身体坐标系到单腿坐标系的转换
Eigen::Vector3d RobotLeg::FootEnd_Body2Leg(Eigen::Vector3d &foot_end_body)
{
    Eigen::Vector3d foot_end_leg;
    switch(leg_type_)
    {
        case robot_leg_type::FRONT_RIGHT:
            foot_end_leg[0] = foot_end_body[0] - leg_offset_x;
            foot_end_leg[1] = foot_end_body[1] + leg_offset_y;
            foot_end_leg[2] = foot_end_body[2];
            break;
        case robot_leg_type::FRONT_LEFT:
            foot_end_leg[0] = foot_end_body[0] - leg_offset_x;
            foot_end_leg[1] = foot_end_body[1] - leg_offset_y;
            foot_end_leg[2] = foot_end_body[2];
            break;
        case robot_leg_type::REAR_RIGHT:
            foot_end_leg[0] = foot_end_body[0] + leg_offset_x;
            foot_end_leg[1] = foot_end_body[1] + leg_offset_y;
            foot_end_leg[2] = foot_end_body[2];
            break;
        case robot_leg_type::REAR_LEFT:
            foot_end_leg[0] = foot_end_body[0] + leg_offset_x;
            foot_end_leg[1] = foot_end_body[1] - leg_offset_y;
            foot_end_leg[2] = foot_end_body[2];
            break;
    }
    return foot_end_leg;
}
//足底从单腿坐标系到body坐标系的转换
Eigen::Vector3d RobotLeg::FootEnd_Leg2Body(Eigen::Vector3d &foot_end_leg)
{
    Eigen::Vector3d foot_end_body;
    switch(leg_type_)
    {
        case robot_leg_type::FRONT_RIGHT:
            foot_end_body[0] = foot_end_leg[0] + leg_offset_x;
            foot_end_body[1] = foot_end_leg[1] - leg_offset_y;
            foot_end_body[2] = foot_end_leg[2];
            break;
        case robot_leg_type::FRONT_LEFT:
            foot_end_body[0] = foot_end_leg[0] + leg_offset_x;
            foot_end_body[1] = foot_end_leg[1] + leg_offset_y;
            foot_end_body[2] = foot_end_leg[2];
            break;
        case robot_leg_type::REAR_RIGHT:
            foot_end_body[0] = foot_end_leg[0] - leg_offset_x;
            foot_end_body[1] = foot_end_leg[1] - leg_offset_y;
            foot_end_body[2] = foot_end_leg[2];
            break;
        case robot_leg_type::REAR_LEFT:
            foot_end_body[0] = foot_end_leg[0] - leg_offset_x;
            foot_end_body[1] = foot_end_leg[1] + leg_offset_y;
            foot_end_body[2] = foot_end_leg[2];
            break;
    }
    return foot_end_body;
}
void RobotLeg::logName()
{
    RCLCPP_INFO(rclcpp::get_logger("RobotLeg"),"RobotLeg Name:%s",leg_prefix_.c_str());
}
