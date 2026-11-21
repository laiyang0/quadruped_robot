#include "StateFixedStand.hpp"
#include "robot_leg.hpp"

#include <stdexcept>

StateFixedStand::StateFixedStand(const std::vector<RobotLeg*> &robot_legs_ptr)
{
    if (robot_legs_ptr.size() != 4)
    {
        throw std::invalid_argument("StateFixedStand requires exactly four robot legs");
    }
    for (const auto * leg : robot_legs_ptr)
    {
        if (leg == nullptr)
        {
            throw std::invalid_argument("StateFixedStand received a null robot leg");
        }
    }

    for(auto leg:robot_legs_ptr)
    {
        leg->logName();
    }
    robot_leg_fr=robot_legs_ptr[0];
    robot_leg_fl=robot_legs_ptr[1];
    robot_leg_rr=robot_legs_ptr[2];
    robot_leg_rl=robot_legs_ptr[3];
    robot_leg_fr->logName();
    robot_leg_fl->logName();
    robot_leg_rr->logName();
    robot_leg_rl->logName();

    default_start_leg_legcoordinate.resize(4);
    default_target_leg_legcoordinate.resize(4);

    default_start_leg_legcoordinate[0]=robot_leg_fr->FootEnd_Body2Leg(default_start_leg_bodycoordinate[0]);
    default_start_leg_legcoordinate[1]=robot_leg_fl->FootEnd_Body2Leg(default_start_leg_bodycoordinate[1]);
    default_start_leg_legcoordinate[2]=robot_leg_rr->FootEnd_Body2Leg(default_start_leg_bodycoordinate[2]);
    default_start_leg_legcoordinate[3]=robot_leg_rl->FootEnd_Body2Leg(default_start_leg_bodycoordinate[3]);
    
    default_target_leg_legcoordinate[0]=robot_leg_fr->FootEnd_Body2Leg(default_target_leg_bodycoordinate[0]);
    default_target_leg_legcoordinate[1]=robot_leg_fl->FootEnd_Body2Leg(default_target_leg_bodycoordinate[1]);
    default_target_leg_legcoordinate[2]=robot_leg_rr->FootEnd_Body2Leg(default_target_leg_bodycoordinate[2]);
    default_target_leg_legcoordinate[3]=robot_leg_rl->FootEnd_Body2Leg(default_target_leg_bodycoordinate[3]);
    //计算期望的站立高度下的单腿坐标系下的足端位置
    for(auto &leg_position:default_target_leg_legcoordinate)
    {
        leg_position[2]=leg_position[2]-default_stand_height;
    }
    for(auto &leg_position:default_start_leg_legcoordinate)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"start:%f,%f,%f",leg_position[0],leg_position[1],leg_position[2]);   
    }
    for(auto &leg_position:default_target_leg_legcoordinate)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"target:%f,%f,%f",leg_position[0],leg_position[1],leg_position[2]);   
    }
}
void StateFixedStand::enter()
{
    joint_start_angles.push_back(robot_leg_fr->backward_kinematic(default_start_leg_legcoordinate[0]));
    joint_start_angles.push_back(robot_leg_fl->backward_kinematic(default_start_leg_legcoordinate[1]));
    joint_start_angles.push_back(robot_leg_rr->backward_kinematic(default_start_leg_legcoordinate[2]));
    joint_start_angles.push_back(robot_leg_rl->backward_kinematic(default_start_leg_legcoordinate[3]));

    joint_target_angles.push_back(robot_leg_fr->backward_kinematic(default_target_leg_legcoordinate[0]));
    joint_target_angles.push_back(robot_leg_fl->backward_kinematic(default_target_leg_legcoordinate[1]));
    joint_target_angles.push_back(robot_leg_rr->backward_kinematic(default_target_leg_legcoordinate[2]));
    joint_target_angles.push_back(robot_leg_rl->backward_kinematic(default_target_leg_legcoordinate[3]));
    RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"enter StateFixedStand");

    for (size_t i = 0; i < joint_start_angles.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"Joint %zu start angles: %f, %f, %f", i, joint_start_angles[i][0], joint_start_angles[i][1], joint_start_angles[i][2]);
    }
    for (size_t i = 0; i < joint_target_angles.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"Joint %zu target angles: %f, %f, %f", i, joint_target_angles[i][0], joint_target_angles[i][1], joint_target_angles[i][2]);
    }
}
void StateFixedStand::run()
{
    RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"run StateFixedStand");
}
void StateFixedStand::exit()
{
    RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"exit StateFixedStand");
}
