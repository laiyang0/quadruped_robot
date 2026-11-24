#include "StateFixedStand.hpp"
#include "robot_leg.hpp"

StateFixedStand::StateFixedStand(std::array<RobotLeg, 4> &robot_legs,CtrlInterfaces &ctrl_interfaces)
    : robot_legs_(robot_legs), ctrl_interfaces_(ctrl_interfaces)
{
    for(auto &leg:robot_legs_)
    {
        leg.logName();
    }

    default_start_leg_legcoordinate.resize(4);
    default_target_leg_legcoordinate.resize(4);

    for(std::size_t i=0;i<robot_legs_.size();++i)
    {
        default_start_leg_legcoordinate[i]=robot_legs_[i].FootEnd_Body2Leg(default_start_leg_bodycoordinate[i]);
        default_target_leg_legcoordinate[i]=robot_legs_[i].FootEnd_Body2Leg(default_target_leg_bodycoordinate[i]);
    }
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
}void StateFixedStand::enter(std::vector<Eigen::Vector3d> &joint_start)

{
    // joint_start_angles.reserve(4);

    // joint_start_angles[0]=joint_start[0];
    // joint_start_angles[1]=joint_start[1];
    // joint_start_angles[2]=joint_start[2];
    // joint_start_angles[3]=joint_start[3];
    for(std::size_t i=0;i<robot_legs_.size();++i)
    {
        joint_start_angles[i]=robot_legs_[i].backward_kinematic(default_start_leg_legcoordinate[i]);
    }
    for(std::size_t i=0;i<robot_legs_.size();++i)
    {
        joint_target_angles[i]=robot_legs_[i].backward_kinematic(default_target_leg_legcoordinate[i]);
    }
    RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"enter StateFixedStand");

    for (size_t i = 0; i < joint_start_angles.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"Joint %zu start angles: %f, %f, %f", i, joint_start_angles[i][0], joint_start_angles[i][1], joint_start_angles[i][2]);
    }
    for (size_t i = 0; i < joint_target_angles.size(); ++i)
    {
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"Joint %zu target angles: %f, %f, %f", i, joint_target_angles[i][0], joint_target_angles[i][1], joint_target_angles[i][2]);
    }
    for(int i=0;i<joint_start_angles.size();i++)
    {
        // RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"number");
        // A[i][0]=joint_start_angles[i][0];
        // A[i][1]=0;
        // A[i][2]=3.0/joint_target_angles[i][0]-joint_start_angles;
        for(int j=0;j<3;j++)    //计算多项式插值矩阵系数
        {
            A[i*3+j][0]=joint_start_angles[i][j];
            A[i*3+j][1]=0;
            A[i*3+j][2]=3.0*(joint_target_angles[i][j]-joint_start_angles[i][j]);
            A[i*3+j][3]=-2.0*(joint_target_angles[i][j]-joint_start_angles[i][j]);
        }

    }

}
void StateFixedStand::run()
{
    static double time=0;
    time+=0.1;
    if(time<=5.1)
    {
        double s = std::clamp(time/default_stand_time, 0.0, 1.0);
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<3;j++)
            {
                angle[i*3+j]=A[i*3+j][0]+A[i*3+j][1]*s+A[i*3+j][2]*s*s+A[i*3+j][3]*s*s*s;
                speed[i*3+j]=A[i*3+j][1]+2*A[i*3+j][2]*s+3*A[i*3+j][3]*s*s;
            }
        }   
        for(int i=0;i<12;i++)
        {
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(angle[i]);
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(speed[i]);
            // ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(
            // ctrl_interfaces_.joint_effort_state_interface_[i].get().get_optional().value());
        }
        
    }
    std::array<Eigen::Vector3d,4> foot_force; //获取当前时刻足端力
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<3;j++)
        {
            foot_force[i][j]=ctrl_interfaces_.foot_force_state_interface_[i*3+j].get().get_optional().value();
        }
    }
    for(int i=0;i<4;i++)    
    {
        //获取当前足的关节位置
        Eigen::Vector3d joint_pos;
        joint_pos[0]=ctrl_interfaces_.joint_position_state_interface_[i*3+0].get().get_optional().value();
        joint_pos[1]=ctrl_interfaces_.joint_position_state_interface_[i*3+1].get().get_optional().value();
        joint_pos[2]=ctrl_interfaces_.joint_position_state_interface_[i*3+2].get().get_optional().value();
        //获取当前足的关节速度
        Eigen::Vector3d joint_speed;
        joint_speed[0]=ctrl_interfaces_.joint_velocity_state_interface_[i*3+0].get().get_optional().value();
        joint_speed[1]=ctrl_interfaces_.joint_velocity_state_interface_[i*3+1].get().get_optional().value();
        joint_speed[2]=ctrl_interfaces_.joint_velocity_state_interface_[i*3+2].get().get_optional().value();

        robot_legs_[i].setJointPosition(joint_pos);
        robot_legs_[i].setJointVelocity(joint_speed);
        Eigen::Matrix3d jacobin_T=robot_legs_[i].calJacobinT();//计算jacobin的转置
        // Eigen::Vector3d joint_force=-1.0*jacobin_T*foot_force[i];
        Eigen::Vector3d joint_force;
        if(i<=1)
        {
            foot_force[i]={0,0,default_foot_torque};
            joint_force=jacobin_T*foot_force[i];
            joint_force=-joint_force;
        }
        else{
            foot_force[i]={0,0,default_foot_torque+8};
             joint_force=jacobin_T*foot_force[i];
            joint_force=-joint_force;
        }
        RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"joint:force:%f,%f,%f",joint_force[0],joint_force[1],joint_force[2]);
        ctrl_interfaces_.joint_torque_command_interface_[3*i+0].get().set_value(joint_force[0]);
        ctrl_interfaces_.joint_torque_command_interface_[3*i+1].get().set_value(joint_force[1]);
        ctrl_interfaces_.joint_torque_command_interface_[3*i+2].get().set_value(joint_force[2]);
    }   
    


    RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"run StateFixedStand");
}
void StateFixedStand::exit()
{
    RCLCPP_INFO(rclcpp::get_logger("StateFixedStand"),"exit StateFixedStand");
}
