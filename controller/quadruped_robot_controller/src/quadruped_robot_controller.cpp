#include "quadruped_robot_controller/quadruped_robot_controller.hpp"

// #include <pinocchio/algorithm/rnea.hpp>
namespace quadruped_robot_controller
{
    //final是指该类不会被继承，public是指public继承
    QuadrupedRobotController::QuadrupedRobotController():controller_interface::ControllerInterface() 
    {

    }

    QuadrupedRobotController::~QuadrupedRobotController()
    {

    }
    controller_interface::CallbackReturn QuadrupedRobotController::on_init()
    {
        //用于将yaml配置文件中的joints对应的配置项传进controller
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
        RCLCPP_INFO(get_node()->get_logger(),"joints_size:%ld",joint_names_.size());
        RCLCPP_INFO(get_node()->get_logger(),"command_interfaces_size:%ld",command_interface_types_.size());
        RCLCPP_INFO(get_node()->get_logger(),"state_interfaces_size:%ld",state_interface_types_.size());
        for(const auto& i:joint_names_)
        {
            RCLCPP_INFO(get_node()->get_logger(),"joint_name:%s",i.c_str());
        }
        for(const auto& i:command_interface_types_)
        {
            RCLCPP_INFO(get_node()->get_logger(),"command_interface_name:%s",i.c_str());
        }
        for(const auto& i:state_interface_types_ )
        {
            RCLCPP_INFO(get_node()->get_logger(),"state_interface_name:%s",i.c_str());
        }

        path_pub_=get_node()->create_publisher<nav_msgs::msg::Path>("foot_end",10);
        path_msg.header.stamp = get_node()->get_clock()->now();
        path_msg.header.frame_id = "base";

        state_fixed_stand_.enter();
        return controller_interface::CallbackReturn::SUCCESS;
    }
    
    controller_interface::CallbackReturn QuadrupedRobotController::on_configure(const rclcpp_lifecycle::State & )
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }
    //定义该控制器需要控制hardware_interface中的哪些数据
    controller_interface::InterfaceConfiguration QuadrupedRobotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size() * command_interface_types_.size()); //预先给std::vector分配内存
        for(const auto& joint:joint_names_)
        {
            for(const auto& command_interface:command_interface_types_)
            {
                config.names.push_back(joint+"/"+command_interface);
            }
        }
        // for(const auto& name:config.names)
        // {
        //     RCLCPP_INFO(get_node()->get_logger(),"command_interface:%s",name.c_str());
        // }
        return config;
    }
    //定义该控制器需要读取hardware_interface中的哪些数据
    controller_interface::InterfaceConfiguration QuadrupedRobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size() * state_interface_types_.size()); //预先给std::vector分配内存
        for(const auto& joint:joint_names_)
        {
            for(const auto& state_interface:state_interface_types_)
            {
                config.names.push_back(joint+"/"+state_interface);
            }
        }
        // for(const auto& name:config.names)
        // {
        //     RCLCPP_INFO(get_node()->get_logger(),"state_interface:%s",name.c_str());
        // }
        
        return config;
    }


    controller_interface::return_type QuadrupedRobotController::update(const rclcpp::Time & , const rclcpp::Duration & )
    {
        static int count=0;
        count++;

        if(count%500==0)
        {
            RCLCPP_INFO(get_node()->get_logger(),"---------count:%d----------",count/500);
            for(auto& joint_position:joint_position_state_interface_)
            {
                // RCLCPP_INFO(get_node()->get_logger(),"position:%.4f",joint_position.get().get_value());
                //RCLCPP_INFO(get_node()->get_logger(),"position:%.4f",joint_position.get().get_value());
            }

        //     Eigen::Vector3d leg_fr=robot_leg_fr.forward_kinematic();
        //     Eigen::Vector3d leg_fl=robot_leg_fl.forward_kinematic();
        //     Eigen::Vector3d leg_rr=robot_leg_rr.forward_kinematic();
        //     Eigen::Vector3d leg_rl=robot_leg_rl.forward_kinematic();
        //     RCLCPP_INFO(get_node()->get_logger(),"leg_fr:%f,%f,%f",leg_fr[0],leg_fr[1],leg_fr[2]);
        //     RCLCPP_INFO(get_node()->get_logger(),"leg_fl:%f,%f,%f",leg_fl[0],leg_fl[1],leg_fl[2]);
        //     RCLCPP_INFO(get_node()->get_logger(),"leg_rr:%f,%f,%f",leg_rr[0],leg_rr[1],leg_rr[2]);
        //     RCLCPP_INFO(get_node()->get_logger(),"leg_rl:%f,%f,%f",leg_rl[0],leg_rl[1],leg_rl[2]);

        //     Eigen::Matrix3d jacobin=robot_leg_fl.calJacobin();
        //     RCLCPP_INFO(get_node()->get_logger(),"\njacobin:\n%f,%f,%f\n%f%f%f\n%f%f%f",jacobin(0,0),jacobin(0,1),jacobin(0,2),\
        // jacobin(1,0),jacobin(1,1),jacobin(1,2),jacobin(2,0),jacobin(2,1),jacobin(2,2));
        //     Eigen::Vector3d w(1,0,1);
        //     Eigen::Vector3d v=jacobin*w;
        //     RCLCPP_INFO(get_node()->get_logger(),"fl_foot_v:%f,%f,%f",v(0),v(1),v(2));

            // bool result=joint_position_command_interface_[0].get().set_value(0.0,10);   //0.0:位置命令为0.0,10:发送失败可重发10次
            // result=joint_position_command_interface_[1].get().set_value(0.7,10);   
            // result=joint_position_command_interface_[2].get().set_value(-1.5,10);

            // result=joint_position_command_interface_[3].get().set_value(0.0,10);
            // result=joint_position_command_interface_[4].get().set_value(0.7,10);
            // result=joint_position_command_interface_[5].get().set_value(-1.5,10);

            // result=joint_position_command_interface_[6].get().set_value(0.0,10);
            // result=joint_position_command_interface_[7].get().set_value(0.7,10);
            // result=joint_position_command_interface_[8].get().set_value(-1.5,10);

            // result=joint_position_command_interface_[9].get().set_value(0.0,10);
            // result=joint_position_command_interface_[10].get().set_value(0.7,10);
            // result=joint_position_command_interface_[11].get().set_value(-1.5,10);

            // pose_msg.header = path_msg.header;
            // pose_msg.pose.position.x = leg_position[0];
            // pose_msg.pose.position.y = leg_position[1];
            // pose_msg.pose.position.z = leg_position[2];
            // path_msg.poses.push_back(pose_msg);
            // path_pub_->publish(path_msg);
            // Eigen::Vector3d joint={0,0,-3.1415926/2.0};
            // robot_leg_fl.setJointPosition(joint);
            // Eigen::Vector3d joint_velocity={1,1,1};
            // robot_leg_fl.setJointVelocity(joint_velocity);
            // robot_leg_fl.calJacobin();
            // Eigen::Vector3d leg_velocity=robot_leg_fl.getLegVelocityleg();
            // // RCLCPP_INFO(get_node()->get_logger(),"fl_leg_velocity:%f,%f,%f",leg_velocity(0),leg_velocity(1),leg_velocity(2));
            // Eigen::Vector3d joint_position_state;
            // Eigen::Vector3d leg_end_leg;
            // joint_position_state(0) = joint_position_state_interface_[0].get().get_value();
            // joint_position_state(1) = joint_position_state_interface_[1].get().get_value();
            // joint_position_state(2) = joint_position_state_interface_[2].get().get_value();
            // leg_end_leg=robot_leg_fr.forward_kinematic(joint_position_state);
            // RCLCPP_INFO(get_node()->get_logger(),"fr_leg_position2:%f,%f,%f",leg_end_leg(0),leg_end_leg(1),leg_end_leg(2));
            // joint_position_state(0) = joint_position_state_interface_[3].get().get_value();
            // joint_position_state(1) = joint_position_state_interface_[4].get().get_value();
            // joint_position_state(2) = joint_position_state_interface_[5].get().get_value();
            // leg_end_leg=robot_leg_fl.forward_kinematic(joint_position_state);
            // RCLCPP_INFO(get_node()->get_logger(),"fl_leg_position2:%f,%f,%f",leg_end_leg(0),leg_end_leg(1),leg_end_leg(2));
            // joint_position_state(0) = joint_position_state_interface_[6].get().get_value();
            // joint_position_state(1) = joint_position_state_interface_[7].get().get_value();
            // joint_position_state(2) = joint_position_state_interface_[8].get().get_value();
            // leg_end_leg=robot_leg_rr.forward_kinematic(joint_position_state);
            // RCLCPP_INFO(get_node()->get_logger(),"rr_leg_position2:%f,%f,%f",leg_end_leg(0),leg_end_leg(1),leg_end_leg(2));
            // joint_position_state(0) = joint_position_state_interface_[9].get().get_value();
            // joint_position_state(1) = joint_position_state_interface_[10].get().get_value();
            // joint_position_state(2) = joint_position_state_interface_[11].get().get_value();
            // leg_end_leg=robot_leg_rl.forward_kinematic(joint_position_state);
            // RCLCPP_INFO(get_node()->get_logger(),"rl_leg_position2:%f,%f,%f",leg_end_leg(0),leg_end_leg(1),leg_end_leg(2));
            // Eigen::Vector3d joint={1.004,-0.32,-2.572};
            // robot_leg_fl.setJointPosition(joint);
            // Eigen::Vector3d leg_position=robot_leg_fl.forward_kinematic();
            // robot_leg_fl.setLegPosition(leg_position);
            // RCLCPP_INFO(get_node()->get_logger(),"fl_leg_position:%f,%f,%f",leg_position(0),leg_position(1),leg_position(2));
            // Eigen::Vector3d joint_position=robot_leg_fl.backward_kinematic();
            // RCLCPP_INFO(get_node()->get_logger(),"fl_joint_position:%f,%f,%f",joint_position(0),joint_position(1),joint_position(2));
            // robot_leg_fl.setJointPosition(joint_position);
            // leg_position=robot_leg_fl.forward_kinematic();
            // RCLCPP_INFO(get_node()->get_logger(),"fl_leg_position2:%f,%f,%f",leg_position(0),leg_position(1),leg_position(2));

            // bool result=joint_position_command_interface_[0].get().set_value(0.0,10);   //0.0:位置命令为0.0,10:发送失败可重发10次
            // result=joint_position_command_interface_[1].get().set_value(0.0,10);   
            // result=joint_position_command_interface_[2].get().set_value(-1.78,10);

            // result=joint_position_command_interface_[3].get().set_value(0.0,10);
            // result=joint_position_command_interface_[4].get().set_value(0.0,10);
            // result=joint_position_command_interface_[5].get().set_value(-1.78,10);

            // result=joint_position_command_interface_[6].get().set_value(0.0,10);
            // result=joint_position_command_interface_[7].get().set_value(0.0,10);
            // result=joint_position_command_interface_[8].get().set_value(-1.78,10);

            // result=joint_position_command_interface_[9].get().set_value(0.0,10);
            // result=joint_position_command_interface_[10].get().set_value(0.0,10);
            // result=joint_position_command_interface_[11].get().set_value(-1.78,10);


            // result=joint_kp_command_interface_[0].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[1].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[2].get().set_value(7.0,10);

            // result=joint_kp_command_interface_[3].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[4].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[5].get().set_value(7.0,10);


            // result=joint_kp_command_interface_[6].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[7].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[8].get().set_value(7.0,10);

            // result=joint_kp_command_interface_[9].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[10].get().set_value(7.0,10);
            // result=joint_kp_command_interface_[11].get().set_value(7.0,10);

            // auto it=command_interface_map_.find("effort");
            // if(it!=command_interface_map_.end()&& it->second!=nullptr)
            // {
            //     for(auto & interface: *(it->second))
            //     {
            //                 RCLCPP_INFO(
            //             get_node()->get_logger(),
            //             "effort:%.2f",
            //             interface.get().get_value()
            //         );
            //     }
            // }
        }
        
        return controller_interface::return_type::OK;
    }


    controller_interface::CallbackReturn QuadrupedRobotController::on_activate(const rclcpp_lifecycle::State & )
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();

        joint_torque_command_interface_.reserve(joint_names_.size());
        joint_position_command_interface_.reserve(joint_names_.size());
        joint_velocity_command_interface_.reserve(joint_names_.size());
        joint_kd_command_interface_.reserve(joint_names_.size());
        joint_kp_command_interface_.reserve(joint_names_.size());

        joint_effort_state_interface_.reserve(joint_names_.size());
        joint_position_state_interface_.reserve(joint_names_.size());
        joint_velocity_state_interface_.reserve(joint_names_.size());

        for(auto & interface:command_interfaces_)   //遍历所有的command_interfaces_接口，接口顺序跟yaml配置文件的顺序是一致的
        {
            const std::string & full_name = interface.get_name();
            RCLCPP_WARN(get_node()->get_logger(),"name:%s",full_name.c_str());
            for(auto & joint:joint_names_)  
            {
                if(full_name==joint+"/effort")
                {
                    joint_torque_command_interface_.push_back(interface);//参考yaml配置文件的顺序
                }
                else if(full_name==joint+"/position")
                {
                    joint_position_command_interface_.push_back(interface);
                }
                else if(full_name==joint+"/velocity")
                {
                    joint_velocity_command_interface_.push_back(interface);
                }
                else if(full_name==joint+"/kp")
                {
                    joint_kp_command_interface_.push_back(interface);
                }
                else if(full_name==joint+"/kd")
                {
                    joint_kd_command_interface_.push_back(interface);
                }
            }
        }
        for(auto & interface:state_interfaces_)
        {
            const std::string & full_name=interface.get_name();
            for(auto & joint:joint_names_)  
            {
                if(full_name==joint+"/effort")
                {
                    joint_effort_state_interface_.push_back(interface);
                }
                if(full_name==joint+"/position")
                {
                    joint_position_state_interface_.push_back(interface);
                }
                if(full_name==joint+"/velocity")
                {
                    joint_velocity_state_interface_.push_back(interface);
                }
            }
        }
        // double namber=0.0;
        // for(auto& joint_torque:joint_torque_command_interface_)
        // {
        //     namber++;
        //     joint_torque.get().set_value(namber);
        // }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn QuadrupedRobotController::on_deactivate(const rclcpp_lifecycle::State & )
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_robot_controller::QuadrupedRobotController, controller_interface::ControllerInterface);

