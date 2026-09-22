#include "quadruped_robot_controller/quadruped_robot_controller.hpp"

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
                RCLCPP_INFO(get_node()->get_logger(),"position:%.4f",joint_position.get().get_value());
                // joint_torque.get().set_value(joint_torque.get().get_value());
            }
            joint_position_command_interface_[0].get().set_value(0.0);
            joint_position_command_interface_[1].get().set_value(0.7);
            joint_position_command_interface_[2].get().set_value(-1.5);

            joint_position_command_interface_[3].get().set_value(0.0);
            joint_position_command_interface_[4].get().set_value(0.7);
            joint_position_command_interface_[5].get().set_value(-1.5);

            joint_position_command_interface_[6].get().set_value(0.0);
            joint_position_command_interface_[7].get().set_value(0.7);
            joint_position_command_interface_[8].get().set_value(-1.5);

            joint_position_command_interface_[9].get().set_value(0.0);
            joint_position_command_interface_[10].get().set_value(0.7);
            joint_position_command_interface_[11].get().set_value(-1.5);


            joint_kp_command_interface_[0].get().set_value(7.0);
            joint_kp_command_interface_[1].get().set_value(7.0);
            joint_kp_command_interface_[2].get().set_value(7.0);

            joint_kp_command_interface_[3].get().set_value(7.0);
            joint_kp_command_interface_[4].get().set_value(7.0);
            joint_kp_command_interface_[5].get().set_value(7.0);


            joint_kp_command_interface_[6].get().set_value(7.0);
            joint_kp_command_interface_[7].get().set_value(7.0);
            joint_kp_command_interface_[8].get().set_value(7.0);

            joint_kp_command_interface_[9].get().set_value(7.0);
            joint_kp_command_interface_[10].get().set_value(7.0);
            joint_kp_command_interface_[11].get().set_value(7.0);
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

