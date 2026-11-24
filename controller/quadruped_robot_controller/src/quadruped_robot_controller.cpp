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
        //关节控制数据和反馈数据
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
        //imu反馈数据
        imu_name_=auto_declare<std::string>("imu_name", "imu_sensor");  //第一个参数是读取yaml文件中的名字,第二个参数是读取失败的默认参数
        base_name_=auto_declare<std::string>("base_name", "base");
        imu_interface_types_=auto_declare<std::vector<std::string>>("imu_interfaces", imu_interface_types_);
        //足端力数据
        foot_names_=auto_declare<std::vector<std::string>>("feet", foot_names_);
        foot_force_interfaces_=auto_declare<std::vector<std::string>>("foot_force_interfaces", foot_force_interfaces_);

        //路径发布者
        path_pub_=get_node()->create_publisher<nav_msgs::msg::Path>("foot_end",10);
        path_msg.header.stamp = get_node()->get_clock()->now();
        path_msg.header.frame_id = "base";


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
        config.names.reserve(joint_names_.size() * state_interface_types_.size()+imu_interface_types_.size()+foot_force_interfaces_.size()); //预先给std::vector分配内存
        for(const auto& joint:joint_names_)
        {
            for(const auto& state_interface:state_interface_types_)
            {
                config.names.push_back(joint+"/"+state_interface);
            }
        }
        for(const auto& imu_interface:imu_interface_types_)
        {
            config.names.push_back(imu_name_+"/"+imu_interface);
        }
        for(const auto& foot:foot_names_)
        {
            for(const auto& foot_force_interfaces:foot_force_interfaces_)
            {
                config.names.push_back(foot+"/"+foot_force_interfaces);
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
        static bool is_first=true;
        if(is_first==true)
        {
            is_first=false;
            std::vector<Eigen::Vector3d> joint_position_(4);
            joint_position_[0][0] = ctrl_interfaces_.joint_position_state_interface_[0].get().get_optional().value();
            joint_position_[0][1] = ctrl_interfaces_.joint_position_state_interface_[1].get().get_optional().value();
            joint_position_[0][2] = ctrl_interfaces_.joint_position_state_interface_[2].get().get_optional().value();
            joint_position_[1][0] = ctrl_interfaces_.joint_position_state_interface_[3].get().get_optional().value();
            joint_position_[1][1] = ctrl_interfaces_.joint_position_state_interface_[4].get().get_optional().value();
            joint_position_[1][2] = ctrl_interfaces_.joint_position_state_interface_[5].get().get_optional().value();
            joint_position_[2][0] = ctrl_interfaces_.joint_position_state_interface_[6].get().get_optional().value();
            joint_position_[2][1] = ctrl_interfaces_.joint_position_state_interface_[7].get().get_optional().value();
            joint_position_[2][2] = ctrl_interfaces_.joint_position_state_interface_[8].get().get_optional().value();
            joint_position_[3][0] = ctrl_interfaces_.joint_position_state_interface_[9].get().get_optional().value();
            joint_position_[3][1] = ctrl_interfaces_.joint_position_state_interface_[10].get().get_optional().value();
            joint_position_[3][2] = ctrl_interfaces_.joint_position_state_interface_[11].get().get_optional().value();
            for(int i=0;i<12;i++)   //将初始位置设置成当前位置,防止启动后设计KP就开始运动
            {
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(\
                     ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().value());
            }
            state_fixed_stand_.enter(joint_position_);
            for(int i=0;i<12;i++)
            {
                if(i<6)
                {
                    bool flag=ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp);
                    flag=ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd);
                }
                else{
                    bool flag=ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp);
                    flag=ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd);
                }
            }
            
        }

        static int count=0;
        count++;

        if((count>=400)&&(count%10==0))
        {
            RCLCPP_INFO(get_node()->get_logger(),"---------count:%d----------",count/10-100);
            kalman_estimator.update();
            state_fixed_stand_.run();
            std::vector<double> foot_force(3);
            foot_force[0]=ctrl_interfaces_.foot_force_state_interface_[0].get().get_optional().value();
            foot_force[1]=ctrl_interfaces_.foot_force_state_interface_[1].get().get_optional().value();
            foot_force[2]=ctrl_interfaces_.foot_force_state_interface_[2].get().get_optional().value();
            RCLCPP_INFO(get_node()->get_logger(),"FR foot_force:%f,%f,%f",foot_force[0],foot_force[1],foot_force[2]);
            foot_force[0]=ctrl_interfaces_.foot_force_state_interface_[3].get().get_optional().value();
            foot_force[1]=ctrl_interfaces_.foot_force_state_interface_[4].get().get_optional().value();
            foot_force[2]=ctrl_interfaces_.foot_force_state_interface_[5].get().get_optional().value();
            RCLCPP_INFO(get_node()->get_logger(),"FL foot_force:%f,%f,%f",foot_force[0],foot_force[1],foot_force[2]);
        }
        
        return controller_interface::return_type::OK;
    }


    controller_interface::CallbackReturn QuadrupedRobotController::on_activate(const rclcpp_lifecycle::State & )
    {
        // joint_torque_command_interface_.clear();
        // joint_position_command_interface_.clear();
        // joint_velocity_command_interface_.clear();
        // joint_kd_command_interface_.clear();
        // joint_kp_command_interface_.clear();

        // joint_effort_state_interface_.clear();
        // joint_position_state_interface_.clear();
        // joint_velocity_state_interface_.clear();

        // imu_state_interface_.clear();
        // foot_force_state_interface_.clear();
        ctrl_interfaces_.clear();

        ctrl_interfaces_.joint_torque_command_interface_.reserve(joint_names_.size());
        ctrl_interfaces_.joint_position_command_interface_.reserve(joint_names_.size());
        ctrl_interfaces_.joint_velocity_command_interface_.reserve(joint_names_.size());
        ctrl_interfaces_.joint_kd_command_interface_.reserve(joint_names_.size());
        ctrl_interfaces_.joint_kp_command_interface_.reserve(joint_names_.size());

        ctrl_interfaces_.joint_effort_state_interface_.reserve(joint_names_.size());
        ctrl_interfaces_.joint_position_state_interface_.reserve(joint_names_.size());
        ctrl_interfaces_.joint_velocity_state_interface_.reserve(joint_names_.size());

        ctrl_interfaces_.imu_state_interface_.reserve(imu_interface_types_.size());
        ctrl_interfaces_.foot_force_state_interface_.reserve(foot_names_.size()*foot_force_interfaces_.size());
        
        for(auto & interface:command_interfaces_)   //遍历所有的command_interfaces_接口，接口顺序跟yaml配置文件的顺序是一致的
        {
            const std::string & full_name = interface.get_name();
            RCLCPP_WARN(get_node()->get_logger(),"command_name:%s",full_name.c_str());
            for(auto & joint:joint_names_)  
            {
                if(full_name==joint+"/effort")
                {
                    ctrl_interfaces_.joint_torque_command_interface_.push_back(interface);//参考yaml配置文件的顺序
                }
                else if(full_name==joint+"/position")
                {
                    ctrl_interfaces_.joint_position_command_interface_.push_back(interface);
                }
                else if(full_name==joint+"/velocity")
                {
                    ctrl_interfaces_.joint_velocity_command_interface_.push_back(interface);
                }
                else if(full_name==joint+"/kp")
                {
                    ctrl_interfaces_.joint_kp_command_interface_.push_back(interface);
                }
                else if(full_name==joint+"/kd")
                {
                    ctrl_interfaces_.joint_kd_command_interface_.push_back(interface);
                }
            }
        }
        std::string imu_sensor_prefix = "imu_sensor/";
        for(auto & interface:state_interfaces_)
        {
            const std::string & full_name=interface.get_name();
            RCLCPP_WARN(get_node()->get_logger(),"state_name:%s",full_name.c_str());
            //IMU接口数据
            if(interface.get_name().rfind(imu_sensor_prefix, 0) == 0) 
            {
                ctrl_interfaces_.imu_state_interface_.push_back(interface);
            }
            
            else{
                //关节状态数据
                for(auto & joint:joint_names_)  
                {
                    if(full_name==joint+"/effort")
                    {
                        ctrl_interfaces_.joint_effort_state_interface_.push_back(interface);
                    }
                    if(full_name==joint+"/position")
                    {
                        ctrl_interfaces_.joint_position_state_interface_.push_back(interface);
                    }
                    if(full_name==joint+"/velocity")
                    {
                        ctrl_interfaces_.joint_velocity_state_interface_.push_back(interface);
                    }

                }
                //足端力数据
                for(auto &foot:foot_names_)
                {
                    if(interface.get_name().rfind(foot+"/", 0) == 0)
                    {
                        ctrl_interfaces_.foot_force_state_interface_.push_back(interface);
                    }
                }
            }
            
        }

        // for(auto & imu_interface:imu_interface_types_)
        // {
        //     imu_state_interface_.push_back(imu_interface);
        // }
        // for(auto & imu_interface:imu_interface_types_)
        // {
        //     imu_state_interface_.push_back(imu_interface);
        // }
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

