#ifndef __QUADRUPED_ROBOT_CONTROLLER_HPP__
#define __QUADRUPED_ROBOT_CONTROLLER_HPP__

#include "controller_interface/controller_interface.hpp"
#include "robot_leg.hpp"
#include "nav_msgs/msg/path.hpp"
#include "StateFixedStand.hpp"
namespace quadruped_robot_controller
{
    class QuadrupedRobotController final: public controller_interface::ControllerInterface
    {
        public:
            QuadrupedRobotController();
            ~QuadrupedRobotController();
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::CallbackReturn on_init() override;
            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            joint_torque_command_interface_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            joint_position_command_interface_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            joint_velocity_command_interface_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            joint_kp_command_interface_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            joint_kd_command_interface_;


            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            joint_effort_state_interface_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            joint_position_state_interface_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            joint_velocity_state_interface_;

            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            imu_state_interface_;

            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
            nav_msgs::msg::Path path_msg;  //质心轨迹消息
        protected:
            std::vector<std::string> joint_names_;
            std::vector<std::string> command_interface_types_;
            std::vector<std::string> state_interface_types_;
            std::unordered_map<
                std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
            command_interface_map_ = {
                    {"effort", &joint_torque_command_interface_},
                    {"position", &joint_position_command_interface_},
                    {"velocity", &joint_velocity_command_interface_},
                    {"kp", &joint_kp_command_interface_},
                    {"kd", &joint_kd_command_interface_}
                    };
            std::unordered_map<
                std::string,std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            state_interface_map_={
                {"effort",&joint_effort_state_interface_},
                {"position",&joint_position_state_interface_},
                {"velocity",&joint_velocity_state_interface_}
            };
        private:
            RobotLeg robot_leg_fr{robot_leg_type::FRONT_RIGHT};//这里不能使用圆括号，编译器会把原来的代码按成员函数声明解析，也就是将这个函数当作声明
            RobotLeg robot_leg_fl{robot_leg_type::FRONT_LEFT};
            RobotLeg robot_leg_rr{robot_leg_type::REAR_RIGHT};
            RobotLeg robot_leg_rl{robot_leg_type::REAR_LEFT};
            //该行代码会创建新的四个leg对象，而不是使用上面创建的leg对象，这样会导致leg对象的状态无法被正确维护。
            //std::shared_ptr<std::vector<RobotLeg>> robot_legs_=std::make_shared<std::vector<RobotLeg>>(std::vector<RobotLeg>{robot_leg_fr,robot_leg_fl,robot_leg_rr,robot_leg_rl});
            //标准容器不允许使用原生引用，因此使用std::reference_wrapper来包装引用类型，确保引用的合法性和可复制性。
            std::vector<std::reference_wrapper<RobotLeg>> robot_legs_{
                robot_leg_fr,
                robot_leg_fl,
                robot_leg_rr,
                robot_leg_rl
            };
            //原始类的指针传递方式
            std::vector<RobotLeg*> robot_legs_ptr{
                &robot_leg_fr,
                &robot_leg_fl,
                &robot_leg_rr,
                &robot_leg_rl
            };
            StateFixedStand state_fixed_stand_{robot_legs_ptr};
        
    };
}
#endif  // __QUADRUPED_ROBOT_CONTROLLER_HPP__
