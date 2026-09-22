#ifndef __QUADRUPED_ROBOT_CONTROLLER_HPP__
#define __QUADRUPED_ROBOT_CONTROLLER_HPP__

#include "controller_interface/controller_interface.hpp"

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
        
    };
}
#endif  // __QUADRUPED_ROBOT_CONTROLLER_HPP__