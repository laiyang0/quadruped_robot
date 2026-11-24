#ifndef __QUADRUPED_ROBOT_CONTROLLER_HPP__
#define __QUADRUPED_ROBOT_CONTROLLER_HPP__

#include <array>

#include "controller_interface/controller_interface.hpp"
#include "robot_leg.hpp"
#include "nav_msgs/msg/path.hpp"
#include "StateFixedStand.hpp"
#include "KalmanFilterEstimator.hpp"
#include "CtrlInterfaces.h"
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

            CtrlInterfaces ctrl_interfaces_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            // joint_torque_command_interface_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            // joint_position_command_interface_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            // joint_velocity_command_interface_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            // joint_kp_command_interface_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            // joint_kd_command_interface_;


            // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            // joint_effort_state_interface_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            // joint_position_state_interface_;
            // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            // joint_velocity_state_interface_;

            // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            // imu_state_interface_;

            // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            // foot_force_state_interface_;

            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
            nav_msgs::msg::Path path_msg;  //质心轨迹消息
        protected:

            double kp=6.0;
            double kd=5.0;
            //四足的12个关节数据
            std::vector<std::string> joint_names_;
            std::vector<std::string> command_interface_types_;
            std::vector<std::string> state_interface_types_;
            //IMU的数据
            std::string imu_name_;
            std::string base_name_;
            std::vector<std::string> imu_interface_types_;
            //足端力数据,该力不能用于控制,只能作为验证基准
            std::vector<std::string> foot_names_;
            std::vector<std::string> foot_force_interfaces_;

            std::unordered_map<
                std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
            command_interface_map_ = {
                    {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
                    {"position", &ctrl_interfaces_.joint_position_command_interface_},
                    {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
                    {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
                    {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
                    };
            std::unordered_map<
                std::string,std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            state_interface_map_={
                {"effort",&ctrl_interfaces_.joint_effort_state_interface_},
                {"position",&ctrl_interfaces_.joint_position_state_interface_},
                {"velocity",&ctrl_interfaces_.joint_velocity_state_interface_}
            };
        private:
            // 顺序与关节接口一致：FR、FL、RR、RL。容器直接拥有四条腿的对象。
            std::array<RobotLeg, 4> robot_legs_{
                RobotLeg{robot_leg_type::FRONT_RIGHT},
                RobotLeg{robot_leg_type::FRONT_LEFT},
                RobotLeg{robot_leg_type::REAR_RIGHT},
                RobotLeg{robot_leg_type::REAR_LEFT}
            };
            StateFixedStand state_fixed_stand_{robot_legs_,ctrl_interfaces_};
            KalmanFilterEstimator kalman_estimator{ctrl_interfaces_};
    };
}
#endif  // __QUADRUPED_ROBOT_CONTROLLER_HPP__
