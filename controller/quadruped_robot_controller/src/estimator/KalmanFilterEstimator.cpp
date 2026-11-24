#include "KalmanFilterEstimator.hpp"



KalmanFilterEstimator::KalmanFilterEstimator(CtrlInterfaces &ctrl_interface):ctrl_interface_(ctrl_interface)
{
    RCLCPP_INFO(rclcpp::get_logger("KalmanFilterEstimator"),"Init Successful");
}
KalmanFilterEstimator::~KalmanFilterEstimator()
{
    RCLCPP_INFO(rclcpp::get_logger("KalmanFilterEstimator"),"Destory Successful");
}
//状态更新
void KalmanFilterEstimator::update()
{
    std::vector<double> imu_(12);
    imu_[0] = ctrl_interface_.imu_state_interface_[0].get().get_optional().value();
    imu_[1] = ctrl_interface_.imu_state_interface_[1].get().get_optional().value();
    imu_[2] = ctrl_interface_.imu_state_interface_[2].get().get_optional().value();
    imu_[3] = ctrl_interface_.imu_state_interface_[3].get().get_optional().value();
    RCLCPP_INFO(rclcpp::get_logger("KalmanFilterEstimator"),"IMU:%f,%f,%f,%f",imu_[0],imu_[1],imu_[2],imu_[3]);
    //     std::vector<double> imu_(12);
    // imu_[0] = joint_position_state_[0].get().get_value();
    // imu_[1] = joint_position_state_[1].get().get_value();
    // imu_[2] = joint_position_state_[2].get().get_value();
    // imu_[3] = joint_position_state_[3].get().get_value();
    // RCLCPP_INFO(rclcpp::get_logger("KalmanFilterEstimator"),"IMU:%f,%f,%f,%f",imu_[0],imu_[1],imu_[2],imu_[3]);
}