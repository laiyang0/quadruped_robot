#ifndef __KALMAN_FILTER_ESTIMATOR_HPP
#define __KALMAN_FILTER_ESTIMATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "CtrlInterfaces.h"
#include <Eigen/Dense>

using Vector18d = Eigen::Matrix<double, 18, 1>;


class KalmanFilterEstimator
{
    public:
        KalmanFilterEstimator(CtrlInterfaces &ctrl_interface);
        ~KalmanFilterEstimator();
        void update();
    private:
        CtrlInterfaces &ctrl_interface_;    //控制接口


        Vector18d x_hat;  //18维状态向量
        Eigen::Vector3d  u;      //3维系统输入
        Eigen::Matrix<double, 28, 1> z;     //观测向量测量值
        Eigen::Matrix<double, 28, 1> z_hat; //观测向量估计值

        Eigen::Matrix<double, 18, 18> A;//状态转移矩阵
        Eigen::Matrix<double, 18, 3> B; //输入矩阵
        Eigen::Matrix<double, 28, 18> H;//观测矩阵




    
};


#endif
