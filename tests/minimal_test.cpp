/*
 * minimal_test.cpp
 * 
 * 最小化UKF测试 - 只测试单步运行
 */

#include <iostream>
#include <iomanip>
#include "../include/semitrailer_dynamics.h"
#include "../include/ukf_estimator.h"

bool check_state_reasonable(const Eigen::VectorXd& state) {
    for (int i = 0; i < state.size(); i++) {
        if (std::isnan(state(i)) || std::isinf(state(i))) {
            return false;
        }
        if (std::abs(state(i)) > 50.0) {  // 更严格的检查
            return false;
        }
    }
    return true;
}

int main()
{
    std::cout << "最小化UKF测试" << std::endl;
    
    // 1. 初始化
    SemitrailerDynamics dynamics;
    dynamics.setVehicleParameters(8000, 25000, 5000, 30000, 1.4, 2.8, 7.0, 0.8, 180000, 300000, 400000);
    dynamics.setHitchParameters(1000.0, 5000.0);
    
    UKFEstimator ukf;
    ukf.setDynamicsModel(&dynamics);
    
    // 使用极小的初始值
    Eigen::VectorXd initial_state(5);
    initial_state << 0.1, 0.0, 0.0, 0.0, 0.0;  // 极小初始速度
    
    Eigen::MatrixXd initial_P = Eigen::MatrixXd::Identity(5, 5) * 1e-6;  // 极小协方差
    ukf.initialize(initial_state, initial_P);
    
    // 极小噪声
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5) * 1e-9;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * 1e-6;
    ukf.setProcessNoise(Q);
    ukf.setMeasurementNoise(R);
    
    std::cout << "初始化完成，初始状态: [";
    for (int i = 0; i < 5; i++) {
        std::cout << initial_state(i);
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    // 2. 单步测试
    Eigen::VectorXd control_input(2);
    control_input << 0.0, 0.0;  // 零控制输入
    
    Eigen::VectorXd measurement(4);
    measurement << 0.1, 0.0, 0.0, 0.0;  // 接近初始状态的测量
    
    std::cout << "\n执行预测步骤..." << std::endl;
    ukf.predict(control_input, 0.001);
    
    Eigen::VectorXd state_after_predict = ukf.getState();
    std::cout << "预测后状态: [";
    for (int i = 0; i < 5; i++) {
        std::cout << std::fixed << std::setprecision(6) << state_after_predict(i);
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]";
    
    if (check_state_reasonable(state_after_predict)) {
        std::cout << " ✓" << std::endl;
    } else {
        std::cout << " ✗ (不合理)" << std::endl;
        return -1;
    }
    
    std::cout << "\n执行更新步骤..." << std::endl;
    ukf.update(measurement);
    
    Eigen::VectorXd final_state = ukf.getState();
    std::cout << "更新后状态: [";
    for (int i = 0; i < 5; i++) {
        std::cout << std::fixed << std::setprecision(6) << final_state(i);
        if (i < 4) std::cout << ", ";
    }
    std::cout << "]";
    
    if (check_state_reasonable(final_state)) {
        std::cout << " ✓" << std::endl;
        std::cout << "\n测试通过！" << std::endl;
        return 0;
    } else {
        std::cout << " ✗ (不合理)" << std::endl;
        std::cout << "\n测试失败！" << std::endl;
        return -1;
    }
}
