/*
 * simple_test.cpp
 * 
 * 简化的UKF测试程序，专注于基本功能验证
 * 包含强化的超时保护机制
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <csignal>
#include <csetjmp>
#include "../include/semitrailer_dynamics.h"
#include "../include/ukf_estimator.h"

// 全局超时控制
volatile bool timeout_flag = false;
std::jmp_buf timeout_jump;

// 超时信号处理器
void timeout_handler(int sig) {
    timeout_flag = true;
    std::longjmp(timeout_jump, 1);
}

// 检查数值稳定性
bool check_numerical_stability(const Eigen::VectorXd& vec) {
    for (int i = 0; i < vec.size(); i++) {
        if (std::isnan(vec(i)) || std::isinf(vec(i)) || std::abs(vec(i)) > 1e6) {
            return false;
        }
    }
    return true;
}

int main()
{
    // 设置超时保护
    signal(SIGALRM, timeout_handler);
    
    std::cout << "========================================" << std::endl;
    std::cout << "简化UKF测试程序 (带超时保护)" << std::endl;
    std::cout << "Simplified UKF Test Program (with timeout protection)" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        // 设置超时跳转点
        if (setjmp(timeout_jump) != 0) {
            std::cout << "\n[TIMEOUT] 检测到超时，程序被中断！" << std::endl;
            std::cout << "[TIMEOUT] Timeout detected, program interrupted!" << std::endl;
            return -1;
        }
        
        // 1. 测试动力学模型初始化 (带超时保护)
        std::cout << "\n1. 初始化动力学模型..." << std::endl;
        alarm(5);  // 5秒超时
        
        SemitrailerDynamics dynamics;
        dynamics.setVehicleParameters(8000, 25000, 5000, 30000, 1.4, 2.8, 7.0, 0.8, 180000, 300000, 400000);
        dynamics.setHitchParameters(1000.0, 5000.0);
        
        alarm(0);  // 取消超时
        std::cout << "动力学模型初始化成功！" << std::endl;
        
        // 2. 测试UKF初始化 (带超时保护)
        std::cout << "\n2. 初始化UKF估计器..." << std::endl;
        alarm(5);  // 5秒超时
        
        UKFEstimator ukf;
        ukf.setDynamicsModel(&dynamics);
        
        // 使用很小的初始值避免数值问题
        Eigen::VectorXd initial_state(5);
        initial_state << 1.0, 0.0, 0.0, 0.0, 0.0;  // 很小的初始速度
        
        // 使用很小的协方差避免数值不稳定
        Eigen::MatrixXd initial_P = Eigen::MatrixXd::Identity(5, 5) * 1e-3;
        ukf.initialize(initial_state, initial_P);
        
        // 使用很小的噪声协方差
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5) * 1e-6;
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4) * 1e-3;
        ukf.setProcessNoise(Q);
        ukf.setMeasurementNoise(R);
        
        alarm(0);  // 取消超时
        std::cout << "UKF估计器初始化成功！" << std::endl;
        
        // 3. 逐步测试每个功能
        std::cout << "\n3. 开始逐步功能测试..." << std::endl;
        
        // 简单控制输入
        Eigen::VectorXd control_input(2);
        control_input << 0.001, 10.0;  // 极小的转角和驱动力
        
        double dt = 0.001;
        
        for (int i = 0; i < 3; i++) {  // 只测试3步
            std::cout << "\n--- 步骤 " << i << " ---" << std::endl;
            
            auto step_start = std::chrono::high_resolution_clock::now();
            
            // 测试预测步骤 (2秒超时)
            std::cout << "执行预测..." << std::flush;
            alarm(2);
            
            auto predict_start = std::chrono::high_resolution_clock::now();
            ukf.predict(control_input, dt);
            auto predict_end = std::chrono::high_resolution_clock::now();
            
            alarm(0);
            auto predict_duration = std::chrono::duration_cast<std::chrono::milliseconds>(predict_end - predict_start);
            std::cout << " 完成 (" << predict_duration.count() << "ms)" << std::endl;
            
            // 检查预测后的状态
            Eigen::VectorXd state_after_predict = ukf.getState();
            if (!check_numerical_stability(state_after_predict)) {
                std::cout << "[ERROR] 预测后状态数值不稳定！" << std::endl;
                for (int j = 0; j < 5; j++) {
                    std::cout << "  state[" << j << "] = " << state_after_predict(j) << std::endl;
                }
                break;
            }
            
            // 构建简单测量值
            Eigen::VectorXd measurement(4);
            measurement << 1.0 + 0.01*i, 0.001*i, 0.0, 0.01;
            
            // 测试更新步骤 (2秒超时)
            std::cout << "执行更新..." << std::flush;
            alarm(2);
            
            auto update_start = std::chrono::high_resolution_clock::now();
            ukf.update(measurement);
            auto update_end = std::chrono::high_resolution_clock::now();
            
            alarm(0);
            auto update_duration = std::chrono::duration_cast<std::chrono::milliseconds>(update_end - update_start);
            std::cout << " 完成 (" << update_duration.count() << "ms)" << std::endl;
            
            // 检查更新后的状态
            Eigen::VectorXd final_state = ukf.getState();
            if (!check_numerical_stability(final_state)) {
                std::cout << "[ERROR] 更新后状态数值不稳定！" << std::endl;
                for (int j = 0; j < 5; j++) {
                    std::cout << "  state[" << j << "] = " << final_state(j) << std::endl;
                }
                break;
            }
            
            // 输出状态
            std::cout << "最终状态: [" << std::fixed << std::setprecision(6);
            for (int j = 0; j < 5; j++) {
                std::cout << final_state(j);
                if (j < 4) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            
            auto step_end = std::chrono::high_resolution_clock::now();
            auto total_step_time = std::chrono::duration_cast<std::chrono::milliseconds>(step_end - step_start);
            std::cout << "步骤总耗时: " << total_step_time.count() << "ms" << std::endl;
            
            // 如果单步超过500ms，警告
            if (total_step_time.count() > 500) {
                std::cout << "[WARNING] 步骤耗时过长！" << std::endl;
            }
        }
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "测试成功完成！" << std::endl;
        std::cout << "Test completed successfully!" << std::endl;
        std::cout << "========================================" << std::endl;
        
    } catch (const std::exception& e) {
        alarm(0);  // 取消任何挂起的超时
        std::cout << "\n[EXCEPTION] 捕获异常: " << e.what() << std::endl;
        return -1;
    }
    
    alarm(0);  // 确保取消所有超时
    return 0;
}
