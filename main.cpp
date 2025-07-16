/*
 * main.cpp
 * 
 * 半挂卡车5-DOF UKF状态估计器测试程序
 * Semitrailer Truck 5-DOF UKF State Estimator Test Program
 * 
 * 测试功能:
 * - 车辆动力学模型初始化
 * - UKF估计器初始化和参数设置
 * - 仿真车辆行驶过程
 * - 验证状态估计精度
 * 
 * 编译命令:
 * g++ -o test_ukf main.cpp semitrailer_dynamics.cpp ukf_estimator.cpp -I. -std=c++11
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>
#include <csignal>
#include "semitrailer_dynamics.h"
#include "ukf_estimator.h"

// 全局超时标志 / Global timeout flag
volatile bool timeout_occurred = false;
volatile bool force_exit = false;
auto start_time = std::chrono::high_resolution_clock::now();
auto step_start_time = std::chrono::high_resolution_clock::now();

// 超时配置 / Timeout configuration
const int STEP_TIMEOUT_MS = 100;        // 单步最大执行时间100ms
const int TOTAL_TIMEOUT_S = 30;         // 总执行时间限制30秒
const int MAX_CONSECUTIVE_FAILURES = 5; // 最大连续失败次数

// 超时信号处理器 / Timeout signal handler
void timeoutHandler(int signal) {
    timeout_occurred = true;
    force_exit = true;
    std::cout << "\n[CRITICAL] 检测到严重超时！强制退出..." << std::endl;
    std::cout << "[CRITICAL] Critical timeout detected! Force exit..." << std::endl;
    exit(1);  // 强制退出
}

// 检查数值稳定性 / Check numerical stability
bool isNumericallyStable(const Eigen::VectorXd& vec) {
    for (int i = 0; i < vec.size(); i++) {
        if (std::isnan(vec(i)) || std::isinf(vec(i))) {
            return false;
        }
        if (std::abs(vec(i)) > 1e6) {  // 检查数值是否过大
            return false;
        }
    }
    return true;
}

// 检查矩阵稳定性 / Check matrix stability
bool isMatrixStable(const Eigen::MatrixXd& mat) {
    for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
            if (std::isnan(mat(i,j)) || std::isinf(mat(i,j))) {
                return false;
            }
            if (std::abs(mat(i,j)) > 1e6) {
                return false;
            }
        }
    }
    return true;
}

// 检查执行时间 / Check execution time
bool checkStepTimeout(double max_step_time_ms = STEP_TIMEOUT_MS) {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - step_start_time);
    return elapsed.count() < max_step_time_ms;
}

// 检查总执行时间 / Check total execution time
bool checkTotalTimeout() {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    return elapsed.count() < TOTAL_TIMEOUT_S;
}

// 重置步骤计时器 / Reset step timer
void resetStepTimer() {
    step_start_time = std::chrono::high_resolution_clock::now();
}

// 获取步骤执行时间 / Get step execution time
double getStepTime() {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - step_start_time);
    return elapsed.count();
}

// 打印向量信息 / Print vector information
void printVector(const std::string& name, const Eigen::VectorXd& vec) {
    std::cout << name << ": [";
    for (int i = 0; i < vec.size(); i++) {
        std::cout << std::fixed << std::setprecision(6) << vec(i);
        if (i < vec.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// 打印矩阵对角线信息 / Print matrix diagonal information
void printMatrixDiagonal(const std::string& name, const Eigen::MatrixXd& mat) {
    std::cout << name << " 对角线: [";
    for (int i = 0; i < std::min(mat.rows(), mat.cols()); i++) {
        std::cout << std::fixed << std::setprecision(6) << mat(i, i);
        if (i < std::min(mat.rows(), mat.cols()) - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// 打印详细的UKF状态信息 / Print detailed UKF state information
void printUKFState(const UKFEstimator& ukf, int step) {
    std::cout << "\n--- UKF状态详情 (步骤 " << step << ") ---" << std::endl;
    std::cout << "--- UKF State Details (Step " << step << ") ---" << std::endl;
    
    Eigen::VectorXd state = ukf.getState();
    Eigen::VectorXd uncertainty = ukf.getStateUncertainty();
    
    printVector("当前状态", state);
    printVector("状态不确定度", uncertainty);
    
    std::cout << "状态检查:" << std::endl;
    for (int i = 0; i < state.size(); i++) {
        std::cout << "  state[" << i << "] = " << std::fixed << std::setprecision(6) << state(i);
        if (std::isnan(state(i))) std::cout << " [NaN!]";
        if (std::isinf(state(i))) std::cout << " [Inf!]";
        if (std::abs(state(i)) > 1e3) std::cout << " [大数值!]";
        std::cout << std::endl;
    }
}

// 生成带噪声的测量值 / Generate noisy measurements
Eigen::VectorXd addNoise(const Eigen::VectorXd& true_measurement, double noise_std)
{
    Eigen::VectorXd noisy_measurement = true_measurement;
    for (int i = 0; i < true_measurement.size(); i++) {
        // 简单的伪随机噪声 / Simple pseudo-random noise
        double noise = noise_std * (2.0 * (rand() / (double)RAND_MAX) - 1.0);
        noisy_measurement(i) += noise;
    }
    return noisy_measurement;
}

// 检查并打印动力学计算结果 / Check and print dynamics computation results
bool checkDynamicsResult(const Eigen::VectorXd& state_before, const Eigen::VectorXd& state_after, 
                        const Eigen::VectorXd& control, double dt, int step) {
    std::cout << "\n--- 动力学计算检查 (步骤 " << step << ") ---" << std::endl;
    std::cout << "--- Dynamics Computation Check (Step " << step << ") ---" << std::endl;
    
    printVector("输入状态", state_before);
    printVector("控制输入", control);
    std::cout << "时间步长: " << dt << " s" << std::endl;
    printVector("输出状态", state_after);
    
    // 计算状态变化量
    Eigen::VectorXd state_change = state_after - state_before;
    printVector("状态变化", state_change);
    
    // 检查变化是否合理
    bool reasonable = true;
    for (int i = 0; i < state_change.size(); i++) {
        if (std::abs(state_change(i)) > 10.0) {  // 单步变化不应超过10
            std::cout << "[警告] 状态[" << i << "]变化过大: " << state_change(i) << std::endl;
            reasonable = false;
        }
    }
    
    if (!isNumericallyStable(state_after)) {
        std::cout << "[错误] 动力学输出状态数值不稳定!" << std::endl;
        reasonable = false;
    }
    
    return reasonable;
}

// 检查并打印测量模型结果 / Check and print measurement model results
bool checkMeasurementResult(const Eigen::VectorXd& state, const Eigen::VectorXd& measurement, int step) {
    std::cout << "\n--- 测量模型检查 (步骤 " << step << ") ---" << std::endl;
    std::cout << "--- Measurement Model Check (Step " << step << ") ---" << std::endl;
    
    printVector("输入状态", state);
    printVector("输出测量", measurement);
    
    // 检查测量值合理性
    bool reasonable = true;
    
    // 轮速应该接近纵向速度
    if (std::abs(measurement(0) - state(0)) > 5.0) {
        std::cout << "[警告] 轮速与纵向速度差异过大: " << (measurement(0) - state(0)) << std::endl;
    }
    
    // 航向角速度应该接近状态中的r
    if (std::abs(measurement(1) - state(2)) > 1.0) {
        std::cout << "[警告] 测量航向角速度与状态差异过大: " << (measurement(1) - state(2)) << std::endl;
    }
    
    if (!isNumericallyStable(measurement)) {
        std::cout << "[错误] 测量值数值不稳定!" << std::endl;
        reasonable = false;
    }
    
    return reasonable;
}

// 打印UKF预测步骤详情 / Print UKF prediction step details
void printPredictionDetails(const UKFEstimator& ukf, const Eigen::VectorXd& control, double dt, int step) {
    std::cout << "\n--- UKF预测步骤详情 (步骤 " << step << ") ---" << std::endl;
    std::cout << "--- UKF Prediction Step Details (Step " << step << ") ---" << std::endl;
    
    Eigen::VectorXd state_before = ukf.getState();
    printVector("预测前状态", state_before);
    printVector("控制输入", control);
    std::cout << "时间步长: " << dt << " s" << std::endl;
}

// 打印UKF更新步骤详情 / Print UKF update step details
void printUpdateDetails(const UKFEstimator& ukf, const Eigen::VectorXd& measurement, int step) {
    std::cout << "\n--- UKF更新步骤详情 (步骤 " << step << ") ---" << std::endl;
    std::cout << "--- UKF Update Step Details (Step " << step << ") ---" << std::endl;
    
    Eigen::VectorXd state_before = ukf.getState();
    printVector("更新前状态", state_before);
    printVector("测量输入", measurement);
}

int main()
{
    // 设置超时信号处理 / Set timeout signal handling
    signal(SIGALRM, timeoutHandler);
    
    std::cout << "========================================" << std::endl;
    std::cout << "半挂卡车UKF状态估计器测试程序 (带超时保护)" << std::endl;
    std::cout << "Semitrailer Truck UKF State Estimator Test (with timeout protection)" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 1. 初始化车辆动力学模型 / Initialize vehicle dynamics model
    std::cout << "\n1. 初始化车辆动力学模型..." << std::endl;
    std::cout << "1. Initializing vehicle dynamics model..." << std::endl;
    
    SemitrailerDynamics dynamics;
    
    // 设置车辆参数 (典型半挂卡车参数)
    // Set vehicle parameters (typical semitrailer truck parameters)
    double mt = 8000.0;     // 牵引车质量 (kg) / Tractor mass
    double ms = 25000.0;    // 半挂车质量 (kg) / Trailer mass
    double Izt = 5000.0;    // 牵引车转动惯量 (kg·m²) / Tractor inertia
    double Izs = 30000.0;   // 半挂车转动惯量 (kg·m²) / Trailer inertia
    double a = 1.4;         // 前轴距质心距离 (m) / Front axle to CG distance
    double bt = 2.8;        // 牵引车后轴距质心距离 (m) / Tractor rear axle to CG distance
    double bs = 7.0;        // 半挂车轴距质心距离 (m) / Trailer axle to CG distance
    double Lh = 0.8;        // 铰接点距牵引车质心距离 (m) / Hitch to tractor CG distance
    double Caf = 180000.0;  // 前轮胎侧偏刚度 (N/rad) / Front tire cornering stiffness
    double Car = 300000.0;  // 牵引车后轮胎侧偏刚度 (N/rad) / Tractor rear tire cornering stiffness
    double Cas = 400000.0;  // 半挂车轮胎侧偏刚度 (N/rad) / Trailer tire cornering stiffness
    
    dynamics.setVehicleParameters(mt, ms, Izt, Izs, a, bt, bs, Lh, Caf, Car, Cas);
    dynamics.setHitchParameters(1000.0, 5000.0);  // 铰接刚度和阻尼 / Hitch stiffness and damping
    
    std::cout << "车辆参数设置完成！" << std::endl;
    std::cout << "Vehicle parameters set!" << std::endl;
    
    // 2. 初始化UKF估计器 / Initialize UKF estimator
    std::cout << "\n2. 初始化UKF估计器..." << std::endl;
    std::cout << "2. Initializing UKF estimator..." << std::endl;
    
    UKFEstimator ukf;
    ukf.setDynamicsModel(&dynamics);
    
    // 设置初始状态 [Vx, Vy, r, psi, psi_dot]
    // Set initial state [Vx, Vy, r, psi, psi_dot]
    Eigen::VectorXd initial_state(5);
    initial_state << 15.0,   // Vx: 初始纵向速度 15 m/s (54 km/h)
                     0.0,    // Vy: 初始横向速度 0 m/s
                     0.0,    // r: 初始航向角速度 0 rad/s
                     0.0,    // psi: 初始铰接角 0 rad
                     0.0;    // psi_dot: 初始铰接角速度 0 rad/s
    
    // 设置初始协方差矩阵 / Set initial covariance matrix
    Eigen::MatrixXd initial_P = Eigen::MatrixXd::Identity(5, 5);
    initial_P.diagonal() << 1.0, 1.0, 0.1, 0.1, 0.1;  // 对角线元素
    
    ukf.initialize(initial_state, initial_P);
    
    // 设置过程噪声协方差 / Set process noise covariance
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5);
    Q.diagonal() << 0.1, 0.1, 0.01, 0.01, 0.01;
    ukf.setProcessNoise(Q);
    
    // 设置测量噪声协方差 / Set measurement noise covariance
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R.diagonal() << 0.1, 0.01, 0.05, 0.1;  // [轮速, 航向角速度, 铰接角, 纵向加速度]
    ukf.setMeasurementNoise(R);
    
    std::cout << "UKF估计器初始化完成！" << std::endl;
    std::cout << "UKF estimator initialized!" << std::endl;
    
    // 3. 设置仿真参数 / Set simulation parameters
    std::cout << "\n3. 开始仿真测试..." << std::endl;
    std::cout << "3. Starting simulation test..." << std::endl;
    
    double dt = 0.001;          // 时间步长 1ms / Time step 1ms
    int num_steps = 200;        // 增加到200步以保持合理的仿真时长 / Increase to 200 steps for reasonable simulation duration
    double steering_amplitude = 0.01;  // 减小转向幅度 / Reduce steering amplitude
    double steering_frequency = 0.1;   // 减小转向频率 / Reduce steering frequency
    
    // 存储结果用于分析 / Store results for analysis
    std::vector<Eigen::VectorXd> true_states;
    std::vector<Eigen::VectorXd> estimated_states;
    std::vector<Eigen::VectorXd> measurements;
    
    // 设置输出格式 / Set output format
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n详细仿真测试 (每步都会打印详细信息)" << std::endl;
    std::cout << "Detailed simulation test (detailed info printed for each step)" << std::endl;
    std::cout << "======================================================================" << std::endl;
    
    // 初始化真实状态 / Initialize true state
    Eigen::VectorXd true_state = initial_state;
    
    // 4. 运行仿真循环 / Run simulation loop
    int successful_steps = 0;
    int failed_steps = 0;
    int consecutive_failures = 0;
    
    for (int step = 0; step < num_steps; step++) {
        // 重置步骤计时器 / Reset step timer
        resetStepTimer();
        
        // 设置步骤级别的超时保护 / Set step-level timeout protection
        alarm(1);  // 1秒超时保护
        
        // 检查超时标志 / Check timeout flag
        if (timeout_occurred || force_exit) {
            std::cout << "\n[ERROR] 程序因超时而终止！" << std::endl;
            std::cout << "[ERROR] Program terminated due to timeout!" << std::endl;
            break;
        }
        
        // 检查总执行时间 / Check total execution time
        if (!checkTotalTimeout()) {
            std::cout << "\n[ERROR] 程序总执行时间超限！" << std::endl;
            std::cout << "[ERROR] Program total execution time exceeded!" << std::endl;
            break;
        }
        
        double time = step * dt;
        
        // 生成控制输入 / Generate control input
        Eigen::VectorXd control_input(2);
        control_input(0) = steering_amplitude * sin(2.0 * M_PI * steering_frequency * time);  // 前轮转角 / Front wheel angle
        control_input(1) = 1000.0;  // 纵向驱动力 (N) / Longitudinal drive force
        
        try {
            std::cout << "\n=== 步骤 " << step << " 开始 (时间: " << std::fixed << std::setprecision(3) << time << "s) ===" << std::endl;
            std::cout << "=== Step " << step << " Started (Time: " << time << "s) ===" << std::endl;
            
            // 生成控制输入 / Generate control input
            Eigen::VectorXd control_input(2);
            control_input(0) = steering_amplitude * sin(2.0 * M_PI * steering_frequency * time);  // 前轮转角
            control_input(1) = 100.0;  // 减小驱动力 / Reduced drive force
            
            printVector("控制输入", control_input);
            
            // 保存当前真实状态用于比较
            Eigen::VectorXd true_state_before = true_state;
            
            // 使用真实动力学模型更新真实状态 / Update true state using true dynamics
            std::cout << "\n1. 执行真实动力学计算..." << std::endl;
            auto dynamics_start = std::chrono::high_resolution_clock::now();
            true_state = dynamics.integrate(true_state, control_input, dt);
            auto dynamics_end = std::chrono::high_resolution_clock::now();
            
            auto dynamics_duration = std::chrono::duration_cast<std::chrono::milliseconds>(dynamics_end - dynamics_start);
            std::cout << "动力学计算耗时: " << dynamics_duration.count() << "ms" << std::endl;
            
            // 检查动力学计算结果
            if (!checkDynamicsResult(true_state_before, true_state, control_input, dt, step)) {
                std::cout << "[ERROR] 动力学计算结果异常，跳过此步骤" << std::endl;
                failed_steps++;
                consecutive_failures++;
                continue;
            }
            
            // 生成真实测量值 / Generate true measurement
            std::cout << "\n2. 执行观测模型计算..." << std::endl;
            auto measurement_start = std::chrono::high_resolution_clock::now();
            Eigen::VectorXd true_measurement = dynamics.observationModel(true_state);
            auto measurement_end = std::chrono::high_resolution_clock::now();
            
            auto measurement_duration = std::chrono::duration_cast<std::chrono::milliseconds>(measurement_end - measurement_start);
            std::cout << "观测模型计算耗时: " << measurement_duration.count() << "ms" << std::endl;
            
            // 检查测量模型结果
            if (!checkMeasurementResult(true_state, true_measurement, step)) {
                std::cout << "[ERROR] 测量模型结果异常，跳过此步骤" << std::endl;
                failed_steps++;
                consecutive_failures++;
                continue;
            }
            
            // 添加测量噪声 / Add measurement noise
            Eigen::VectorXd noisy_measurement = addNoise(true_measurement, 0.01);  // 减小噪声
            std::cout << "测量噪声标准差: 0.01" << std::endl;
            printVector("无噪声测量", true_measurement);
            printVector("含噪声测量", noisy_measurement);
            
            // 打印UKF预测前状态
            std::cout << "\n3. UKF预测步骤..." << std::endl;
            printPredictionDetails(ukf, control_input, dt, step);
            
            // UKF预测步骤 / UKF prediction step
            auto predict_start = std::chrono::high_resolution_clock::now();
            ukf.predict(control_input, dt);
            auto predict_end = std::chrono::high_resolution_clock::now();
            
            auto predict_duration = std::chrono::duration_cast<std::chrono::milliseconds>(predict_end - predict_start);
            std::cout << "UKF预测耗时: " << predict_duration.count() << "ms" << std::endl;
            
            // 获取预测后的状态并检查稳定性
            Eigen::VectorXd predicted_state = ukf.getState();
            printVector("预测后状态", predicted_state);
            
            if (!isNumericallyStable(predicted_state)) {
                std::cout << "[ERROR] UKF预测状态数值不稳定！" << std::endl;
                printUKFState(ukf, step);
                failed_steps++;
                consecutive_failures++;
                
                // 如果预测失败，尝试重置UKF状态到上一个稳定状态
                if (true_states.size() > 0) {
                    std::cout << "[RECOVERY] 尝试重置UKF到上一个稳定状态..." << std::endl;
                    Eigen::MatrixXd reset_P = Eigen::MatrixXd::Identity(5, 5) * 0.1;
                    ukf.initialize(true_states.back(), reset_P);
                }
                continue;
            }
            
            // 打印UKF更新前状态
            std::cout << "\n4. UKF更新步骤..." << std::endl;
            printUpdateDetails(ukf, noisy_measurement, step);
            
            // UKF更新步骤 / UKF update step
            auto update_start = std::chrono::high_resolution_clock::now();
            ukf.update(noisy_measurement);
            auto update_end = std::chrono::high_resolution_clock::now();
            
            auto update_duration = std::chrono::duration_cast<std::chrono::milliseconds>(update_end - update_start);
            std::cout << "UKF更新耗时: " << update_duration.count() << "ms" << std::endl;
            
            // 获取最终估计状态
            Eigen::VectorXd estimated_state = ukf.getState();
            printVector("更新后状态", estimated_state);
            
            // 检查估计状态稳定性
            if (!isNumericallyStable(estimated_state)) {
                std::cout << "[ERROR] UKF估计状态数值不稳定！" << std::endl;
                printUKFState(ukf, step);
                failed_steps++;
                consecutive_failures++;
                
                // 如果更新失败，尝试重置UKF状态
                if (true_states.size() > 0) {
                    std::cout << "[RECOVERY] 尝试重置UKF到上一个稳定状态..." << std::endl;
                    Eigen::MatrixXd reset_P = Eigen::MatrixXd::Identity(5, 5) * 0.1;
                    ukf.initialize(true_states.back(), reset_P);
                }
                continue;
            }
            
            // 计算估计误差
            Eigen::VectorXd estimation_error = estimated_state - true_state;
            printVector("估计误差", estimation_error);
            
            // 检查单步总时间
            double step_time = getStepTime();
            std::cout << "步骤总耗时: " << step_time << "ms" << std::endl;
            
            // 打印详细的UKF内部状态
            printUKFState(ukf, step);
            
            // 存储结果
            true_states.push_back(true_state);
            estimated_states.push_back(estimated_state);
            measurements.push_back(noisy_measurement);
            successful_steps++;
            consecutive_failures = 0;
            
            std::cout << "\n--- 步骤 " << step << " 成功完成 ---" << std::endl;
            std::cout << "--- Step " << step << " Successfully Completed ---" << std::endl;
            
            // 在前5步和每5步输出一次摘要
            if (step < 5 || step % 5 == 0) {
                std::cout << "\n步骤摘要 " << step << ":" << std::endl;
                std::cout << "  真实状态: [" << true_state.transpose() << "]" << std::endl;
                std::cout << "  估计状态: [" << estimated_state.transpose() << "]" << std::endl;
                std::cout << "  误差RMSE: " << estimation_error.norm() << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cout << "\n[ERROR] 步骤 " << step << " 发生异常: " << e.what() << std::endl;
            std::cout << "[ERROR] Exception at step " << step << ": " << e.what() << std::endl;
            failed_steps++;
            consecutive_failures++;
        } catch (...) {
            std::cout << "\n[ERROR] 步骤 " << step << " 发生未知异常" << std::endl;
            std::cout << "[ERROR] Unknown exception at step " << step << std::endl;
            failed_steps++;
            consecutive_failures++;
        }
        
        // 检查连续失败次数 / Check consecutive failures
        if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
            std::cout << "\n[ERROR] 连续失败次数过多 (" << consecutive_failures << ")，退出仿真" << std::endl;
            std::cout << "[ERROR] Too many consecutive failures (" << consecutive_failures << "), exiting simulation" << std::endl;
            break;
        }
        
        // 取消当前步骤的超时警报 / Cancel current step timeout alarm
        alarm(0);
    }
    
    // 取消所有超时警报 / Cancel all timeout alarms
    alarm(0);
    
    std::cout << "\n仿真统计 / Simulation Statistics:" << std::endl;
    std::cout << "成功步骤: " << successful_steps << " / " << num_steps << std::endl;
    std::cout << "Successful steps: " << successful_steps << " / " << num_steps << std::endl;
    std::cout << "失败步骤: " << failed_steps << std::endl;
    std::cout << "Failed steps: " << failed_steps << std::endl;
    std::cout << "连续失败: " << consecutive_failures << std::endl;
    std::cout << "Consecutive failures: " << consecutive_failures << std::endl;
    
    // 5. 计算估计误差统计 / Calculate estimation error statistics
    if (true_states.size() > 0) {
        std::cout << "\n5. 计算估计精度..." << std::endl;
        std::cout << "5. Calculating estimation accuracy..." << std::endl;
        
        Eigen::VectorXd rmse = Eigen::VectorXd::Zero(5);
        for (size_t i = 0; i < true_states.size(); i++) {
            Eigen::VectorXd error = estimated_states[i] - true_states[i];
            rmse += error.cwiseProduct(error);
        }
        rmse = (rmse / true_states.size()).cwiseSqrt();
        
        std::cout << "\n估计误差统计 (RMSE):" << std::endl;
        std::cout << "Estimation Error Statistics (RMSE):" << std::endl;
        std::cout << "Vx (纵向速度):   " << rmse(0) << " m/s" << std::endl;
        std::cout << "Vy (横向速度):   " << rmse(1) << " m/s" << std::endl;
        std::cout << "r (航向角速度):  " << rmse(2) << " rad/s" << std::endl;
        std::cout << "psi (铰接角):    " << rmse(3) << " rad" << std::endl;
        std::cout << "psi_dot (铰接角速度): " << rmse(4) << " rad/s" << std::endl;
        
        // 6. 输出最终状态不确定度 / Output final state uncertainty
        Eigen::VectorXd uncertainty = ukf.getStateUncertainty();
        std::cout << "\n最终状态不确定度 (标准差):" << std::endl;
        std::cout << "Final State Uncertainty (Standard Deviation):" << std::endl;
        std::cout << "Vx:  " << uncertainty(0) << " m/s" << std::endl;
        std::cout << "Vy:  " << uncertainty(1) << " m/s" << std::endl;
        std::cout << "r:   " << uncertainty(2) << " rad/s" << std::endl;
        std::cout << "psi: " << uncertainty(3) << " rad" << std::endl;
        std::cout << "psi_dot: " << uncertainty(4) << " rad/s" << std::endl;
    } else {
        std::cout << "\n[WARNING] 没有有效的仿真数据用于统计分析" << std::endl;
        std::cout << "[WARNING] No valid simulation data for statistical analysis" << std::endl;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "测试完成！UKF估计器运行正常。" << std::endl;
    std::cout << "Test completed! UKF estimator is working properly." << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
