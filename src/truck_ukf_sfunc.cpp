/*
 * truck_ukf_sfunc.cpp
 * 
 * MATLAB/Simulink S-Function for UKF-based Semitrailer State Estimation
 * 半挂卡车UKF状态估计器的MATLAB S-Function接口
 * 
 * 输入信号（6个端口）：
 *   - Port 0: 前轮转向角 [rad]
 *   - Port 1: 驱动力 [N]
 *   - Port 2: 前轮转速 [rad/s]
 *   - Port 3: 后轮转速 [rad/s]
 *   - Port 4: 航向角速度 [rad/s]
 *   - Port 5: 铰接角 [rad]
 * 
 * 输出信号（6个端口）：
 *   - Port 0: 估计纵向速度 [m/s]
 *   - Port 1: 估计横向速度 [m/s]
 *   - Port 2: 估计航向角速度 [rad/s]
 *   - Port 3: 估计铰接角 [rad]
 *   - Port 4: 估计铰接角速度 [rad/s]
 *   - Port 5: 估计状态协方差迹
 * 
 * 参数（13个）：
 *   P0: 牵引车质量 mt [kg]
 *   P1: 挂车质量 ms [kg]
 *   P2: 牵引车转动惯量 Izt [kg·m²]
 *   P3: 挂车转动惯量 Izs [kg·m²]
 *   P4: 前桥到质心距离 a [m]
 *   P5: 后桥到质心距离 bt [m]
 *   P6: 挂车轴到挂车质心距离 bs [m]
 *   P7: 牵引销到牵引车质心距离 Lh [m]
 *   P8: 前桥侧偏刚度 Caf [N/rad]
 *   P9: 后桥侧偏刚度 Car [N/rad]
 *   P10: 挂车桥侧偏刚度 Cas [N/rad]
 *   P11: 铰接刚度 kh [N·m/rad]
 *   P12: 铰接阻尼 ch [N·m·s/rad]
 */

#define S_FUNCTION_NAME truck_ukf_sfunc
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "../include/semitrailer_dynamics.h"
#include "../include/ukf_estimator.h"
#include <memory>

#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

// S-Function配置常量
#define NUM_INPUTS      6    // 输入端口数量
#define NUM_OUTPUTS     6    // 输出端口数量
#define NUM_PARAMS      13   // 参数数量
#define SAMPLE_TIME     0.001 // 采样时间 1ms

// 使用智能指针管理对象生命周期
struct UKFData {
    std::unique_ptr<SemitrailerDynamics> dynamics;
    std::unique_ptr<UKFEstimator> ukf;
    bool initialized;
    double last_time;
    
    UKFData() : initialized(false), last_time(0.0) {}
};

/*====================*
 * S-Function方法     *
 *====================*/

// mdlInitializeSizes - 初始化输入/输出端口和参数
static void mdlInitializeSizes(SimStruct *S)
{
    // 设置参数数量
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; // 参数数量错误
    }
    
    // 设置输入端口
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    
    for (int i = 0; i < NUM_INPUTS; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 0);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }
    
    // 设置输出端口
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        ssSetOutputPortWidth(S, i, 1);
    }
    
    // 设置采样时间
    ssSetNumSampleTimes(S, 1);
    
    // 分配用户数据存储空间
    ssSetNumPWork(S, 1);  // 存储UKFData指针
    
    // 其他设置
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

// mdlInitializeSampleTimes - 设置采样时间
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

// mdlStart - 初始化运行时数据
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // 创建UKF数据结构
    UKFData* ukf_data = new UKFData();
    ssGetPWork(S)[0] = (void*)ukf_data;
    
    try {
        // 创建动力学模型
        ukf_data->dynamics = std::make_unique<SemitrailerDynamics>();
        
        // 从参数中获取车辆参数
        double mt = mxGetScalar(ssGetSFcnParam(S, 0));    // 牵引车质量
        double ms = mxGetScalar(ssGetSFcnParam(S, 1));    // 挂车质量
        double Izt = mxGetScalar(ssGetSFcnParam(S, 2));   // 牵引车转动惯量
        double Izs = mxGetScalar(ssGetSFcnParam(S, 3));   // 挂车转动惯量
        double a = mxGetScalar(ssGetSFcnParam(S, 4));     // 前桥到质心距离
        double bt = mxGetScalar(ssGetSFcnParam(S, 5));    // 后桥到质心距离
        double bs = mxGetScalar(ssGetSFcnParam(S, 6));    // 挂车轴到质心距离
        double Lh = mxGetScalar(ssGetSFcnParam(S, 7));    // 牵引销到质心距离
        double Caf = mxGetScalar(ssGetSFcnParam(S, 8));   // 前桥侧偏刚度
        double Car = mxGetScalar(ssGetSFcnParam(S, 9));   // 后桥侧偏刚度
        double Cas = mxGetScalar(ssGetSFcnParam(S, 10));  // 挂车桥侧偏刚度
        double kh = mxGetScalar(ssGetSFcnParam(S, 11));   // 铰接刚度
        double ch = mxGetScalar(ssGetSFcnParam(S, 12));   // 铰接阻尼
        
        // 设置车辆参数
        ukf_data->dynamics->setVehicleParameters(mt, ms, Izt, Izs, a, bt, bs, Lh, Caf, Car, Cas);
        ukf_data->dynamics->setHitchParameters(kh, ch);
        
        // 创建UKF估计器
        ukf_data->ukf = std::make_unique<UKFEstimator>();
        ukf_data->ukf->setDynamicsModel(ukf_data->dynamics.get());
        
        // 初始化状态和协方差
        Eigen::VectorXd initial_state(5);
        initial_state << 20.0, 0.0, 0.0, 0.0, 0.0;  // [Vx=20m/s, Vy=0, r=0, psi=0, psi_dot=0]
        
        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Identity(5, 5);
        initial_cov.diagonal() << 1.0, 1.0, 0.1, 0.1, 0.1;  // 初始不确定度
        
        ukf_data->ukf->initialize(initial_state, initial_cov);
        
        // 设置过程噪声
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5);
        Q.diagonal() << 0.1, 0.1, 0.01, 0.01, 0.01;  // 过程噪声方差
        ukf_data->ukf->setProcessNoise(Q);
        
        // 设置测量噪声
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
        R.diagonal() << 0.5, 0.5, 0.01, 0.02;  // 测量噪声方差 [轮速1, 轮速2, 航向角速度, 铰接角]
        ukf_data->ukf->setMeasurementNoise(R);
        
        // 设置UKF参数
        ukf_data->ukf->setUKFParameters(1e-3, 2.0, 0.0);  // alpha, beta, kappa
        
        ukf_data->initialized = true;
        
    } catch (const std::exception& e) {
        // 初始化失败，设置错误信息
        ssSetErrorStatus(S, "UKF initialization failed");
        return;
    }
}

// mdlOutputs - 计算输出
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // 获取UKF数据
    UKFData* ukf_data = (UKFData*)ssGetPWork(S)[0];
    if (!ukf_data || !ukf_data->initialized) {
        return;
    }
    
    // 获取输入信号
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    double delta_f = *uPtrs[0];      // 前轮转向角
    
    uPtrs = ssGetInputPortRealSignalPtrs(S, 1);
    double F_drive = *uPtrs[0];      // 驱动力
    
    uPtrs = ssGetInputPortRealSignalPtrs(S, 2);
    double wheel_speed_f = *uPtrs[0]; // 前轮转速
    
    uPtrs = ssGetInputPortRealSignalPtrs(S, 3);
    double wheel_speed_r = *uPtrs[0]; // 后轮转速
    
    uPtrs = ssGetInputPortRealSignalPtrs(S, 4);
    double yaw_rate_meas = *uPtrs[0]; // 测量的航向角速度
    
    uPtrs = ssGetInputPortRealSignalPtrs(S, 5);
    double hitch_angle_meas = *uPtrs[0]; // 测量的铰接角
    
    // 获取当前时间
    time_T current_time = ssGetT(S);
    double dt = (ukf_data->last_time > 0) ? (current_time - ukf_data->last_time) : SAMPLE_TIME;
    dt = fmax(dt, 1e-6);  // 防止dt过小
    ukf_data->last_time = current_time;
    
    try {
        // 准备控制输入
        Eigen::VectorXd control_input(2);
        control_input << delta_f, F_drive;
        
        // UKF预测步骤
        ukf_data->ukf->predict(control_input, dt);
        
        // 准备测量向量 [前轮线速度, 后轮线速度, 航向角速度, 铰接角]
        Eigen::VectorXd measurement(4);
        double wheel_radius = 0.5;  // 轮胎半径，可以作为参数传入
        measurement << wheel_speed_f * wheel_radius,  // 前轮线速度
                      wheel_speed_r * wheel_radius,  // 后轮线速度  
                      yaw_rate_meas,                 // 航向角速度
                      hitch_angle_meas;             // 铰接角
        
        // UKF更新步骤
        ukf_data->ukf->update(measurement);
        
        // 获取估计状态
        Eigen::VectorXd estimated_state = ukf_data->ukf->getState();
        Eigen::MatrixXd covariance = ukf_data->ukf->getCovariance();
        
        // 设置输出
        real_T *y0 = ssGetOutputPortRealSignal(S, 0);  // 估计纵向速度
        real_T *y1 = ssGetOutputPortRealSignal(S, 1);  // 估计横向速度
        real_T *y2 = ssGetOutputPortRealSignal(S, 2);  // 估计航向角速度
        real_T *y3 = ssGetOutputPortRealSignal(S, 3);  // 估计铰接角
        real_T *y4 = ssGetOutputPortRealSignal(S, 4);  // 估计铰接角速度
        real_T *y5 = ssGetOutputPortRealSignal(S, 5);  // 协方差迹
        
        y0[0] = estimated_state(0);  // Vx
        y1[0] = estimated_state(1);  // Vy
        y2[0] = estimated_state(2);  // r
        y3[0] = estimated_state(3);  // psi
        y4[0] = estimated_state(4);  // psi_dot
        y5[0] = covariance.trace();  // 协方差迹
        
    } catch (const std::exception& e) {
        // 运行时错误，输出默认值
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            real_T *y = ssGetOutputPortRealSignal(S, i);
            y[0] = 0.0;
        }
    }
}

// mdlTerminate - 清理资源
static void mdlTerminate(SimStruct *S)
{
    UKFData* ukf_data = (UKFData*)ssGetPWork(S)[0];
    if (ukf_data) {
        delete ukf_data;
        ssGetPWork(S)[0] = nullptr;
    }
}

/*=============================*
 * 必需的S-Function模板        *
 *=============================*/

#ifdef MATLAB_MEX_FILE    /* 这个定义在MEX编译时才存在 */
#include "simulink.c"     /* MEX-file接口机制 */
#else
#include "cg_sfun.h"      /* 代码生成注册函数 */
#endif
