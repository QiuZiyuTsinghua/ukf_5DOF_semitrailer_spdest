/*
 * truck_ukf_sfunc.cpp
 * 
 * Simulink S-Function for UKF-based semitrailer state estimation
 * 
 * 该S-Function为MATLAB/Simulink提供半挂卡车UKF状态估计功能
 * 输入: [delta_f, F_drive, measurements]
 * 输出: [estimated_states, state_uncertainties]
 * 
 * 默认值:
 * - 控制输入: [0.0, 0.0] (无转向, 无驱动力)
 * - 测量输入: [20.0, 0.0, 0.0, 0.0] (20m/s直线行驶)
 * - 初始状态: [20.0, 0.0, 0.0, 0.0, 0.0] (20m/s直线行驶)
 * - 采样时间: 0.01s
 * - 过程噪声: [0.01, 0.01, 0.001, 0.001, 0.001]
 * - 测量噪声: [0.1, 0.01, 0.01, 0.1]
 * 
 * 编译命令:
 * mex -I../include truck_ukf_sfunc.cpp semitrailer_dynamics.cpp ukf_estimator.cpp
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME truck_ukf_sfunc

/*
 * 包含必要的头文件
 */
#include "simstruc.h"
#include "../include/ukf_estimator.h"
#include "../include/semitrailer_dynamics.h"
#include <memory>
#include <cstdint>

/* 避免未使用参数警告的宏 */
#ifndef UNUSED_ARG
#define UNUSED_ARG(x) ((void)(x))
#endif

/*
 * S-Function参数定义
 */
#define NUM_PARAMS          3
#define SAMPLE_TIME_PARAM   0
#define PROCESS_NOISE_PARAM 1
#define MEAS_NOISE_PARAM    2

/*
 * 输入输出端口定义
 */
#define NUM_INPUTS          3
#define CONTROL_INPUT_PORT  0  // [delta_f, F_drive] (2维)
#define MEASUREMENT_PORT    1  // [Vx_meas, r_meas, psi_meas, psi_dot_meas] (4维)
#define INIT_STATE_PORT     2  // [Vx, Vy, r, psi, psi_dot] (5维，仅初始化时使用)

#define NUM_OUTPUTS         2
#define STATE_OUTPUT_PORT   0  // 估计状态 (5维)
#define UNCERT_OUTPUT_PORT  1  // 状态不确定度 (5维)

/*
 * 工作向量定义，用于存储UKF实例
 */
#define UKF_INSTANCE_PTR    0
#define DYNAMICS_INSTANCE_PTR 1
#define INITIALIZED_FLAG_PTR  2

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    设置S-Function的基本属性：输入输出端口数量、参数数量等
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* 设置参数数量 */
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* 参数数量不匹配，返回错误 */
    }

    /* 设置参数不可调 */
    for (int i = 0; i < NUM_PARAMS; i++) {
        ssSetSFcnParamTunable(S, i, 0);
    }

    /* 设置输入端口 */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    
    /* 控制输入端口：[delta_f, F_drive] */
    ssSetInputPortWidth(S, CONTROL_INPUT_PORT, 2);
    ssSetInputPortRequiredContiguous(S, CONTROL_INPUT_PORT, 1);
    ssSetInputPortDirectFeedThrough(S, CONTROL_INPUT_PORT, 1);
    
    /* 测量输入端口：[Vx_meas, r_meas, psi_meas, psi_dot_meas] */
    ssSetInputPortWidth(S, MEASUREMENT_PORT, 4);
    ssSetInputPortRequiredContiguous(S, MEASUREMENT_PORT, 1);
    ssSetInputPortDirectFeedThrough(S, MEASUREMENT_PORT, 1);
    
    /* 初始状态端口：[Vx, Vy, r, psi, psi_dot] */
    ssSetInputPortWidth(S, INIT_STATE_PORT, 5);
    ssSetInputPortRequiredContiguous(S, INIT_STATE_PORT, 1);
    ssSetInputPortDirectFeedThrough(S, INIT_STATE_PORT, 0);

    /* 设置输出端口 */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* 状态估计输出端口 */
    ssSetOutputPortWidth(S, STATE_OUTPUT_PORT, 5);
    
    /* 状态不确定度输出端口 */
    ssSetOutputPortWidth(S, UNCERT_OUTPUT_PORT, 5);

    /* 设置采样时间 */
    ssSetNumSampleTimes(S, 1);

    /* 设置工作向量大小（用于存储C++对象指针和标志） */
    ssSetNumPWork(S, 3);  /* 存储UKF、Dynamics对象指针和初始化标志 */

    /* 设置S-Function选项 */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
                   SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    设置采样时间
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    const real_T *sample_time_ptr = mxGetPr(ssGetSFcnParam(S, SAMPLE_TIME_PARAM));
    double dt = (sample_time_ptr != NULL && sample_time_ptr[0] > 0) ? sample_time_ptr[0] : 0.01;
    ssSetSampleTime(S, 0, dt);
    ssSetOffsetTime(S, 0, 0.0);
}

/* Function: mdlStart ========================================================
 * Abstract:
 *    初始化S-Function，创建UKF实例
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    /* 创建车辆动力学模型实例 */
    SemitrailerDynamics* dynamics = new SemitrailerDynamics();
    
    /* 设置车辆参数（典型半挂卡车参数） */
    dynamics->setVehicleParameters(
        6000.0,    // mt: 牵引车质量 (kg)
        10000.0,   // ms: 挂车质量 (kg)
        12000.0,   // Izt: 牵引车转动惯量 (kg·m²)
        80000.0,   // Izs: 挂车转动惯量 (kg·m²)
        1.4,       // a: 前桥到质心距离 (m)
        1.6,       // bt: 后桥到质心距离 (m)
        6.0,       // bs: 挂车轴到挂车质心距离 (m)
        0.5,       // Lh: 牵引销到牵引车质心距离 (m)
        80000.0,   // Caf: 前桥侧偏刚度 (N/rad)
        160000.0,  // Car: 后桥侧偏刚度 (N/rad)
        120000.0   // Cas: 挂车桥侧偏刚度 (N/rad)
    );
    
    /* 设置铰接参数 */
    dynamics->setHitchParameters(5000.0, 2000.0); // kh, ch
    
    /* 创建UKF估计器实例 */
    UKFEstimator* ukf = new UKFEstimator();
    
    /* 设置动力学模型 */
    ukf->setDynamicsModel(dynamics);
    
    /* 从参数设置过程噪声协方差，如果参数无效则使用默认值 */
    const real_T *Q_param = mxGetPr(ssGetSFcnParam(S, PROCESS_NOISE_PARAM));
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5);
    static const real_T default_Q[5] = {0.01, 0.01, 0.001, 0.001, 0.001};  // 默认过程噪声
    
    for (int i = 0; i < 5; i++) {
        if (Q_param != NULL && mxGetNumberOfElements(ssGetSFcnParam(S, PROCESS_NOISE_PARAM)) >= 5) {
            Q(i, i) = Q_param[i];
        } else {
            Q(i, i) = default_Q[i];
        }
    }
    ukf->setProcessNoise(Q);
    
    /* 从参数设置测量噪声协方差，如果参数无效则使用默认值 */
    const real_T *R_param = mxGetPr(ssGetSFcnParam(S, MEAS_NOISE_PARAM));
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    static const real_T default_R[4] = {0.1, 0.01, 0.01, 0.1};  // 默认测量噪声
    
    for (int i = 0; i < 4; i++) {
        if (R_param != NULL && mxGetNumberOfElements(ssGetSFcnParam(S, MEAS_NOISE_PARAM)) >= 4) {
            R(i, i) = R_param[i];
        } else {
            R(i, i) = default_R[i];
        }
    }
    ukf->setMeasurementNoise(R);
    
    /* 设置UKF参数 */
    ukf->setUKFParameters(1e-3, 2.0, 0.0);  // alpha, beta, kappa
    
    /* 将对象指针存储在工作向量中 */
    ssGetPWork(S)[UKF_INSTANCE_PTR] = (void*)ukf;
    ssGetPWork(S)[DYNAMICS_INSTANCE_PTR] = (void*)dynamics;
    ssGetPWork(S)[INITIALIZED_FLAG_PTR] = (void*)0; /* 初始化标志设为0（未初始化） */
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    计算S-Function的输出
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid); /* 避免未使用参数警告 */
    
    /* 获取UKF实例 */
    UKFEstimator* ukf = static_cast<UKFEstimator*>(ssGetPWork(S)[UKF_INSTANCE_PTR]);
    
    /* 获取输入，如果没有连接则使用默认值 */
    const real_T *u_control_ptr = (const real_T*) ssGetInputPortSignal(S, CONTROL_INPUT_PORT);
    const real_T *measurements_ptr = (const real_T*) ssGetInputPortSignal(S, MEASUREMENT_PORT);
    const real_T *init_state_ptr = (const real_T*) ssGetInputPortSignal(S, INIT_STATE_PORT);
    
    /* 定义默认值 */
    static const real_T default_control[2] = {0.0, 0.0};  // [delta_f=0, F_drive=0]
    static const real_T default_measurements[4] = {20.0, 0.0, 0.0, 0.0};  // [Vx=20m/s, r=0, psi=0, psi_dot=0]
    static const real_T default_init_state[5] = {20.0, 0.0, 0.0, 0.0, 0.0};  // [Vx=20m/s, Vy=0, r=0, psi=0, psi_dot=0]
    
    /* 使用输入值或默认值 */
    const real_T *u_control = (u_control_ptr != NULL) ? u_control_ptr : default_control;
    const real_T *measurements = (measurements_ptr != NULL) ? measurements_ptr : default_measurements;
    const real_T *init_state = (init_state_ptr != NULL) ? init_state_ptr : default_init_state;
    
    /* 获取输出指针 */
    real_T *estimated_states = (real_T*)ssGetOutputPortSignal(S, STATE_OUTPUT_PORT);
    real_T *state_uncertainties = (real_T*)ssGetOutputPortSignal(S, UNCERT_OUTPUT_PORT);
    
    /* 在第一次调用时初始化滤波器 */
    int initialized_flag = (int)(intptr_t)ssGetPWork(S)[INITIALIZED_FLAG_PTR];
    if (!initialized_flag) {
        /* 使用输入的初始状态 */
        Eigen::VectorXd x0(5);
        for (int i = 0; i < 5; i++) {
            x0(i) = init_state[i];
        }
        
        /* 设置初始协方差矩阵 */
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(5, 5);
        P0(0, 0) = 1.0;   // Vx不确定度
        P0(1, 1) = 1.0;   // Vy不确定度
        P0(2, 2) = 0.1;   // r不确定度
        P0(3, 3) = 0.1;   // psi不确定度
        P0(4, 4) = 0.1;   // psi_dot不确定度
        
        ukf->initialize(x0, P0);
        ssGetPWork(S)[INITIALIZED_FLAG_PTR] = (void*)1; /* 设置为已初始化 */
    }
    
    /* 获取采样时间，如果参数无效则使用默认值 */
    const real_T *sample_time_ptr = mxGetPr(ssGetSFcnParam(S, SAMPLE_TIME_PARAM));
    double dt = (sample_time_ptr != NULL && sample_time_ptr[0] > 0) ? sample_time_ptr[0] : 0.01;
    
    /* 构造控制输入向量 */
    Eigen::VectorXd u(2);
    u(0) = u_control[0];  // delta_f
    u(1) = u_control[1];  // F_drive
    
    /* 预测步骤 */
    ukf->predict(u, dt);
    
    /* 构造测量向量 */
    Eigen::VectorXd z(4);
    z(0) = measurements[0];  // Vx_meas
    z(1) = measurements[1];  // r_meas
    z(2) = measurements[2];  // psi_meas
    z(3) = measurements[3];  // psi_dot_meas
    
    /* 更新步骤 */
    ukf->update(z);
    
    /* 获取估计结果 */
    Eigen::VectorXd x_est = ukf->getState();
    Eigen::VectorXd uncertainty = ukf->getStateUncertainty();
    
    /* 输出估计状态 */
    for (int i = 0; i < 5; i++) {
        estimated_states[i] = x_est(i);
        state_uncertainties[i] = uncertainty(i);
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    清理S-Function，释放分配的内存
 */
static void mdlTerminate(SimStruct *S)
{
    /* 释放UKF实例 */
    UKFEstimator* ukf = static_cast<UKFEstimator*>(ssGetPWork(S)[UKF_INSTANCE_PTR]);
    if (ukf != nullptr) {
        delete ukf;
        ssGetPWork(S)[UKF_INSTANCE_PTR] = nullptr;
    }
    
    /* 释放动力学模型实例 */
    SemitrailerDynamics* dynamics = static_cast<SemitrailerDynamics*>(ssGetPWork(S)[DYNAMICS_INSTANCE_PTR]);
    if (dynamics != nullptr) {
        delete dynamics;
        ssGetPWork(S)[DYNAMICS_INSTANCE_PTR] = nullptr;
    }
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
