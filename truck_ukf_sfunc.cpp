/*
 * truck_ukf_sfunc.cpp
 * 
 * 半挂卡车5-DOF UKF状态估计S-Function
 * 用于Simulink与TruckSim联合仿真
 * 
 * 状态变量: [Vx, Vy, r, psi, psi_dot] (5-DOF)
 * 输入:
 *   - 轮速信号 (m/s)
 *   - 纵向加速度 ax (m/s²)
 *   - 横向加速度 ay (m/s²)
 *   - 航向角速度 r (rad/s)
 *   - 纵向驱动力 F_drive (N)
 *   - 转向角度 (rad) - 可以是方向盘转角或前轮转角，由转向传动比参数决定
 * 
 * 输出:
 *   - 估计纵向速度 Vx (m/s)
 *   - 估计横向速度 Vy (m/s)
 *   - 估计航向角速度 r (rad/s)
 *   - 估计铰接角 psi (rad)
 *   - 估计铰接角速度 psi_dot (rad/s)
 *   - 速度不确定度 (m/s)
 */

#define S_FUNCTION_NAME  truck_ukf_sfunc
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "semitrailer_dynamics.h"
#include "ukf_estimator.h"

// S-Function参数
#define NUM_PARAMS 13
#define TRACTOR_MASS_PARAM      ssGetSFcnParam(S, 0)
#define TRAILER_MASS_PARAM      ssGetSFcnParam(S, 1)
#define TRACTOR_INERTIA_PARAM   ssGetSFcnParam(S, 2)
#define TRAILER_INERTIA_PARAM   ssGetSFcnParam(S, 3)
#define FRONT_AXLE_PARAM        ssGetSFcnParam(S, 4)
#define REAR_AXLE_PARAM         ssGetSFcnParam(S, 5)
#define TRAILER_AXLE_PARAM      ssGetSFcnParam(S, 6)
#define HITCH_DISTANCE_PARAM    ssGetSFcnParam(S, 7)
#define FRONT_CORNERING_PARAM   ssGetSFcnParam(S, 8)
#define REAR_CORNERING_PARAM    ssGetSFcnParam(S, 9)
#define TRAILER_CORNERING_PARAM ssGetSFcnParam(S, 10)
#define STEERING_RATIO_PARAM    ssGetSFcnParam(S, 11)
#define SAMPLE_TIME_PARAM       ssGetSFcnParam(S, 12)

// 输入输出端口定义
#define INPUT_WHEEL_SPEED    0
#define INPUT_ACC_X          1
#define INPUT_ACC_Y          2
#define INPUT_YAW_RATE       3
#define INPUT_DRIVE_FORCE    4
#define INPUT_STEER_ANGLE    5

#define OUTPUT_EST_VX        0
#define OUTPUT_EST_VY        1
#define OUTPUT_EST_R         2
#define OUTPUT_EST_PSI       3
#define OUTPUT_EST_PSI_DOT   4
#define OUTPUT_UNCERTAINTY   5

// 为嵌入式平台优化的简单实现，不使用STL

/*================*
 * S-function methods *
 *================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 6);  /* [wheel_speed, acc_x, acc_y, yaw_rate, drive_force, front_steer] */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 6);  /* [est_Vx, est_Vy, est_r, est_psi, est_psi_dot, uncertainty] */

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 2);  /* 存储dynamics和ukf对象指针 */
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T sample_time = mxGetPr(SAMPLE_TIME_PARAM)[0];
    ssSetSampleTime(S, 0, sample_time);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    In this function, you should initialize the continuous and discrete
 *    states for your S-function block.  The initial states are placed
 *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
 *    You can also perform any other initialization activities that your
 *    S-function may require. Note, this routine will be called at the
 *    start of simulation and if it is present in an enabled subsystem
 *    configured to reset states, it will be called when the enabled
 *    subsystem restarts execution to reset the states.
 */
static void mdlInitializeConditions(SimStruct *S)
{
    // 创建车辆动力学模型
    SemitrailerDynamics* dynamics = new SemitrailerDynamics();
    
    // 设置车辆参数
    real_T mt = mxGetPr(TRACTOR_MASS_PARAM)[0];
    real_T ms = mxGetPr(TRAILER_MASS_PARAM)[0];
    real_T Izt = mxGetPr(TRACTOR_INERTIA_PARAM)[0];
    real_T Izs = mxGetPr(TRAILER_INERTIA_PARAM)[0];
    real_T a = mxGetPr(FRONT_AXLE_PARAM)[0];
    real_T bt = mxGetPr(REAR_AXLE_PARAM)[0];
    real_T bs = mxGetPr(TRAILER_AXLE_PARAM)[0];
    real_T Lh = mxGetPr(HITCH_DISTANCE_PARAM)[0];
    real_T Caf = mxGetPr(FRONT_CORNERING_PARAM)[0];
    real_T Car = mxGetPr(REAR_CORNERING_PARAM)[0];
    real_T Cas = mxGetPr(TRAILER_CORNERING_PARAM)[0];
    
    dynamics->setVehicleParameters(mt, ms, Izt, Izs, a, bt, bs, Lh, Caf, Car, Cas);
    dynamics->setHitchParameters(1000.0, 5000.0);  // 铰接刚度和阻尼
    
    // 创建UKF估计器
    UKFEstimator* ukf = new UKFEstimator();
    
    // 初始化状态 [Vx, Vy, r, psi, psi_dot]
    Eigen::VectorXd initial_state(5);
    initial_state << 0.0, 0.0, 0.0, 0.0, 0.0;
    
    // 初始化协方差
    Eigen::MatrixXd initial_P = Eigen::MatrixXd::Identity(5, 5);
    initial_P.diagonal() << 1.0, 1.0, 0.1, 0.1, 0.1;
    
    ukf->initialize(initial_state, initial_P);
    
    // 设置噪声协方差
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(5, 5);
    Q.diagonal() << 0.1, 0.1, 0.01, 0.01, 0.01;
    
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R.diagonal() << 0.1, 0.01, 0.05, 0.1;
    
    ukf->setProcessNoise(Q);
    ukf->setMeasurementNoise(R);
    ukf->setDynamicsModel(dynamics);  // 直接传递指针
    
    // 存储对象指针
    ssGetPWork(S)[0] = dynamics;
    ssGetPWork(S)[1] = ukf;
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // 获取对象实例
    SemitrailerDynamics* dynamics = static_cast<SemitrailerDynamics*>(ssGetPWork(S)[0]);
    UKFEstimator* ukf = static_cast<UKFEstimator*>(ssGetPWork(S)[1]);
    
    if (dynamics == NULL || ukf == NULL) {
        ssSetErrorStatus(S, "Dynamics or UKF instance not found");
        return;
    }
    
    // 获取输入
    const real_T *u = (const real_T*) ssGetInputPortSignal(S, 0);
    real_T wheel_speed = u[0];      // 轮速信号 (m/s)
    real_T acc_x = u[1];            // 纵向加速度 (m/s²)
    real_T acc_y = u[2];            // 横向加速度 (m/s²)
    real_T yaw_rate = u[3];         // 航向角速度 (rad/s)
    real_T drive_force = u[4];      // 纵向驱动力 (N)
    real_T steer_angle = u[5];      // 转向角度 (rad) - 方向盘转角或前轮转角
    
    // 获取采样时间
    real_T sample_time = mxGetPr(SAMPLE_TIME_PARAM)[0];
    
    // 获取转向传动比参数
    real_T steering_ratio = mxGetPr(STEERING_RATIO_PARAM)[0];
    
    // 构建控制输入 [delta_f, F_drive]
    Eigen::VectorXd control_input(2);
    
    // 转向角度处理：如果steering_ratio=1，表示输入已经是前轮转角；否则进行转换
    // Steering angle processing: if steering_ratio=1, input is already front wheel angle; otherwise convert
    if (steering_ratio > 0.1) {
        control_input(0) = steer_angle / steering_ratio;  // 方向盘转角转换为前轮转角 / Convert steering wheel angle to front wheel angle
    } else {
        control_input(0) = steer_angle;                   // 直接使用前轮转角 / Use front wheel angle directly
    }
    
    control_input(1) = drive_force;                       // 纵向驱动力 (直接使用) / Longitudinal drive force (direct use)
    
    // UKF预测步骤
    ukf->predict(control_input, sample_time);
    
    // 构建观测向量 [轮速, 航向角速度, 铰接角估计, 纵向加速度]
    Eigen::VectorXd measurement(4);
    measurement(0) = wheel_speed;
    measurement(1) = yaw_rate;
    measurement(2) = 0.0;  // 铰接角测量 (如果可用)
    measurement(3) = acc_x;
    
    // UKF更新步骤
    ukf->update(measurement);
    
    // 获取状态估计
    Eigen::VectorXd state = ukf->getState();
    Eigen::VectorXd uncertainty = ukf->getStateUncertainty();
    
    // 设置输出
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    y[0] = state(0);          // Vx - 纵向速度
    y[1] = state(1);          // Vy - 横向速度
    y[2] = state(2);          // r - 航向角速度
    y[3] = state(3);          // psi - 铰接角
    y[4] = state(4);          // psi_dot - 铰接角速度
    y[5] = uncertainty(0);    // 纵向速度不确定度
}

#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // 此函数在离散状态更新时调用，我们使用连续更新，所以这里为空
}
#endif /* MDL_UPDATE */

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    // 清理对象实例
    SemitrailerDynamics* dynamics = static_cast<SemitrailerDynamics*>(ssGetPWork(S)[0]);
    UKFEstimator* ukf = static_cast<UKFEstimator*>(ssGetPWork(S)[1]);
    
    if (dynamics != NULL) {
        delete dynamics;
        ssGetPWork(S)[0] = NULL;
    }
    
    if (ukf != NULL) {
        delete ukf;
        ssGetPWork(S)[1] = NULL;
    }
}

/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
