#include "../include/semitrailer_dynamics.h"
#include <cmath>
#include <algorithm>

/**
 * @brief 构造函数 - 初始化半挂卡车动力学模型的默认参数
 * Constructor - Initialize default parameters for semitrailer dynamics model
 */
SemitrailerDynamics::SemitrailerDynamics() 
    : m_t(8000.0), m_s(25000.0), I_zt(5000.0), I_zs(30000.0),
      a_(1.4), b_t(2.8), b_s(7.0), L_h(0.8),
      C_af(180000.0), C_ar(300000.0), C_as(400000.0),
      k_h(1000.0), c_h(5000.0)
{
}

/**
 * @brief 设置车辆参数 - 配置牵引车和挂车的物理参数
 * Set vehicle parameters - Configure physical parameters of tractor and trailer
 * 
 * @param mt  牵引车质量 (kg) / Tractor mass (kg)
 * @param ms  挂车质量 (kg) / Trailer mass (kg)
 * @param Izt 牵引车绕z轴转动惯量 (kg·m²) / Tractor yaw moment of inertia (kg·m²)
 * @param Izs 挂车绕z轴转动惯量 (kg·m²) / Trailer yaw moment of inertia (kg·m²)
 * @param a   前桥到牵引车质心距离 (m) / Distance from front axle to tractor CG (m)
 * @param bt  后桥到牵引车质心距离 (m) / Distance from rear axle to tractor CG (m)
 * @param bs  挂车轴到挂车质心距离 (m) / Distance from trailer axle to trailer CG (m)
 * @param Lh  铰接点到牵引车质心距离 (m) / Distance from hitch to tractor CG (m)
 * @param Caf 前桥侧偏刚度 (N/rad) / Front axle cornering stiffness (N/rad)
 * @param Car 后桥侧偏刚度 (N/rad) / Rear axle cornering stiffness (N/rad)
 * @param Cas 挂车桥侧偏刚度 (N/rad) / Trailer axle cornering stiffness (N/rad)
 */
void SemitrailerDynamics::setVehicleParameters(
    double mt, double ms, double Izt, double Izs,
    double a, double bt, double bs, double Lh,
    double Caf, double Car, double Cas)
{
    m_t = mt; m_s = ms;
    I_zt = Izt; I_zs = Izs;
    a_ = a; b_t = bt; b_s = bs; L_h = Lh;
    C_af = Caf; C_ar = Car; C_as = Cas;
}

/**
 * @brief 设置铰接参数 - 配置铰接点的刚度和阻尼参数
 * Set hitch parameters - Configure stiffness and damping parameters of the hitch
 * 
 * @param kh 铰接刚度 (N·m/rad) / Hitch stiffness (N·m/rad)
 * @param ch 铰接阻尼 (N·m·s/rad) / Hitch damping (N·m·s/rad)
 */
void SemitrailerDynamics::setHitchParameters(double kh, double ch)
{
    k_h = kh;
    c_h = ch;
}

/**
 * @brief 计算状态导数 - 半挂卡车5-DOF动力学微分方程
 * Calculate state derivatives - 5-DOF semitrailer dynamics differential equations
 * 
 * 实现半挂卡车平面运动的状态微分方程，包括：
 * Implements state differential equations for semitrailer planar motion, including:
 * - 牵引车纵横向动力学 / Tractor longitudinal and lateral dynamics
 * - 牵引车横摆动力学 / Tractor yaw dynamics  
 * - 铰接角动力学 / Articulation angle dynamics
 * - 轮胎侧向力计算 / Tire lateral force calculation
 * 
 * @param x 状态向量 [Vx, Vy, r, psi, psi_dot] / State vector [Vx, Vy, r, psi, psi_dot]
 *          Vx: 纵向速度 (m/s) / Longitudinal velocity (m/s)
 *          Vy: 横向速度 (m/s) / Lateral velocity (m/s)
 *          r: 航向角速度 (rad/s) / Yaw rate (rad/s)
 *          psi: 铰接角 (rad) / Articulation angle (rad)
 *          psi_dot: 铰接角速度 (rad/s) / Articulation angular velocity (rad/s)
 * @param u 控制输入 [delta_f, F_drive] / Control input [delta_f, F_drive]
 *          delta_f: 前轮转角 (rad) / Front wheel steering angle (rad)
 *          F_drive: 驱动力 (N) / Drive force (N)
 * @return 状态导数向量 / State derivative vector
 */
Eigen::VectorXd SemitrailerDynamics::stateDerivative(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
    // 状态: [Vx, Vy, r, psi, psi_dot]
    // States: [Vx, Vy, r, psi, psi_dot]
    double Vx = x(0);       // 纵向速度 / Longitudinal velocity
    double Vy = x(1);       // 横向速度 / Lateral velocity
    double r = x(2);        // 航向角速度 / Yaw rate
    double psi = x(3);      // 铰接角 / Articulation angle
    double psi_dot = x(4);  // 铰接角速度 / Articulation angular velocity
    
    // 输入: [delta_f, F_drive]
    // Inputs: [delta_f, F_drive]
    double delta_f = u(0);  // 前轮转角 / Front wheel steering angle
    double F_drive = u(1);  // 驱动力 / Drive force
    
    // 防止除零 - 确保速度不为零以避免数值问题
    // Prevent division by zero - ensure non-zero velocity to avoid numerical issues
    double Vx_safe = std::max(std::abs(Vx), 0.1);
    if (Vx < 0) Vx_safe = -Vx_safe;
    
    // 计算侧偏角 - 轮胎与车辆运动方向的夹角
    // Calculate slip angles - angle between tire and vehicle motion direction
    double alpha_f = delta_f - (Vy + a_ * r) / Vx_safe;      // 前轮侧偏角 / Front tire slip angle
    double alpha_r = -(Vy - b_t * r) / Vx_safe;              // 后轮侧偏角 / Rear tire slip angle
    
    // 挂车运动学 - 将牵引车运动转换到挂车坐标系
    // Trailer kinematics - transform tractor motion to trailer coordinate system
    double cos_psi = std::cos(psi);
    double sin_psi = std::sin(psi);
    
    // 挂车质心速度 (在牵引车坐标系中表示)
    // Trailer CG velocity (expressed in tractor coordinate system)
    double Vx_s = Vx * cos_psi + (Vy + L_h * r) * sin_psi;   // 挂车纵向速度 / Trailer longitudinal velocity
    double Vy_s = -Vx * sin_psi + (Vy + L_h * r) * cos_psi;  // 挂车横向速度 / Trailer lateral velocity
    double r_s = r + psi_dot;                                 // 挂车航向角速度 / Trailer yaw rate
    
    // 挂车速度的除零保护
    // Division by zero protection for trailer velocity
    double Vx_s_safe = std::max(std::abs(Vx_s), 0.1);
    if (Vx_s < 0) Vx_s_safe = -Vx_s_safe;
    
    // 挂车轮胎侧偏角
    // Trailer tire slip angle
    double alpha_s = psi - (Vy_s - b_s * r_s) / Vx_s_safe;
    
    // 轮胎侧向力 - 基于线性轮胎模型
    // Tire lateral forces - based on linear tire model
    double Fy_f = computeLateralForce(alpha_f, C_af);  // 前轮侧向力 / Front tire lateral force
    double Fy_r = computeLateralForce(alpha_r, C_ar);  // 后轮侧向力 / Rear tire lateral force
    double Fy_s = computeLateralForce(alpha_s, C_as);  // 挂车轮胎侧向力 / Trailer tire lateral force
    
    // 铰接力 - 铰接点处的约束力和力矩
    // Hitch force - constraint force and moment at hitch point
    double Fy_h = k_h * psi + c_h * psi_dot;  // 铰接横向力 / Hitch lateral force
    
    // 牵引车动力学方程 - 基于牛顿第二定律
    // Tractor dynamics equations - based on Newton's second law
    
    // 纵向动力学: m*a_x = ΣF_x (包含离心力项)
    // Longitudinal dynamics: m*a_x = ΣF_x (including centrifugal force term)
    double Vx_dot = (F_drive + Fy_f * std::sin(delta_f) - Fy_h * sin_psi) / m_t + r * Vy;
    
    // 横向动力学: m*a_y = ΣF_y (包含离心力项)  
    // Lateral dynamics: m*a_y = ΣF_y (including centrifugal force term)
    double Vy_dot = (Fy_f * std::cos(delta_f) + Fy_r - Fy_h * cos_psi) / m_t - r * Vx;
    
    // 横摆动力学: I_z*α_z = ΣM_z (绕质心的力矩平衡)
    // Yaw dynamics: I_z*α_z = ΣM_z (moment balance about CG)
    double r_dot = (a_ * Fy_f * std::cos(delta_f) - b_t * Fy_r - L_h * Fy_h * cos_psi) / I_zt;
    
    // 挂车横摆动力学 (简化) - 考虑挂车轮胎力和铰接力的影响
    // Trailer yaw dynamics (simplified) - considering trailer tire force and hitch force effects
    double rs_dot = (Fy_s - b_s * Fy_s + Fy_h) / I_zs;  // 简化的挂车横摆方程 / Simplified trailer yaw equation
    
    // 铰接角动力学 - 铰接角速度定义
    // Articulation angle dynamics - definition of articulation angular velocity
    double psi_ddot = rs_dot - r_dot;  // 铰接角加速度 = 挂车角加速度 - 牵引车角加速度 / Articulation angular acceleration
    
    // 组装状态导数向量
    // Assemble state derivative vector
    Eigen::VectorXd x_dot(5);
    x_dot << Vx_dot, Vy_dot, r_dot, psi_dot, psi_ddot;
    
    return x_dot;
}

/**
 * @brief 四阶龙格库塔积分 - 数值求解微分方程
 * Fourth-order Runge-Kutta integration - numerical solution of differential equations
 * 
 * 使用经典的四阶龙格库塔方法对状态微分方程进行数值积分，
 * 相比简单的欧拉方法具有更高的精度和数值稳定性
 * Uses classic fourth-order Runge-Kutta method for numerical integration of state
 * differential equations, providing higher accuracy and numerical stability than simple Euler method
 * 
 * @param x  当前状态向量 / Current state vector
 * @param u  控制输入向量 / Control input vector  
 * @param dt 积分步长 (s) / Integration time step (s)
 * @return   下一时刻的状态向量 / State vector at next time step
 */
Eigen::VectorXd SemitrailerDynamics::integrate(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) const
{
    // 四阶龙格库塔法 - 经典的四步积分算法
    // Fourth-order Runge-Kutta method - classic four-step integration algorithm
    Eigen::VectorXd k1 = dt * stateDerivative(x, u);              // 第一步：在起点处的斜率 / Step 1: slope at starting point
    Eigen::VectorXd k2 = dt * stateDerivative(x + 0.5 * k1, u);   // 第二步：在中点处的斜率 / Step 2: slope at midpoint
    Eigen::VectorXd k3 = dt * stateDerivative(x + 0.5 * k2, u);   // 第三步：在中点处的修正斜率 / Step 3: corrected slope at midpoint
    Eigen::VectorXd k4 = dt * stateDerivative(x + k3, u);         // 第四步：在终点处的斜率 / Step 4: slope at endpoint
    
    // 加权平均得到最终结果：(k1 + 2*k2 + 2*k3 + k4)/6
    // Weighted average for final result: (k1 + 2*k2 + 2*k3 + k4)/6
    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

/**
 * @brief 观测模型 - 将状态变量映射到可测量的观测量
 * Observation model - map state variables to measurable observations
 * 
 * 定义状态向量与传感器测量值之间的关系，用于卡尔曼滤波器的更新步骤
 * Defines relationship between state vector and sensor measurements for Kalman filter update step
 * 
 * @param x 状态向量 [Vx, Vy, r, psi, psi_dot] / State vector [Vx, Vy, r, psi, psi_dot]
 * @return  观测向量 [轮速, 航向角速度, 铰接角, 纵向加速度] / Observation vector [wheel_speed, yaw_rate, articulation_angle, longitudinal_acceleration]
 */
Eigen::VectorXd SemitrailerDynamics::observationModel(const Eigen::VectorXd& x) const
{
    // 从状态向量中提取相关变量
    // Extract relevant variables from state vector
    double Vx = x(0);   // 纵向速度 / Longitudinal velocity
    double r = x(2);    // 航向角速度 / Yaw rate
    double psi = x(3);  // 铰接角 / Articulation angle
    
    // 构建观测向量 - 对应实际传感器可以测量的物理量
    // Build observation vector - corresponding to physical quantities measurable by actual sensors
    Eigen::VectorXd z(4);
    z(0) = Vx;              // 轮速传感器测量的速度 / Velocity measured by wheel speed sensor
    z(1) = r;               // 陀螺仪测量的航向角速度 / Yaw rate measured by gyroscope
    z(2) = psi;             // 铰接角编码器测量值 / Articulation angle measured by encoder
    z(3) = 0.0;             // 纵向加速度 (需要从状态导数计算) / Longitudinal acceleration (needs calculation from state derivatives)
    
    return z;
}

/**
 * @brief 计算侧偏角 - 轮胎与运动方向的夹角
 * Calculate slip angle - angle between tire and motion direction
 * 
 * 侧偏角是轮胎动力学的关键参数，定义为轮胎指向与实际运动方向的夹角
 * Slip angle is a key parameter in tire dynamics, defined as the angle between
 * tire heading and actual motion direction
 * 
 * @param Vy       横向速度 (m/s) / Lateral velocity (m/s)
 * @param r        航向角速度 (rad/s) / Yaw rate (rad/s)  
 * @param distance 轮胎到质心的距离 (m) / Distance from tire to CG (m)
 * @param Vx       纵向速度 (m/s) / Longitudinal velocity (m/s)
 * @param steer    轮胎转向角 (rad) / Tire steering angle (rad)
 * @return         侧偏角 (rad) / Slip angle (rad)
 */
double SemitrailerDynamics::computeSlipAngle(double Vy, double r, double distance, double Vx, double steer) const
{
    // 防止除零保护
    // Division by zero protection
    double Vx_safe = std::max(std::abs(Vx), 0.1);
    if (Vx < 0) Vx_safe = -Vx_safe;
    
    // 侧偏角 = 转向角 - arctan(横向速度分量/纵向速度)
    // Slip angle = steering angle - arctan(lateral velocity component / longitudinal velocity)
    return steer - (Vy + distance * r) / Vx_safe;
}

/**
 * @brief 计算轮胎侧向力 - 基于线性轮胎模型
 * Calculate tire lateral force - based on linear tire model
 * 
 * 采用线性轮胎模型，假设侧向力与侧偏角成正比关系
 * Uses linear tire model, assuming lateral force is proportional to slip angle
 * 
 * @param slip_angle          侧偏角 (rad) / Slip angle (rad)
 * @param cornering_stiffness 侧偏刚度 (N/rad) / Cornering stiffness (N/rad)
 * @return                    轮胎侧向力 (N) / Tire lateral force (N)
 */
double SemitrailerDynamics::computeLateralForce(double slip_angle, double cornering_stiffness) const
{
    // 线性轮胎模型: F_y = -C_α * α
    // Linear tire model: F_y = -C_α * α
    // 负号表示侧向力方向与侧偏角方向相反 (回正力矩)
    // Negative sign indicates lateral force opposes slip angle direction (restoring moment)
    return -cornering_stiffness * slip_angle;
}

/**
 * @brief 计算状态雅可比矩阵 - 用于扩展卡尔曼滤波器 (可选)
 * Calculate state Jacobian matrix - for Extended Kalman Filter (optional)
 * 
 * 使用数值微分方法计算状态转移函数对状态变量的偏导数矩阵
 * Uses numerical differentiation to calculate partial derivative matrix of state transition function
 * with respect to state variables
 * 
 * @param x 状态向量 / State vector
 * @param u 控制输入向量 / Control input vector
 * @return  状态雅可比矩阵 ∂f/∂x / State Jacobian matrix ∂f/∂x
 */
Eigen::MatrixXd SemitrailerDynamics::stateJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
    // 数值雅可比 (简化实现) - 使用中心差分法
    // Numerical Jacobian (simplified implementation) - using central difference method
    const double eps = 1e-6;  // 数值微分步长 / Numerical differentiation step size
    Eigen::MatrixXd J(5, 5);
    
    // 对每个状态变量进行数值微分
    // Numerical differentiation for each state variable
    for (int i = 0; i < 5; ++i) {
        Eigen::VectorXd x_plus = x;
        Eigen::VectorXd x_minus = x;
        x_plus(i) += eps;   // 正向扰动 / Forward perturbation
        x_minus(i) -= eps;  // 负向扰动 / Backward perturbation
        
        // 中心差分法计算偏导数: ∂f/∂x_i ≈ (f(x+ε) - f(x-ε)) / (2ε)
        // Central difference method for partial derivative: ∂f/∂x_i ≈ (f(x+ε) - f(x-ε)) / (2ε)
        Eigen::VectorXd f_plus = stateDerivative(x_plus, u);
        Eigen::VectorXd f_minus = stateDerivative(x_minus, u);
        
        J.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }
    
    return J;
}

/**
 * @brief 计算观测雅可比矩阵 - 观测函数对状态变量的偏导数
 * Calculate observation Jacobian matrix - partial derivatives of observation function w.r.t. state variables
 * 
 * 计算观测模型相对于状态变量的雅可比矩阵，用于卡尔曼滤波器的增益计算
 * Calculate Jacobian matrix of observation model with respect to state variables,
 * used for gain calculation in Kalman filter
 * 
 * @param x 状态向量 / State vector
 * @return  观测雅可比矩阵 H = ∂h/∂x / Observation Jacobian matrix H = ∂h/∂x
 */
Eigen::MatrixXd SemitrailerDynamics::observationJacobian(const Eigen::VectorXd& x) const
{
    // 观测雅可比矩阵 - 线性观测模型的雅可比是常数矩阵
    // Observation Jacobian matrix - Jacobian of linear observation model is constant matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 5);
    
    // 观测方程: z = [Vx, r, psi, ax] = [x(0), x(2), x(3), f(x)]
    // Observation equations: z = [Vx, r, psi, ax] = [x(0), x(2), x(3), f(x)]
    H(0, 0) = 1.0;  // ∂z1/∂Vx = ∂Vx/∂Vx = 1 (轮速对纵向速度的偏导数)
    H(1, 2) = 1.0;  // ∂z2/∂r = ∂r/∂r = 1 (航向角速度对自身的偏导数)
    H(2, 3) = 1.0;  // ∂z3/∂psi = ∂psi/∂psi = 1 (铰接角对自身的偏导数)
    H(3, 0) = 0.0;  // ∂z4/∂Vx = 0 (纵向加速度暂时假设与状态无关)
    
    return H;
}
