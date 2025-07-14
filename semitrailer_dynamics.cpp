#include "semitrailer_dynamics.h"
#include <cmath>
#include <algorithm>

SemitrailerDynamics::SemitrailerDynamics() 
    : m_t(8000.0), m_s(25000.0), I_zt(5000.0), I_zs(30000.0),
      a_(1.4), b_t(2.8), b_s(7.0), L_h(0.8),
      C_af(180000.0), C_ar(300000.0), C_as(400000.0),
      k_h(1000.0), c_h(5000.0)
{
}

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

void SemitrailerDynamics::setHitchParameters(double kh, double ch)
{
    k_h = kh;
    c_h = ch;
}

Eigen::VectorXd SemitrailerDynamics::stateDerivative(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
    // 状态: [Vx, Vy, r, psi, psi_dot]
    double Vx = x(0);
    double Vy = x(1);
    double r = x(2);
    double psi = x(3);
    double psi_dot = x(4);
    
    // 输入: [delta_f, F_drive]
    double delta_f = u(0);
    double F_drive = u(1);
    
    // 防止除零
    double Vx_safe = std::max(std::abs(Vx), 0.1);
    if (Vx < 0) Vx_safe = -Vx_safe;
    
    // 计算侧偏角
    double alpha_f = delta_f - (Vy + a_ * r) / Vx_safe;
    double alpha_r = -(Vy - b_t * r) / Vx_safe;
    
    // 挂车运动学
    double cos_psi = std::cos(psi);
    double sin_psi = std::sin(psi);
    
    double Vx_s = Vx * cos_psi + (Vy + L_h * r) * sin_psi;
    double Vy_s = -Vx * sin_psi + (Vy + L_h * r) * cos_psi;
    double r_s = r + psi_dot;
    
    double Vx_s_safe = std::max(std::abs(Vx_s), 0.1);
    if (Vx_s < 0) Vx_s_safe = -Vx_s_safe;
    
    double alpha_s = psi - (Vy_s - b_s * r_s) / Vx_s_safe;
    
    // 轮胎侧向力
    double Fy_f = computeLateralForce(alpha_f, C_af);
    double Fy_r = computeLateralForce(alpha_r, C_ar);
    double Fy_s = computeLateralForce(alpha_s, C_as);
    
    // 铰接力
    double Fy_h = k_h * psi + c_h * psi_dot;
    
    // 牵引车动力学方程
    double Vx_dot = (F_drive + Fy_f * std::sin(delta_f) - Fy_h * sin_psi) / m_t + r * Vy;
    double Vy_dot = (Fy_f * std::cos(delta_f) + Fy_r - Fy_h * cos_psi) / m_t - r * Vx;
    double r_dot = (a_ * Fy_f * std::cos(delta_f) - b_t * Fy_r - L_h * Fy_h * cos_psi) / I_zt;
    
    // 挂车横摆动力学 (简化)
    double rs_dot = (Fy_s - b_s * Fy_s + Fy_h) / I_zs;  // 简化的挂车横摆方程
    
    // 铰接角动力学
    double psi_ddot = rs_dot - r_dot;
    
    Eigen::VectorXd x_dot(5);
    x_dot << Vx_dot, Vy_dot, r_dot, psi_dot, psi_ddot;
    
    return x_dot;
}

Eigen::VectorXd SemitrailerDynamics::integrate(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) const
{
    // 四阶龙格库塔法
    Eigen::VectorXd k1 = dt * stateDerivative(x, u);
    Eigen::VectorXd k2 = dt * stateDerivative(x + 0.5 * k1, u);
    Eigen::VectorXd k3 = dt * stateDerivative(x + 0.5 * k2, u);
    Eigen::VectorXd k4 = dt * stateDerivative(x + k3, u);
    
    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

Eigen::VectorXd SemitrailerDynamics::observationModel(const Eigen::VectorXd& x) const
{
    // 观测: [轮速估计, 航向角速度, 铰接角, 纵向加速度]
    double Vx = x(0);
    double r = x(2);
    double psi = x(3);
    
    // 简单的观测模型
    Eigen::VectorXd z(4);
    z(0) = Vx;              // 轮速
    z(1) = r;               // 航向角速度
    z(2) = psi;             // 铰接角
    z(3) = 0.0;             // 纵向加速度 (需要从状态导数计算)
    
    return z;
}

double SemitrailerDynamics::computeSlipAngle(double Vy, double r, double distance, double Vx, double steer) const
{
    double Vx_safe = std::max(std::abs(Vx), 0.1);
    if (Vx < 0) Vx_safe = -Vx_safe;
    
    return steer - (Vy + distance * r) / Vx_safe;
}

double SemitrailerDynamics::computeLateralForce(double slip_angle, double cornering_stiffness) const
{
    // 线性轮胎模型
    return -cornering_stiffness * slip_angle;
}

Eigen::MatrixXd SemitrailerDynamics::stateJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
    // 数值雅可比 (简化实现)
    const double eps = 1e-6;
    Eigen::MatrixXd J(5, 5);
    
    for (int i = 0; i < 5; ++i) {
        Eigen::VectorXd x_plus = x;
        Eigen::VectorXd x_minus = x;
        x_plus(i) += eps;
        x_minus(i) -= eps;
        
        Eigen::VectorXd f_plus = stateDerivative(x_plus, u);
        Eigen::VectorXd f_minus = stateDerivative(x_minus, u);
        
        J.col(i) = (f_plus - f_minus) / (2.0 * eps);
    }
    
    return J;
}

Eigen::MatrixXd SemitrailerDynamics::observationJacobian(const Eigen::VectorXd& x) const
{
    // 观测雅可比
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 5);
    H(0, 0) = 1.0;  // dz1/dVx
    H(1, 2) = 1.0;  // dz2/dr
    H(2, 3) = 1.0;  // dz3/dpsi
    H(3, 0) = 0.0;  // dz4/dVx (纵向加速度)
    
    return H;
}
