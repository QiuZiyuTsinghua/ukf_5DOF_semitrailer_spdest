#ifndef SEMITRAILER_DYNAMICS_H
#define SEMITRAILER_DYNAMICS_H

#include <Eigen/Dense>

/**
 * 半挂卡车平面5-DOF动力学模型
 * 状态变量: [Vx, Vy, r, psi, psi_dot]
 * 输入变量: [delta_f, F_drive]
 */
class SemitrailerDynamics {
public:
    // 构造函数
    SemitrailerDynamics();
    
    // 设置车辆参数
    void setVehicleParameters(
        double mt,        // 牵引车质量 (kg)
        double ms,        // 挂车质量 (kg)
        double Izt,       // 牵引车转动惯量 (kg·m²)
        double Izs,       // 挂车转动惯量 (kg·m²)
        double a,         // 前桥到质心距离 (m)
        double bt,        // 后桥到质心距离 (m)
        double bs,        // 挂车轴到挂车质心距离 (m)
        double Lh,        // 牵引销到牵引车质心距离 (m)
        double Caf,       // 前桥侧偏刚度 (N/rad)
        double Car,       // 后桥侧偏刚度 (N/rad)
        double Cas        // 挂车桥侧偏刚度 (N/rad)
    );
    
    // 设置铰接阻尼参数
    void setHitchParameters(double kh, double ch);
    
    // 状态微分方程 dx/dt = f(x, u)
    Eigen::VectorXd stateDerivative(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
    
    // 四阶龙格库塔离散化
    Eigen::VectorXd integrate(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) const;
    
    // 观测方程 (轮速、航向角速度、铰接角等)
    Eigen::VectorXd observationModel(const Eigen::VectorXd& x) const;
    
    // 雅可比矩阵计算 (用于EKF，可选)
    Eigen::MatrixXd stateJacobian(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;
    Eigen::MatrixXd observationJacobian(const Eigen::VectorXd& x) const;
    
    // 获取状态维度
    static constexpr int getStateDim() { return 5; }
    static constexpr int getInputDim() { return 2; }
    static constexpr int getObsDim() { return 4; }

private:
    // 车辆参数
    double m_t, m_s;           // 质量
    double I_zt, I_zs;         // 转动惯量
    double a_, b_t, b_s, L_h;  // 几何参数
    double C_af, C_ar, C_as;   // 侧偏刚度
    double k_h, c_h;           // 铰接刚度和阻尼
    
    // 辅助函数
    double computeSlipAngle(double Vy, double r, double distance, double Vx, double steer = 0.0) const;
    double computeLateralForce(double slip_angle, double cornering_stiffness) const;
};

#endif // SEMITRAILER_DYNAMICS_H
