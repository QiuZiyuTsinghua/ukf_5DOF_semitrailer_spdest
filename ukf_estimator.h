#ifndef UKF_ESTIMATOR_H
#define UKF_ESTIMATOR_H

#include <Eigen/Dense>
#include "semitrailer_dynamics.h"

/**
 * Unscented Kalman Filter for Semitrailer State Estimation
 * 使用Merwe scaled sigma-points方法
 */
class UKFEstimator {
public:
    // 构造函数
    UKFEstimator();
    
    // 初始化滤波器
    void initialize(const Eigen::VectorXd& initial_state, 
                   const Eigen::MatrixXd& initial_covariance);
    
    // 设置噪声协方差
    void setProcessNoise(const Eigen::MatrixXd& Q);
    void setMeasurementNoise(const Eigen::MatrixXd& R);
    
    // 设置UKF参数
    void setUKFParameters(double alpha = 1e-3, double beta = 2.0, double kappa = 0.0);
    
    // 预测步骤
    void predict(const Eigen::VectorXd& control_input, double dt);
    
    // 更新步骤
    void update(const Eigen::VectorXd& measurement);
    
    // 获取估计状态和协方差
    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }
    
    // 设置车辆动力学模型
    void setDynamicsModel(SemitrailerDynamics* dynamics);
    
    // 获取状态不确定度
    Eigen::VectorXd getStateUncertainty() const;

private:
    // 状态和协方差
    Eigen::VectorXd x_;       // 状态估计
    Eigen::MatrixXd P_;       // 协方差矩阵
    
    // 噪声协方差
    Eigen::MatrixXd Q_;       // 过程噪声
    Eigen::MatrixXd R_;       // 测量噪声
    
    // UKF参数
    double alpha_, beta_, kappa_;
    double lambda_;
    
    // 维度
    int n_x_;                 // 状态维度
    int n_z_;                 // 观测维度
    int n_aug_;               // 增广状态维度
    
    // Sigma点权重
    Eigen::VectorXd weights_m_;  // 均值权重
    Eigen::VectorXd weights_c_;  // 协方差权重
    
    // 车辆动力学模型 (原始指针，不使用智能指针)
    SemitrailerDynamics* dynamics_;
    
    // 辅助函数
    void generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, 
                           Eigen::MatrixXd& sigma_points);
    void generateAugmentedSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                                    const Eigen::VectorXd& u, double dt,
                                    Eigen::MatrixXd& sigma_points);
    void predictSigmaPoints(const Eigen::MatrixXd& sigma_points_in,
                          const Eigen::VectorXd& u, double dt,
                          Eigen::MatrixXd& sigma_points_out);
    void transformSigmaPoints(const Eigen::MatrixXd& sigma_points_in,
                            Eigen::MatrixXd& sigma_points_out);
    void computeWeights();
    double normalizeAngle(double angle) const;
};

#endif // UKF_ESTIMATOR_H
