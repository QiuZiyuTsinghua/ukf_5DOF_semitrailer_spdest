#include "ukf_estimator.h"
#include <cmath>

UKFEstimator::UKFEstimator() 
    : alpha_(1e-3), beta_(2.0), kappa_(0.0),
      n_x_(5), n_z_(4), n_aug_(7), dynamics_(NULL)
{
    lambda_ = alpha_ * alpha_ * (n_x_ + kappa_) - n_x_;
    
    // 初始化状态和协方差
    x_ = Eigen::VectorXd::Zero(n_x_);
    P_ = Eigen::MatrixXd::Identity(n_x_, n_x_);
    
    // 初始化噪声协方差
    Q_ = Eigen::MatrixXd::Identity(n_x_, n_x_) * 0.1;
    R_ = Eigen::MatrixXd::Identity(n_z_, n_z_) * 0.1;
    
    computeWeights();
}

void UKFEstimator::initialize(const Eigen::VectorXd& initial_state, 
                             const Eigen::MatrixXd& initial_covariance)
{
    x_ = initial_state;
    P_ = initial_covariance;
}

void UKFEstimator::setProcessNoise(const Eigen::MatrixXd& Q)
{
    Q_ = Q;
}

void UKFEstimator::setMeasurementNoise(const Eigen::MatrixXd& R)
{
    R_ = R;
}

void UKFEstimator::setUKFParameters(double alpha, double beta, double kappa)
{
    alpha_ = alpha;
    beta_ = beta;
    kappa_ = kappa;
    lambda_ = alpha_ * alpha_ * (n_x_ + kappa_) - n_x_;
    computeWeights();
}

void UKFEstimator::setDynamicsModel(SemitrailerDynamics* dynamics)
{
    dynamics_ = dynamics;
}

void UKFEstimator::computeWeights()
{
    int n_sigma = 2 * n_x_ + 1;
    weights_m_ = Eigen::VectorXd(n_sigma);
    weights_c_ = Eigen::VectorXd(n_sigma);
    
    double weight_0 = lambda_ / (n_x_ + lambda_);
    weights_m_(0) = weight_0;
    weights_c_(0) = weight_0 + (1 - alpha_ * alpha_ + beta_);
    
    for (int i = 1; i < n_sigma; i++) {
        double weight = 0.5 / (n_x_ + lambda_);
        weights_m_(i) = weight;
        weights_c_(i) = weight;
    }
}

void UKFEstimator::predict(const Eigen::VectorXd& control_input, double dt)
{
    if (dynamics_ == NULL) {
        // 静默失败，适合嵌入式平台
        return;
    }
    
    // 生成增广sigma点
    Eigen::MatrixXd sigma_points_aug(n_aug_, 2 * n_aug_ + 1);
    generateAugmentedSigmaPoints(x_, P_, control_input, dt, sigma_points_aug);
    
    // 预测sigma点
    Eigen::MatrixXd sigma_points_pred(n_x_, 2 * n_x_ + 1);
    predictSigmaPoints(sigma_points_aug, control_input, dt, sigma_points_pred);
    
    // 计算预测均值
    x_.setZero();
    for (int i = 0; i < 2 * n_x_ + 1; i++) {
        x_ += weights_m_(i) * sigma_points_pred.col(i);
    }
    
    // 归一化角度
    x_(2) = normalizeAngle(x_(2));  // 航向角速度
    x_(3) = normalizeAngle(x_(3));  // 铰接角
    
    // 计算预测协方差
    P_.setZero();
    for (int i = 0; i < 2 * n_x_ + 1; i++) {
        Eigen::VectorXd x_diff = sigma_points_pred.col(i) - x_;
        x_diff(2) = normalizeAngle(x_diff(2));
        x_diff(3) = normalizeAngle(x_diff(3));
        P_ += weights_c_(i) * x_diff * x_diff.transpose();
    }
}

void UKFEstimator::update(const Eigen::VectorXd& measurement)
{
    if (dynamics_ == NULL) {
        // 静默失败，适合嵌入式平台
        return;
    }
    
    // 生成sigma点
    Eigen::MatrixXd sigma_points(n_x_, 2 * n_x_ + 1);
    generateSigmaPoints(x_, P_, sigma_points);
    
    // 变换sigma点到观测空间
    Eigen::MatrixXd sigma_points_z(n_z_, 2 * n_x_ + 1);
    transformSigmaPoints(sigma_points, sigma_points_z);
    
    // 计算预测观测均值
    Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(n_z_);
    for (int i = 0; i < 2 * n_x_ + 1; i++) {
        z_pred += weights_m_(i) * sigma_points_z.col(i);
    }
    
    // 计算观测协方差
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_z_, n_z_);
    for (int i = 0; i < 2 * n_x_ + 1; i++) {
        Eigen::VectorXd z_diff = sigma_points_z.col(i) - z_pred;
        z_diff(2) = normalizeAngle(z_diff(2));  // 铰接角
        S += weights_c_(i) * z_diff * z_diff.transpose();
    }
    S += R_;
    
    // 计算交叉协方差
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_);
    for (int i = 0; i < 2 * n_x_ + 1; i++) {
        Eigen::VectorXd x_diff = sigma_points.col(i) - x_;
        x_diff(2) = normalizeAngle(x_diff(2));
        x_diff(3) = normalizeAngle(x_diff(3));
        
        Eigen::VectorXd z_diff = sigma_points_z.col(i) - z_pred;
        z_diff(2) = normalizeAngle(z_diff(2));
        
        Tc += weights_c_(i) * x_diff * z_diff.transpose();
    }
    
    // 卡尔曼增益
    Eigen::MatrixXd K = Tc * S.inverse();
    
    // 更新状态
    Eigen::VectorXd z_diff = measurement - z_pred;
    z_diff(2) = normalizeAngle(z_diff(2));  // 铰接角差
    
    x_ += K * z_diff;
    x_(2) = normalizeAngle(x_(2));
    x_(3) = normalizeAngle(x_(3));
    
    // 更新协方差
    P_ -= K * S * K.transpose();
}

void UKFEstimator::generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, 
                                      Eigen::MatrixXd& sigma_points)
{
    sigma_points.resize(n_x_, 2 * n_x_ + 1);
    
    // 计算平方根矩阵
    Eigen::MatrixXd A = ((n_x_ + lambda_) * P).llt().matrixL();
    
    // 第一个sigma点
    sigma_points.col(0) = x;
    
    // 其余sigma点
    for (int i = 0; i < n_x_; i++) {
        sigma_points.col(i + 1) = x + A.col(i);
        sigma_points.col(i + 1 + n_x_) = x - A.col(i);
    }
}

void UKFEstimator::generateAugmentedSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                                               const Eigen::VectorXd& u, double dt,
                                               Eigen::MatrixXd& sigma_points)
{
    // 增广状态: [x, noise_x, noise_z]
    Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x;
    
    Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P;
    P_aug.bottomRightCorner(n_x_, n_x_) = Q_;
    
    generateSigmaPoints(x_aug, P_aug, sigma_points);
}

void UKFEstimator::predictSigmaPoints(const Eigen::MatrixXd& sigma_points_in,
                                     const Eigen::VectorXd& u, double dt,
                                     Eigen::MatrixXd& sigma_points_out)
{
    sigma_points_out.resize(n_x_, sigma_points_in.cols());
    
    for (int i = 0; i < sigma_points_in.cols(); i++) {
        Eigen::VectorXd x = sigma_points_in.col(i).head(n_x_);
        Eigen::VectorXd noise = sigma_points_in.col(i).segment(n_x_, n_x_);
        
        // 使用动力学模型预测
        Eigen::VectorXd x_pred = dynamics_->integrate(x, u, dt);
        
        // 添加过程噪声
        sigma_points_out.col(i) = x_pred + noise;
    }
}

void UKFEstimator::transformSigmaPoints(const Eigen::MatrixXd& sigma_points_in,
                                       Eigen::MatrixXd& sigma_points_out)
{
    sigma_points_out.resize(n_z_, sigma_points_in.cols());
    
    for (int i = 0; i < sigma_points_in.cols(); i++) {
        Eigen::VectorXd x = sigma_points_in.col(i);
        sigma_points_out.col(i) = dynamics_->observationModel(x);
    }
}

double UKFEstimator::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::VectorXd UKFEstimator::getStateUncertainty() const
{
    Eigen::VectorXd uncertainty(n_x_);
    for (int i = 0; i < n_x_; i++) {
        uncertainty(i) = std::sqrt(P_(i, i));
    }
    return uncertainty;
}
