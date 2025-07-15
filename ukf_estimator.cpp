#include "ukf_estimator.h"
#include <cmath>

UKFEstimator::UKFEstimator() 
    : alpha_(0.001), beta_(2.0), kappa_(0.0),  // 更保守的alpha值
      n_x_(5), n_z_(4), n_aug_(7), dynamics_(NULL)
{
    lambda_ = alpha_ * alpha_ * (n_x_ + kappa_) - n_x_;
    
    // 初始化状态和协方差
    x_ = Eigen::VectorXd::Zero(n_x_);
    P_ = Eigen::MatrixXd::Identity(n_x_, n_x_) * 0.1;  // 更小的初始协方差
    
    // 初始化噪声协方差 - 更合理的值
    Q_ = Eigen::MatrixXd::Identity(n_x_, n_x_);
    Q_.diagonal() << 0.01, 0.01, 0.001, 0.001, 0.001;  // 更小的过程噪声
    
    R_ = Eigen::MatrixXd::Identity(n_z_, n_z_);
    R_.diagonal() << 0.1, 0.01, 0.01, 0.1;  // 更合理的测量噪声
    
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
    
    // 直接生成状态sigma点，不使用增广方法
    Eigen::MatrixXd sigma_points(n_x_, 2 * n_x_ + 1);
    generateSigmaPoints(x_, P_, sigma_points);
    
    // 预测sigma点
    Eigen::MatrixXd sigma_points_pred(n_x_, 2 * n_x_ + 1);
    sigma_points_pred.resize(n_x_, sigma_points.cols());
    
    for (int i = 0; i < sigma_points.cols(); i++) {
        Eigen::VectorXd x = sigma_points.col(i);
        
        // 使用动力学模型预测
        Eigen::VectorXd x_pred = dynamics_->integrate(x, control_input, dt);
        
        sigma_points_pred.col(i) = x_pred;
    }
    
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
    
    // 添加过程噪声
    P_ += Q_;
    
    // 确保协方差矩阵保持数值稳定性 / Ensure covariance matrix numerical stability
    // 检查并修复负对角元素
    for (int i = 0; i < P_.rows(); i++) {
        if (P_(i, i) < 1e-6) {
            P_(i, i) = 1e-6;
        }
    }
    
    // 强制对称性
    P_ = 0.5 * (P_ + P_.transpose());
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
    
    // 卡尔曼增益 - 添加强化的数值稳定性保护
    Eigen::MatrixXd S_inv;
    
    // 添加正则化项确保S是良条件的
    double reg_factor = 1e-3;  // 增加正则化因子
    S += reg_factor * Eigen::MatrixXd::Identity(n_z_, n_z_);
    
    // 使用更稳定的求逆方法
    Eigen::LDLT<Eigen::MatrixXd> ldlt_S(S);
    if (ldlt_S.info() == Eigen::Success && ldlt_S.isPositive()) {
        S_inv = ldlt_S.solve(Eigen::MatrixXd::Identity(n_z_, n_z_));
    } else {
        // 备用方案：使用SVD求伪逆
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        double tolerance = 1e-6 * svd.singularValues().maxCoeff();
        S_inv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(
            svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().transpose();
    }
    
    Eigen::MatrixXd K = Tc * S_inv;
    
    // 限制卡尔曼增益的大小，防止数值爆炸
    double max_gain = 10.0;
    for (int i = 0; i < K.rows(); i++) {
        for (int j = 0; j < K.cols(); j++) {
            if (std::abs(K(i,j)) > max_gain) {
                K(i,j) = (K(i,j) > 0) ? max_gain : -max_gain;
            }
        }
    }
    
    // 更新状态 - 添加状态限制
    Eigen::VectorXd z_diff = measurement - z_pred;
    z_diff(2) = normalizeAngle(z_diff(2));  // 铰接角差
    
    Eigen::VectorXd x_old = x_;  // 保存旧状态
    x_ += K * z_diff;
    
    // 检查状态更新是否合理，如果不合理则回退
    bool state_valid = true;
    for (int i = 0; i < x_.size(); i++) {
        if (std::isnan(x_(i)) || std::isinf(x_(i))) {
            state_valid = false;
            break;
        }
        // 检查状态变化是否过大
        if (std::abs(x_(i) - x_old(i)) > 100.0) {
            state_valid = false;
            break;
        }
    }
    
    if (!state_valid) {
        // 状态更新失败，回退到预测状态
        x_ = x_old;
        return;  // 跳过协方差更新
    }
    x_(2) = normalizeAngle(x_(2));
    x_(3) = normalizeAngle(x_(3));
    
    // 更新协方差 - 使用简单但稳定的形式
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_x_, n_x_);
    
    // 构建观测矩阵H (线性化近似)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_z_, n_x_);
    H(0, 0) = 1.0;  // 轮速 ≈ Vx
    H(1, 2) = 1.0;  // 航向角速度 ≈ r
    H(2, 3) = 1.0;  // 铰接角 ≈ psi
    H(3, 0) = 0.1;  // 纵向加速度与Vx相关（简化）
    
    // Joseph形式协方差更新
    Eigen::MatrixXd IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();
    
    // 强制协方差矩阵对称且数值稳定
    P_ = 0.5 * (P_ + P_.transpose());
    
    // 确保对角线元素有最小值，防止退化
    for (int i = 0; i < P_.rows(); i++) {
        if (P_(i, i) < 1e-6) {
            P_(i, i) = 1e-6;
        }
        if (P_(i, i) > 1e3) {  // 防止协方差过大
            P_(i, i) = 1e3;
        }
    }
}

void UKFEstimator::generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, 
                                      Eigen::MatrixXd& sigma_points)
{
    int n_dim = x.size();  // 使用输入向量的实际维度
    sigma_points.resize(n_dim, 2 * n_dim + 1);
    
    // 确保协方差矩阵是正定的 / Ensure covariance matrix is positive definite
    Eigen::MatrixXd P_safe = P;
    
    // 添加正则化项防止数值问题 / Add regularization to prevent numerical issues
    double reg = 1e-9;
    P_safe += reg * Eigen::MatrixXd::Identity(n_dim, n_dim);
    
    // 使用更稳定的Cholesky分解 / Use more stable Cholesky decomposition
    Eigen::LDLT<Eigen::MatrixXd> ldlt(P_safe);
    if (ldlt.info() != Eigen::Success) {
        // 如果LDLT分解失败，使用对角矩阵作为备选 / If LDLT fails, use diagonal matrix as fallback
        P_safe = P.diagonal().asDiagonal();
        P_safe += 0.01 * Eigen::MatrixXd::Identity(n_dim, n_dim);
    }
    
    // 计算平方根矩阵
    double lambda_local = (n_dim == n_x_) ? lambda_ : (alpha_ * alpha_ * (n_dim + kappa_) - n_dim);
    Eigen::MatrixXd A = ((n_dim + lambda_local) * P_safe).llt().matrixL();
    
    // 检查Cholesky分解是否成功 / Check if Cholesky decomposition succeeded
    if (A.hasNaN()) {
        // 备选方案：使用对角矩阵 / Fallback: use diagonal matrix
        A = std::sqrt(n_dim + lambda_local) * P_safe.diagonal().cwiseSqrt().asDiagonal();
    }
    
    // 第一个sigma点
    sigma_points.col(0) = x;
    
    // 其余sigma点
    for (int i = 0; i < n_dim; i++) {
        sigma_points.col(i + 1) = x + A.col(i);
        sigma_points.col(i + 1 + n_dim) = x - A.col(i);
    }
}

void UKFEstimator::generateAugmentedSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                                               const Eigen::VectorXd& u, double dt,
                                               Eigen::MatrixXd& sigma_points)
{
    // 增广状态: [x, noise_x]
    Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x;
    
    Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.topLeftCorner(n_x_, n_x_) = P;
    P_aug.bottomRightCorner(n_x_, n_x_) = Q_;
    
    // 为增广维度生成sigma点
    int n_dim = n_aug_;
    sigma_points.resize(n_dim, 2 * n_dim + 1);
    
    // 计算lambda用于增广维度
    double lambda_aug = alpha_ * alpha_ * (n_dim + kappa_) - n_dim;
    
    // 计算平方根矩阵
    Eigen::MatrixXd A = ((n_dim + lambda_aug) * P_aug).llt().matrixL();
    
    // 第一个sigma点
    sigma_points.col(0) = x_aug;
    
    // 其余sigma点
    for (int i = 0; i < n_dim; i++) {
        sigma_points.col(i + 1) = x_aug + A.col(i);
        sigma_points.col(i + 1 + n_dim) = x_aug - A.col(i);
    }
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
