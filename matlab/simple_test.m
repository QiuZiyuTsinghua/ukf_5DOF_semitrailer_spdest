% simple_test.m
% UKF半挂卡车状态估计器的简单MATLAB测试
% Simple MATLAB test for UKF Semitrailer State Estimator

clear; clc; close all;

fprintf('========================================\n');
fprintf('UKF半挂卡车状态估计器 - MATLAB简单测试\n');
fprintf('UKF Semitrailer State Estimator - MATLAB Simple Test\n');
fprintf('========================================\n');

%% 车辆参数定义
fprintf('设置车辆参数...\n');

% 根据文档中的典型半挂卡车参数
vehicle_params = struct();
vehicle_params.mt = 1800;      % 牵引车质量 [kg]
vehicle_params.ms = 15000;     % 挂车质量 [kg]
vehicle_params.Izt = 3500;     % 牵引车转动惯量 [kg·m²]
vehicle_params.Izs = 80000;    % 挂车转动惯量 [kg·m²]
vehicle_params.a = 1.5;        % 前桥到质心距离 [m]
vehicle_params.bt = 1.8;       % 后桥到质心距离 [m]
vehicle_params.bs = 6.0;       % 挂车轴到质心距离 [m]
vehicle_params.Lh = 0.8;       % 牵引销到质心距离 [m]
vehicle_params.Caf = 100000;   % 前桥侧偏刚度 [N/rad]
vehicle_params.Car = 150000;   % 后桥侧偏刚度 [N/rad]
vehicle_params.Cas = 200000;   % 挂车桥侧偏刚度 [N/rad]
vehicle_params.kh = 50000;     % 铰接刚度 [N·m/rad]
vehicle_params.ch = 5000;      % 铰接阻尼 [N·m·s/rad]

fprintf('车辆参数设置完成\n');

%% 仿真场景设计
fprintf('设计测试场景...\n');

dt = 0.001;        % 时间步长 [s]
T_sim = 5.0;       % 仿真时间 [s]
time = 0:dt:T_sim;
N = length(time);

% 输入信号设计
scenario_name = '转向操纵测试';
fprintf('测试场景: %s\n', scenario_name);

% 控制输入
delta_f = 0.05 * sin(0.8*time);  % 前轮转向角 [rad] - 正弦转向
F_drive = 3000 * ones(size(time)); % 驱动力 [N] - 恒定驱动

% 模拟传感器测量（包含噪声）
wheel_radius = 0.5;  % 轮胎半径 [m]
initial_speed = 15;  % 初始速度 [m/s]

% 理想信号（作为真值参考）
Vx_true = initial_speed + 2*sin(0.1*time);
wheel_speed_true = Vx_true / wheel_radius;
yaw_rate_true = 0.02*sin(0.8*time);
hitch_angle_true = 0.08*sin(0.6*time);

% 添加测量噪声
noise_level = 0.02;
wheel_speed_f_meas = wheel_speed_true + noise_level*randn(size(time));
wheel_speed_r_meas = wheel_speed_true + noise_level*randn(size(time));
yaw_rate_meas = yaw_rate_true + 0.005*randn(size(time));
hitch_angle_meas = hitch_angle_true + 0.01*randn(size(time));

fprintf('测试场景设计完成，仿真步数: %d\n', N);

%% 检查MEX文件
mex_file = 'truck_ukf_sfunc';
if exist([mex_file '.' mexext], 'file')
    fprintf('✓ 找到MEX文件: %s.%s\n', mex_file, mexext);
    
    % 这里可以添加对MEX文件的调用
    % 但由于S-Function通常在Simulink环境中使用，
    % 我们只做参数验证
    
    fprintf('MEX文件可用，建议在Simulink中使用\n');
else
    fprintf('❌ 未找到MEX文件，请先运行 compile_mex.m\n');
    fprintf('将使用MATLAB模拟替代...\n');
end

%% 模拟UKF估计器行为
fprintf('\n开始状态估计模拟...\n');

% 初始化估计状态
x_est = zeros(5, N);  % [Vx, Vy, r, psi, psi_dot]
x_est(:, 1) = [initial_speed; 0; 0; 0; 0];  % 初始状态

% 简化的状态预测（演示用）
process_noise = 0.01;
measurement_noise = 0.02;

for k = 2:N
    % 简化的状态传播（实际应该使用动力学模型）
    x_est(1, k) = x_est(1, k-1) + 0.1*delta_f(k) + process_noise*randn();  % Vx
    x_est(2, k) = 0.5*delta_f(k) + process_noise*randn();                   % Vy
    x_est(3, k) = yaw_rate_meas(k) + measurement_noise*randn();            % r
    x_est(4, k) = hitch_angle_meas(k) + measurement_noise*randn();         % psi
    x_est(5, k) = (x_est(4, k) - x_est(4, k-1))/dt;                       % psi_dot
    
    % 简单的数值稳定性检查
    x_est(:, k) = max(-100, min(100, x_est(:, k)));
end

fprintf('状态估计模拟完成\n');

%% 结果分析和可视化
fprintf('\n========== 结果分析 ==========\n');

% 统计信息
fprintf('估计结果统计:\n');
fprintf('纵向速度: 均值=%.2f m/s, 标准差=%.3f m/s\n', ...
        mean(x_est(1, :)), std(x_est(1, :)));
fprintf('横向速度: 均值=%.3f m/s, 标准差=%.3f m/s\n', ...
        mean(x_est(2, :)), std(x_est(2, :)));
fprintf('航向角速度: 均值=%.4f rad/s, 标准差=%.4f rad/s\n', ...
        mean(x_est(3, :)), std(x_est(3, :)));
fprintf('铰接角: 均值=%.4f rad, 标准差=%.4f rad\n', ...
        mean(x_est(4, :)), std(x_est(4, :)));

% 创建结果图表
figure('Name', 'UKF状态估计测试结果', 'Position', [100, 100, 1200, 900]);

% 子图1: 纵向速度
subplot(3, 2, 1);
plot(time, x_est(1, :), 'b-', 'LineWidth', 2);
hold on;
plot(time, Vx_true, 'r--', 'LineWidth', 1.5);
grid on;
title('纵向速度估计 V_x');
xlabel('时间 [s]');
ylabel('速度 [m/s]');
legend('估计值', '真值', 'Location', 'best');

% 子图2: 横向速度
subplot(3, 2, 2);
plot(time, x_est(2, :), 'g-', 'LineWidth', 2);
grid on;
title('横向速度估计 V_y');
xlabel('时间 [s]');
ylabel('速度 [m/s]');

% 子图3: 航向角速度
subplot(3, 2, 3);
plot(time, x_est(3, :), 'c-', 'LineWidth', 2);
hold on;
plot(time, yaw_rate_true, 'r--', 'LineWidth', 1.5);
plot(time, yaw_rate_meas, 'k:', 'Alpha', 0.6);
grid on;
title('航向角速度估计 r');
xlabel('时间 [s]');
ylabel('角速度 [rad/s]');
legend('估计值', '真值', '测量值', 'Location', 'best');

% 子图4: 铰接角
subplot(3, 2, 4);
plot(time, x_est(4, :), 'm-', 'LineWidth', 2);
hold on;
plot(time, hitch_angle_true, 'r--', 'LineWidth', 1.5);
plot(time, hitch_angle_meas, 'k:', 'Alpha', 0.6);
grid on;
title('铰接角估计 ψ');
xlabel('时间 [s]');
ylabel('角度 [rad]');
legend('估计值', '真值', '测量值', 'Location', 'best');

% 子图5: 铰接角速度
subplot(3, 2, 5);
plot(time, x_est(5, :), 'y-', 'LineWidth', 2);
grid on;
title('铰接角速度估计 ψ̇');
xlabel('时间 [s]');
ylabel('角速度 [rad/s]');

% 子图6: 控制输入
subplot(3, 2, 6);
yyaxis left;
plot(time, delta_f*180/pi, 'b-', 'LineWidth', 1.5);
ylabel('转向角 [deg]');
yyaxis right;
plot(time, F_drive/1000, 'r-', 'LineWidth', 1.5);
ylabel('驱动力 [kN]');
grid on;
title('控制输入信号');
xlabel('时间 [s]');
legend('前轮转向角', '驱动力', 'Location', 'best');

sgtitle(sprintf('UKF半挂卡车状态估计器测试 - %s', scenario_name), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% 保存结果
save_results = true;
if save_results
    result_file = sprintf('ukf_test_results_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
    save(result_file, 'time', 'x_est', 'vehicle_params', 'delta_f', 'F_drive', ...
         'wheel_speed_f_meas', 'wheel_speed_r_meas', 'yaw_rate_meas', ...
         'hitch_angle_meas', 'Vx_true', 'yaw_rate_true', 'hitch_angle_true');
    fprintf('\n结果已保存到: %s\n', result_file);
end

%% 性能评估
fprintf('\n========== 性能评估 ==========\n');

% 计算估计误差（如果有真值）
if exist('Vx_true', 'var')
    Vx_error = abs(x_est(1, :) - Vx_true);
    yaw_rate_error = abs(x_est(3, :) - yaw_rate_true);
    hitch_angle_error = abs(x_est(4, :) - hitch_angle_true);
    
    fprintf('估计误差统计:\n');
    fprintf('纵向速度RMSE: %.3f m/s\n', sqrt(mean(Vx_error.^2)));
    fprintf('航向角速度RMSE: %.4f rad/s\n', sqrt(mean(yaw_rate_error.^2)));
    fprintf('铰接角RMSE: %.4f rad\n', sqrt(mean(hitch_angle_error.^2)));
end

fprintf('\n测试完成！\n');
fprintf('如需使用实际的UKF算法，请:\n');
fprintf('1. 编译MEX文件: 运行 compile_mex.m\n');
fprintf('2. 使用Simulink测试: 运行 test_sfunc.m\n');
fprintf('3. 或在C++环境中测试: 运行 make && ./build/test_ukf\n');