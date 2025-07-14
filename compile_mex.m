% 编译UKF MEX模块的MATLAB脚本
% 在Windows MATLAB中运行此脚本

% 清理工作空间
clear; clc;

% 检查是否配置了编译器
try
    mex -setup C++
catch
    warning('请先配置MEX编译器: mex -setup C++');
end

% Eigen库路径
EIGEN_INCLUDE_PATH = 'C:\Users\ziyu.qiu\Documents\lib\eigen-3.4.0';

% 编译MEX文件
try
    mex(['-I' EIGEN_INCLUDE_PATH], 'truck_ukf_sfunc.cpp', 'semitrailer_dynamics.cpp', 'ukf_estimator.cpp')
    fprintf('编译成功! 生成了 truck_ukf_sfunc.%s\n', mexext);
catch ME
    fprintf('编译失败: %s\n', ME.message);
end
