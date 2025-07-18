% compile_mex.m
% 编译UKF MEX模块的MATLAB脚本
% 支持Windows和Linux系统

% 清理工作空间
clear; clc;

fprintf('========================================\n');
fprintf('UKF半挂卡车状态估计器MEX编译脚本\n');
fprintf('UKF Semitrailer State Estimator MEX Compilation\n');
fprintf('========================================\n');

%% 检查编译器配置
fprintf('检查MEX编译器配置...\n');
try
    mex -setup C++
    fprintf('C++编译器配置成功\n');
catch
    warning('请先配置MEX编译器: mex -setup C++');
    fprintf('请运行以下命令配置编译器:\n');
    fprintf('>> mex -setup C++\n');
    return;
end

%% 检测系统和设置路径
if ispc
    fprintf('检测到Windows系统\n');
    % Windows下的常见Eigen路径
    possible_eigen_paths = {
        'C:\Users\ziyu.qiu\Documents\lib\eigen-3.4.0',
        'C:\dev\eigen',
        'C:\eigen',
        'C:\lib\eigen',
        'D:\eigen',
        getenv('EIGEN_ROOT')
    };
    
    % 查找Eigen库
    EIGEN_INCLUDE_PATH = '';
    for i = 1:length(possible_eigen_paths)
        if ~isempty(possible_eigen_paths{i}) && exist(possible_eigen_paths{i}, 'dir')
            eigen_header = fullfile(possible_eigen_paths{i}, 'Eigen', 'Dense');
            if exist(eigen_header, 'file')
                EIGEN_INCLUDE_PATH = possible_eigen_paths{i};
                fprintf('找到Eigen库: %s\n', EIGEN_INCLUDE_PATH);
                break;
            end
        end
    end
    
    if isempty(EIGEN_INCLUDE_PATH)
        fprintf('警告: 未找到Eigen库，请确保已安装并设置正确路径\n');
        fprintf('建议下载Eigen库到以下位置之一:\n');
        for i = 1:4
            fprintf('  %s\n', possible_eigen_paths{i});
        end
        fprintf('或设置环境变量EIGEN_ROOT\n');
        EIGEN_INCLUDE_PATH = input('请输入Eigen库路径 (或按Enter跳过): ', 's');
    end
    
elseif isunix
    fprintf('检测到Unix/Linux系统\n');
    % Linux下的常见Eigen路径
    possible_eigen_paths = {
        '/usr/include/eigen3',
        '/usr/local/include/eigen3', 
        '/opt/eigen',
        getenv('EIGEN_ROOT')
    };
    
    % 查找Eigen库
    EIGEN_INCLUDE_PATH = '';
    for i = 1:length(possible_eigen_paths)
        if ~isempty(possible_eigen_paths{i}) && exist(possible_eigen_paths{i}, 'dir')
            eigen_header = fullfile(possible_eigen_paths{i}, 'Eigen', 'Dense');
            if exist(eigen_header, 'file')
                EIGEN_INCLUDE_PATH = possible_eigen_paths{i};
                fprintf('找到Eigen库: %s\n', EIGEN_INCLUDE_PATH);
                break;
            end
        end
    end
    
    if isempty(EIGEN_INCLUDE_PATH)
        fprintf('警告: 未找到Eigen库\n');
        fprintf('在Ubuntu/Debian上安装: sudo apt-get install libeigen3-dev\n');
        fprintf('在CentOS/RHEL上安装: sudo yum install eigen3-devel\n');
        EIGEN_INCLUDE_PATH = input('请输入Eigen库路径 (或按Enter跳过): ', 's');
    end
else
    fprintf('未知系统，使用默认设置\n');
    EIGEN_INCLUDE_PATH = '';
end

%% 准备编译参数
fprintf('\n准备编译参数...\n');

% 基本编译参数
compile_args = {'-I../include'};

% 添加Eigen路径（如果找到）
if ~isempty(EIGEN_INCLUDE_PATH)
    compile_args{end+1} = ['-I' EIGEN_INCLUDE_PATH];
    fprintf('使用Eigen路径: %s\n', EIGEN_INCLUDE_PATH);
else
    fprintf('跳过Eigen路径设置\n');
end

% 源文件
source_files = {
    '../src/truck_ukf_sfunc.cpp',
    '../src/semitrailer_dynamics.cpp', 
    '../src/ukf_estimator.cpp'
};

% 检查源文件是否存在
fprintf('检查源文件...\n');
missing_files = {};
for i = 1:length(source_files)
    if ~exist(source_files{i}, 'file')
        missing_files{end+1} = source_files{i};
        fprintf('错误: 找不到文件 %s\n', source_files{i});
    else
        fprintf('✓ %s\n', source_files{i});
    end
end

if ~isempty(missing_files)
    fprintf('\n错误: 缺少源文件，无法编译\n');
    return;
end

%% 开始编译
fprintf('\n开始编译MEX文件...\n');
fprintf('编译命令: mex %s %s\n', strjoin(compile_args, ' '), strjoin(source_files, ' '));

try
    % 执行编译
    mex_args = [compile_args, source_files];
    mex(mex_args{:});
    
    % 检查编译结果
    mex_file = ['truck_ukf_sfunc.' mexext];
    if exist(mex_file, 'file')
        fprintf('\n🎉 编译成功!\n');
        fprintf('生成文件: %s\n', mex_file);
        
        % 显示文件信息
        file_info = dir(mex_file);
        fprintf('文件大小: %.1f KB\n', file_info.bytes/1024);
        fprintf('修改时间: %s\n', file_info.date);
        
        % 测试基本加载
        fprintf('\n测试MEX文件加载...\n');
        try
            clear(mex_file);  % 清除可能的缓存
            which(mex_file)
            fprintf('✓ MEX文件可以正常加载\n');
        catch ME_load
            fprintf('警告: MEX文件加载测试失败: %s\n', ME_load.message);
        end
        
        fprintf('\n可以运行 test_sfunc.m 进行功能测试\n');
        
    else
        fprintf('❌ 编译失败: 未生成MEX文件\n');
    end
    
catch ME
    fprintf('\n❌ 编译失败!\n');
    fprintf('错误信息: %s\n', ME.message);
    
    % 提供调试建议
    fprintf('\n调试建议:\n');
    fprintf('1. 确保已正确安装MEX编译器\n');
    fprintf('2. 确认Eigen库路径正确\n');
    fprintf('3. 检查源文件是否完整\n');
    fprintf('4. 在MATLAB命令窗口运行: mex -setup C++\n');
    
    if contains(ME.message, 'Eigen')
        fprintf('5. 安装Eigen库或正确设置路径\n');
    end
    
    if contains(ME.message, 'simstruc.h')
        fprintf('5. 确保MATLAB/Simulink已正确安装\n');
    end
end

fprintf('\n编译脚本执行完成\n');
