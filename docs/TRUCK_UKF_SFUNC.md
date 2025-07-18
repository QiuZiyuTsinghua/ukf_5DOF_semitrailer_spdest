# truck_ukf_sfunc.cpp 说明文档

## 概述

`truck_ukf_sfunc.cpp` 是一个专为 MATLAB/Simulink 设计的 S-Function，它将半挂卡车UKF状态估计器集成到 Simulink 仿真环境中。该文件实现了标准的 MATLAB S-Function Level 2 接口，使得用户可以在 Simulink 模型中直接使用 UKF 算法进行实时状态估计。

## 功能特性

- **实时状态估计**: 使用无迹卡尔曼滤波器对半挂卡车5自由度状态进行实时估计
- **标准 S-Function 接口**: 完全兼容 MATLAB/Simulink 环境
- **可配置参数**: 支持13个车辆参数的动态配置
- **多信号输入输出**: 6个输入端口和6个输出端口
- **异常处理**: 包含完整的错误处理和数值稳定性检查

## 输入输出接口

### 输入端口 (6个)

| 端口 | 信号名称 | 单位 | 描述 |
|------|----------|------|------|
| 0 | 前轮转向角 | rad | 方向盘输入转换后的前轮转向角 |
| 1 | 驱动力 | N | 发动机产生的纵向驱动力 |
| 2 | 前轮转速 | rad/s | 前轮角速度传感器信号 |
| 3 | 后轮转速 | rad/s | 后轮角速度传感器信号 |
| 4 | 航向角速度 | rad/s | 陀螺仪测量的航向角速度 |
| 5 | 铰接角 | rad | 铰接角度传感器信号 |

### 输出端口 (6个)

| 端口 | 信号名称 | 单位 | 描述 |
|------|----------|------|------|
| 0 | 估计纵向速度 | m/s | UKF估计的牵引车纵向速度 Vx |
| 1 | 估计横向速度 | m/s | UKF估计的牵引车横向速度 Vy |
| 2 | 估计航向角速度 | rad/s | UKF估计的航向角速度 r |
| 3 | 估计铰接角 | rad | UKF估计的铰接角 ψ |
| 4 | 估计铰接角速度 | rad/s | UKF估计的铰接角速度 ψ̇ |
| 5 | 协方差迹 | - | 状态估计不确定度指标 |

### 参数设置 (13个)

| 参数 | 符号 | 单位 | 典型值 | 描述 |
|------|------|------|--------|------|
| P0 | mt | kg | 1800 | 牵引车质量 |
| P1 | ms | kg | 15000 | 挂车质量 |
| P2 | Izt | kg·m² | 3500 | 牵引车绕z轴转动惯量 |
| P3 | Izs | kg·m² | 80000 | 挂车绕z轴转动惯量 |
| P4 | a | m | 1.5 | 前桥到牵引车质心距离 |
| P5 | bt | m | 1.8 | 后桥到牵引车质心距离 |
| P6 | bs | m | 6.0 | 挂车轴到挂车质心距离 |
| P7 | Lh | m | 0.8 | 牵引销到牵引车质心距离 |
| P8 | Caf | N/rad | 100000 | 前桥轮胎侧偏刚度 |
| P9 | Car | N/rad | 150000 | 后桥轮胎侧偏刚度 |
| P10 | Cas | N/rad | 200000 | 挂车轮胎侧偏刚度 |
| P11 | kh | N·m/rad | 50000 | 铰接刚度 |
| P12 | ch | N·m·s/rad | 5000 | 铰接阻尼 |

## 编译方法

### Windows 平台

1. **准备环境**:
   - 安装 MATLAB R2018b 或更高版本
   - 配置 MEX 编译器: `mex -setup C++`
   - 下载并安装 Eigen 库

2. **编译步骤**:
   ```matlab
   cd matlab
   compile_mex
   ```

3. **验证编译**:
   编译成功后会生成 `truck_ukf_sfunc.mexw64` (Windows 64位) 文件

### Linux 平台

1. **安装依赖**:
   ```bash
   sudo apt-get install libeigen3-dev  # Ubuntu/Debian
   # 或
   sudo yum install eigen3-devel       # CentOS/RHEL
   ```

2. **编译步骤**:
   ```matlab
   cd matlab
   compile_mex
   ```

## 使用方法

### 在 Simulink 中使用

1. **添加 S-Function 块**:
   - 从 Simulink Library Browser 中拖拽 "S-Function" 块到模型
   - 双击打开参数对话框

2. **配置 S-Function**:
   - S-function name: `truck_ukf_sfunc`
   - S-function parameters: `[mt,ms,Izt,Izs,a,bt,bs,Lh,Caf,Car,Cas,kh,ch]`

3. **连接信号**:
   - 连接6个输入信号（转向角、驱动力、轮速等）
   - 连接6个输出信号到显示或记录模块

### MATLAB 脚本测试

```matlab
% 运行完整功能测试
cd matlab
test_sfunc

% 运行简化测试  
simple_test
```

## 算法配置

### UKF 参数设置

S-Function 内部使用以下 UKF 配置:

```cpp
// UKF 参数
alpha = 1e-3;  // Sigma点扩散参数
beta = 2.0;    // 高阶矩参数 (高斯分布最优值)
kappa = 0.0;   // 二级参数

// 过程噪声协方差
Q = diag([0.1, 0.1, 0.01, 0.01, 0.01]);

// 测量噪声协方差  
R = diag([0.5, 0.5, 0.01, 0.02]);
```

### 初始条件

```cpp
// 初始状态 [Vx, Vy, r, ψ, ψ̇]
x0 = [20.0, 0.0, 0.0, 0.0, 0.0];

// 初始协方差
P0 = diag([1.0, 1.0, 0.1, 0.1, 0.1]);
```

## 性能特性

- **采样时间**: 1ms (1000Hz)
- **计算复杂度**: O(n³) where n=5 (状态维度)
- **内存使用**: 约 ~50KB (包含所有矩阵存储)
- **实时性能**: 适合实时仿真和硬件在环测试

## 注意事项

1. **数值稳定性**: 
   - 包含完整的数值边界检查
   - 自动处理矩阵奇异性问题

2. **线程安全**: 
   - 每个 S-Function 实例维护独立状态
   - 支持多实例并行运行

3. **错误处理**:
   - 自动检测和恢复数值异常
   - 输出默认安全值在错误情况下

4. **兼容性**:
   - MATLAB R2018b+ 
   - Simulink Real-Time
   - dSPACE/Speedgoat 硬件平台

## 故障排除

### 常见编译错误

1. **"simstruc.h not found"**:
   - 确保 MATLAB/Simulink 正确安装
   - 检查 MEX 编译器配置

2. **"Eigen/Dense not found"**:
   - 安装 Eigen 库
   - 检查 `compile_mex.m` 中的路径设置

3. **链接错误**:
   - 确保所有源文件存在
   - 检查 C++ 编译器版本兼容性

### 运行时问题

1. **输出异常值**:
   - 检查输入信号范围和单位
   - 验证车辆参数设置
   - 调整过程/测量噪声参数

2. **性能问题**:
   - 降低采样频率
   - 优化 UKF 参数
   - 检查系统资源占用

## 扩展开发

如需修改算法或添加功能，请参考:

- `include/ukf_estimator.h` - UKF 算法接口
- `include/semitrailer_dynamics.h` - 车辆动力学模型
- `docs/DYNAMICS_AND_ESTIMATION.md` - 详细的数学模型

## 版本历史

- v1.0: 初始版本，基本 UKF S-Function 实现
- 支持半挂卡车5-DOF状态估计
- 兼容 MATLAB/Simulink 标准接口
