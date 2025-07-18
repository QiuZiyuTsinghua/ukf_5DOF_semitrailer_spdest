# UKF半挂卡车状态估计器

## 项目简介

本项目实现了基于无迹卡尔曼滤波（UKF）的半挂卡车5自由度状态估计器，支持C++独立运行和MATLAB/Simulink集成。

## 目录结构

```
ukf_simple/
├── include/           # 头文件
├── src/              # 源代码
├── tests/            # 测试程序  
├── matlab/           # MATLAB集成
├── docs/             # 技术文档
├── build/            # 编译输出
└── Makefile          # 构建文件
```

详细结构说明请参考 `PROJECT_STRUCTURE_NEW.md`

## 快速开始

### C++测试程序编译与运行

```bash
# 编译主测试程序
make

# 运行测试
make run

# 编译所有测试程序
make all-tests

# 运行不同的测试
make run-simple    # 简化测试
make run-minimal   # 最小测试
```

### MATLAB MEX模块编译

#### 在Windows上编译MEX模块

**方法1: 使用MATLAB脚本 (推荐)**
1. 将项目文件复制到Windows电脑上
2. 打开MATLAB，切换到项目根目录
3. 运行: `cd matlab; compile_mex`

**方法2: 使用批处理文件**
1. 双击运行 `matlab/compile_mex.bat`

**方法3: 直接在MATLAB命令行**
```matlab
cd matlab
mex -I../include ../src/truck_ukf_sfunc.cpp ../src/semitrailer_dynamics.cpp ../src/ukf_estimator.cpp
```

### 编译要求
- MATLAB R2015b或更高版本
- 配置好的C++编译器 (Visual Studio或MinGW)
- Eigen3库（路径需在compile_mex.m中配置）

### 编译成功后
会生成 `truck_ukf_sfunc.mexw64` (64位Windows) 或 `truck_ukf_sfunc.mexw32` (32位Windows)

## MATLAB/Simulink集成使用

### 在Simulink中使用
1. 打开Simulink模型
2. 添加一个 **S-Function** 模块 (Simulink > User-Defined Functions > S-Function)
3. 双击S-Function模块，设置参数：
   - **S-function name**: `truck_ukf_sfunc`
   - **S-function parameters**: `[采样时间, 过程噪声向量(5维), 测量噪声向量(4维)]`
   
   **直接粘贴版本**: `0.01, [0.01, 0.01, 0.001, 0.001, 0.001], [0.1, 0.01, 0.01, 0.1]`
   
   参数说明:
   - **采样时间**: 0.01s (100Hz控制频率)
   - **过程噪声向量** (5维): [Vx, Vy, r, psi, psi_dot] 的过程噪声方差
     - Vx噪声方差: 0.01 (m/s)²
     - Vy噪声方差: 0.01 (m/s)²
     - r噪声方差: 0.001 (rad/s)²
     - psi噪声方差: 0.001 (rad)²
     - psi_dot噪声方差: 0.001 (rad/s)²
   - **测量噪声向量** (4维): [Vx_meas, r_meas, psi_meas, psi_dot_meas] 的测量噪声方差
     - Vx测量噪声方差: 0.1 (m/s)²
     - r测量噪声方差: 0.01 (rad/s)²
     - psi测量噪声方差: 0.01 (rad)²
     - psi_dot测量噪声方差: 0.1 (rad/s)²

### 车辆参数配置
S-Function内部使用固定的典型半挂卡车参数:
   - 牵引车质量: 6000kg
   - 挂车质量: 10000kg
   - 牵引车转动惯量: 12000kg·m²
   - 挂车转动惯量: 80000kg·m²
   - 前桥到质心距离: 1.4m
   - 后桥到质心距离: 1.6m
   - 挂车轴到质心距离: 6.0m
   - 铰接点到牵引车质心距离: 0.5m
   - 前桥侧偏刚度: 80000N/rad
   - 后桥侧偏刚度: 160000N/rad
   - 挂车桥侧偏刚度: 120000N/rad
   - 铰接刚度: 5000N·m/rad
   - 铰接阻尼: 2000N·m·s/rad

### 输入端口 (3个)
1. **控制输入** (2维): [delta_f, F_drive]
   - delta_f: 前轮转角 (rad)
   - F_drive: 驱动力 (N)
2. **测量输入** (4维): [Vx_meas, r_meas, psi_meas, psi_dot_meas]
   - Vx_meas: 测量的纵向速度 (m/s)
   - r_meas: 测量的横摆角速度 (rad/s)
   - psi_meas: 测量的航向角/铰接角 (rad)
   - psi_dot_meas: 测量的航向角速度/铰接角速度 (rad/s)
3. **初始状态** (5维): [Vx, Vy, r, psi, psi_dot]
   - 仅在滤波器初始化时使用

### 输出端口 (2个)
1. **估计状态** (5维): [Vx_est, Vy_est, r_est, psi_est, psi_dot_est]
   - Vx_est: 估计纵向速度 (m/s)
   - Vy_est: 估计横向速度 (m/s)
   - r_est: 估计横摆角速度 (rad/s)
   - psi_est: 估计铰接角 (rad)
   - psi_dot_est: 估计铰接角速度 (rad/s)
2. **状态不确定度** (5维): 各状态估计的标准差 (相同单位)

### 半挂卡车5-DOF模型说明
- **状态变量**: [Vx, Vy, r, psi, psi_dot]
  - Vx: 牵引车纵向速度 (m/s)
  - Vy: 牵引车横向速度 (m/s)
  - r: 牵引车横摆角速度 (rad/s)
  - psi: 铰接角 (挂车与牵引车纵轴夹角) (rad)
  - psi_dot: 铰接角速度 (rad/s)
- **控制输入**: [delta_f, F_drive]
  - delta_f: 前轮转角 (rad)
  - F_drive: 驱动力 (N)
- **观测**: [Vx_meas, r_meas, psi_meas, psi_dot_meas]
  - Vx_meas: 测量纵向速度 (m/s)
  - r_meas: 测量横摆角速度 (rad/s)
  - psi_meas: 测量铰接角 (rad)
  - psi_dot_meas: 测量铰接角速度 (rad/s)

### UKF算法配置
- **UKF参数**: alpha=1e-3, beta=2.0, kappa=0.0
- **状态维度**: 5 (Vx, Vy, r, psi, psi_dot)
- **观测维度**: 4 (Vx_meas, r_meas, psi_meas, psi_dot_meas)
- **控制输入维度**: 2 (delta_f, F_drive)

### 噪声参数调优建议
- **过程噪声**: 反映模型不确定性，值越大表示对模型信任度越低
  - 速度相关 (Vx, Vy): 0.01-0.1
  - 角速度相关 (r, psi_dot): 0.001-0.01
  - 角度相关 (psi): 0.001-0.01
- **测量噪声**: 反映传感器精度，应根据实际传感器规格设定
  - GPS速度: 0.1-1.0
  - IMU角速度: 0.01-0.1
  - 角度传感器: 0.01-0.1

### 注意事项
- 确保MEX文件在MATLAB路径中或当前工作目录中
- 采样时间建议设置为0.01s (100Hz)，可根据实际系统调整
- 车辆参数在S-Function中硬编码，如需修改请重新编译
- 初始状态应根据车辆实际状态设定，建议静止时设为 [当前速度, 0, 0, 0, 0]
- 过程噪声和测量噪声参数需根据实际系统特性调优

### 使用示例
```matlab
% 在MATLAB中设置S-Function参数
dt = 0.01;  % 采样时间
Q_diag = [0.01, 0.01, 0.001, 0.001, 0.001];  % 过程噪声
R_diag = [0.1, 0.01, 0.01, 0.1];  % 测量噪声
sfunc_params = [dt, Q_diag, R_diag];
```
