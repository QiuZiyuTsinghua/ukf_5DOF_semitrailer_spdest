# UKF半挂卡车状态估计器 - 项目目录结构

## 目录组织

```
ukf_simple/
├── include/                    # 头文件目录
│   ├── semitrailer_dynamics.h # 车辆动力学模型头文件
│   └── ukf_estimator.h        # UKF估计器头文件
├── src/                       # 源代码目录
│   ├── semitrailer_dynamics.cpp # 车辆动力学模型实现
│   ├── ukf_estimator.cpp      # UKF估计器实现
│   └── truck_ukf_sfunc.cpp    # MATLAB S-Function接口
├── tests/                     # 测试程序目录
│   ├── main.cpp              # 主测试程序（详细仿真）
│   ├── simple_test.cpp       # 简化测试程序
│   ├── minimal_test.cpp      # 最小测试程序
│   └── test_sfunc_standalone.cpp # S-Function独立测试
├── matlab/                    # MATLAB相关文件
│   ├── compile_mex.m         # MATLAB编译脚本
│   ├── compile_mex.bat       # Windows批处理编译脚本
│   ├── simple_test.m         # MATLAB测试脚本
│   └── test_sfunc.m          # S-Function测试脚本
├── docs/                      # 文档目录
│   ├── DYNAMICS_AND_ESTIMATION.md # 动力学建模与估计原理
│   ├── PROJECT_STRUCTURE.md   # 原项目结构说明
│   └── tractor_semitrailer_5DOF_model.md # 车辆模型详细说明
├── build/                     # 编译输出目录
├── Makefile                   # Make构建文件
└── README.md                  # 项目说明文档
```

## 模块说明

### 核心算法模块 (include/ + src/)

#### 1. 车辆动力学模型 (`semitrailer_dynamics.*`)
- **功能**: 实现半挂卡车平面5-DOF动力学模型
- **状态变量**: [Vx, Vy, r, psi, psi_dot]
  - Vx: 牵引车纵向速度 (m/s)
  - Vy: 牵引车横向速度 (m/s)
  - r: 牵引车航向角速度 (rad/s)
  - psi: 铰接角 (rad)
  - psi_dot: 铰接角速度 (rad/s)
- **控制输入**: [delta_f, F_drive] (前轮转角, 驱动力)
- **主要方法**:
  - `stateDerivative()`: 状态微分方程
  - `integrate()`: 四阶龙格库塔积分
  - `observationModel()`: 观测方程

#### 2. UKF估计器 (`ukf_estimator.*`)
- **功能**: 无迹卡尔曼滤波器实现
- **特性**: Merwe scaled sigma-points方法
- **主要方法**:
  - `predict()`: 预测步骤
  - `update()`: 更新步骤
  - `getState()`: 获取估计状态
  - `getStateUncertainty()`: 获取状态不确定度

#### 3. S-Function接口 (`truck_ukf_sfunc.cpp`)
- **功能**: MATLAB/Simulink集成接口
- **输入**: 6个信号端口
- **输出**: 6个估计状态端口
- **参数**: 13个车辆和算法参数

### 测试程序 (tests/)

#### 1. 主测试程序 (`main.cpp`)
- **特点**: 详细的仿真测试，包含完整的调试输出
- **功能**: 200步仿真，1ms时间步长
- **输出**: 详细的状态信息、误差分析、执行时间统计

#### 2. 简化测试程序 (`simple_test.cpp`)
- **特点**: 基本功能验证，包含超时保护
- **功能**: 快速验证UKF算法正确性
- **适用**: 快速调试和验证

#### 3. 最小测试程序 (`minimal_test.cpp`)
- **特点**: 最简单的单步测试
- **功能**: 验证基本的预测和更新功能
- **适用**: 单元测试和基础验证

### MATLAB集成 (matlab/)

#### 1. 编译脚本
- `compile_mex.m`: MATLAB环境下的编译脚本
- `compile_mex.bat`: Windows批处理自动编译

#### 2. 测试脚本
- `simple_test.m`: MATLAB环境下的简单测试
- `test_sfunc.m`: S-Function功能测试

### 文档 (docs/)
- 详细的技术文档和模型说明
- 算法原理和实现细节
- 使用指南和参数配置

## 编译指南

### 使用Makefile (Linux/macOS)
```bash
# 编译主测试程序
make

# 编译所有测试程序
make all-tests

# 运行测试
make run
make run-simple
make run-minimal

# 清理
make clean
```

### 使用MATLAB (Windows)
```matlab
% 进入matlab目录
cd matlab

% 运行编译脚本
compile_mex
```

### 直接编译
```bash
# 编译主测试程序
g++ -std=c++11 -Iinclude -I/usr/include/eigen3 -O2 \
    tests/main.cpp src/semitrailer_dynamics.cpp src/ukf_estimator.cpp \
    -o build/test_ukf
```

## 开发指南

### 添加新的测试程序
1. 在`tests/`目录下创建新的.cpp文件
2. 包含头文件: `#include "../include/semitrailer_dynamics.h"`
3. 更新Makefile添加新的编译目标

### 修改算法参数
- 车辆参数: 在测试程序中修改`setVehicleParameters()`调用
- UKF参数: 使用`setUKFParameters()`方法
- 噪声参数: 使用`setProcessNoise()`和`setMeasurementNoise()`

### 集成到Simulink
1. 编译MEX文件: 运行`matlab/compile_mex.m`
2. 在Simulink中添加S-Function块
3. 设置S-function name为`truck_ukf_sfunc`
4. 配置13个车辆参数

## 依赖要求

- **C++**: C++11标准或更高
- **Eigen3**: 线性代数库 (Ubuntu: `sudo apt install libeigen3-dev`)
- **MATLAB**: R2015b或更高版本 (仅用于MEX编译)
- **编译器**: GCC 4.9+ 或 Visual Studio 2015+
