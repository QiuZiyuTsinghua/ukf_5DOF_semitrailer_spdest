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
   - **S-function parameters**: `[牵引车质量, 挂车质量, 牵引车转动惯量, 挂车转动惯量, 前桥距离, 后桥距离, 挂车轴距离, 铰接距离, 前桥侧偏刚度, 后桥侧偏刚度, 挂车侧偏刚度, 转向传动比, 采样时间]`
   
   **直接粘贴版本**: `8000, 25000, 5000, 30000, 1.4, 2.8, 7.0, 0.8, 180000, 300000, 400000, 20, 0.01`
   
   参数说明:
   - 牵引车质量: 8000kg
   - 挂车质量: 25000kg (满载)
   - 牵引车转动惯量: 5000kg·m²
   - 挂车转动惯量: 30000kg·m²
   - 前桥到质心距离: 1.4m
   - 后桥到质心距离: 2.8m
   - 挂车轴到质心距离: 7.0m
   - 铰接点到牵引车质心距离: 0.8m
   - 前桥侧偏刚度: 180000N/rad
   - 后桥侧偏刚度: 300000N/rad
   - 挂车桥侧偏刚度: 400000N/rad
   - 转向传动比: 20 (典型卡车值，设为1则输入为前轮转角)
   - 采样时间: 0.01s (100Hz)

### 输入端口 (6个)
1. 轮速信号 (m/s)
2. 纵向加速度 ax (m/s²)
3. 横向加速度 ay (m/s²)
4. 航向角速度 (rad/s)
5. 纵向驱动力 (N)
6. 转向角度 (rad) - 可以是方向盘转角或前轮转角

### 输出端口 (6个)
1. 估计纵向速度 Vx (m/s)
2. 估计横向速度 Vy (m/s)
3. 估计航向角速度 r (rad/s)
4. 估计铰接角 psi (rad)
5. 估计铰接角速度 psi_dot (rad/s)
6. 纵向速度不确定度 (m/s)

### 半挂卡车5-DOF模型说明
- **状态变量**: [Vx, Vy, r, psi, psi_dot]
  - Vx: 牵引车纵向速度
  - Vy: 牵引车横向速度  
  - r: 牵引车航向角速度
  - psi: 铰接角 (挂车与牵引车纵轴夹角)
  - psi_dot: 铰接角速度
- **控制输入**: [delta_f, F_drive]
  - delta_f: 前轮转角 (直接输入)
  - F_drive: 驱动力 (直接输入)
- **观测**: [轮速, 航向角速度, 铰接角, 纵向加速度]

### 注意事项
- 确保MEX文件在MATLAB路径中或当前工作目录中
- 采样时间建议设置为0.01s (100Hz)
- 参数根据实际车辆调整

### 转向系统配置
- **转向传动比 > 1**: 输入为方向盘转角，系统自动转换为前轮转角
  - 典型卡车转向传动比: 18-22
  - 前轮转角 = 方向盘转角 / 转向传动比
- **转向传动比 = 1**: 输入直接为前轮转角
- **转向传动比 ≤ 0.1**: 忽略转换，直接使用输入值作为前轮转角
