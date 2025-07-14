# UKF MEX 模块编译说明

## 在Windows上编译MEX模块

### 方法1: 使用MATLAB脚本 (推荐)
1. 将所有文件复制到Windows电脑上
2. 打开MATLAB
3. 切换到包含这些文件的目录
4. 运行: `compile_mex`

### 方法2: 使用批处理文件
1. 将所有文件复制到Windows电脑上
2. 双击运行 `compile_mex.bat`

### 方法3: 直接在MATLAB命令行
```matlab
mex truck_ukf_sfunc.cpp semitrailer_dynamics.cpp ukf_estimator.cpp
```

### 编译要求
- MATLAB R2015b或更高版本
- 配置好的C++编译器 (Visual Studio或MinGW)

### 编译成功后
会生成 `truck_ukf_sfunc.mexw64` (64位Windows) 或 `truck_ukf_sfunc.mexw32` (32位Windows)

## 使用MEX模块

### 在Simulink中使用
1. 打开Simulink模型
2. 添加一个 **S-Function** 模块 (Simulink > User-Defined Functions > S-Function)
3. 双击S-Function模块，设置参数：
   - **S-function name**: `truck_ukf_sfunc`
   - **S-function parameters**: `[牵引车质量, 挂车质量, 牵引车转动惯量, 挂车转动惯量, 前桥距离, 后桥距离, 挂车轴距离, 铰接距离, 前桥侧偏刚度, 后桥侧偏刚度, 挂车侧偏刚度, 采样时间]`
   
   **直接粘贴版本**: `8000, 25000, 5000, 30000, 1.4, 2.8, 7.0, 0.8, 180000, 300000, 400000, 0.01`
   
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
   - 采样时间: 0.01s (100Hz)

### 输入端口 (7个)
1. 轮速信号 (m/s)
2. 纵向加速度 ax (m/s²)
3. 横向加速度 ay (m/s²)
4. 航向角速度 (rad/s)
5. 油门开度 (0-1)
6. 制动压力 (0-1)
7. 方向盘转角 (rad)

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
  - delta_f: 前轮转角
  - F_drive: 驱动力 (从油门制动计算)
- **观测**: [轮速, 航向角速度, 铰接角, 纵向加速度]

### 注意事项
- 确保MEX文件在MATLAB路径中或当前工作目录中
- 采样时间建议设置为0.01s (100Hz)
- 参数根据实际车辆调整
