# 半挂卡车平面 5‑DOF 动力学模型

本文档给出 **牵引车‑半挂车（tractor‑semitrailer）** 的平面五自由度（5‑DOF）动力学模型推导、参数说明与 UKF 离散化接口，适用于速度/姿态联合估计与横向稳定性控制原型开发。

---

## 目录
1. [建模目标与假设](#section1)  
2. [坐标系与符号定义](#section2)  
3. [动力学方程推导](#section3)  
4. [轮胎侧向力模型](#section4)  
5. [离散化与 UKF 接口](#section5)  
6. [参数标定与辨识](#section6)  
7. [Simulink/Matlab 实现建议](#section7)  
8. [参考文献](#section8)

---

<a name="section1"></a>
## 1. 建模目标与假设

- **自由度**  
  - 牵引车质心纵向速度 \(V_x\)  
  - 牵引车质心横向速度 \(V_y\)  
  - 牵引车横摆角速度 \(r\)  
  - 铰接角 \(\psi\)（挂车与牵引车纵轴夹角，左正）  
  - 铰接角速度 \(\dot\psi\)

- **主要输入**  
  - 前桥转向角 \(\delta_f\)  
  - 牵引车驱动力 \(F_{x_\text{drive}}\)（如需纵向动力）

- **假设**  
  1. 车辆在水平面内运动，忽略侧倾、俯仰等高阶自由度。  
  2. 轮胎纵‑横耦合弱，侧向力由侧偏角单独决定。  
  3. 挂车转向架简化为单车桥并与牵引车后桥共面。  
  4. 前桥为可转向轮，其他车桥保持零转向角。  

---

<a name="section2"></a>
## 2. 坐标系与符号定义

![示意图](./pics/tractor_semi_coordinates.png)

| 符号 | 含义 | 方向（正向） |
|------|------|--------------|
| \(m_t, m_s\) | 牵引车、挂车质量 | – |
| \(I_{z_t}, I_{z_s}\) | 牵引车、挂车绕垂直轴转动惯量 | – |
| \(a, b_t\) | 牵引车前、后桥质心到 CG 距离 | \(+x\) |
| \(b_s\) | 挂车车桥到挂车 CG 距离 | \(+x\) |
| \(L_h\) | 牵引销到牵引车 CG 距离 | \(+x\) |
| \(C_{\alpha f}, C_{\alpha r}, C_{\alpha s}\) | 前、后、挂车桥侧偏刚度 | – |

---

<a name="section3"></a>
## 3. 动力学方程推导

### 3.1 牵引车动力学

质心平动：
\[
\begin{aligned}
m_t(\dot V_x - r V_y) &= F_{x_f}+F_{x_r} - F_{x,h} \\
m_t(\dot V_y + r V_x) &= F_{y_f}+F_{y_r} - F_{y,h}
\end{aligned}
\]

横摆：
\[
I_{z_t}\dot r = a F_{y_f} - b_t F_{y_r} - L_h F_{y,h}
\]

### 3.2 挂车动力学

令挂车质心速度在牵引车坐标系表示：
\[
\begin{aligned}
V_{x,s} &= V_x \cos\psi + (V_y + L_h r)\sin\psi \\
V_{y,s} &= -V_x \sin\psi + (V_y + L_h r)\cos\psi
\end{aligned}
\]

平动：
\[
m_s(\dot V_{y,s} + r_s V_{x,s}) = F_{y,sr} + F_{y,sf} + F_{y,h}
\]

横摆：
\[
I_{z_s}\dot r_s = -b_s F_{y,sr} + (L_h - b_s)F_{y,sf} + L_h F_{y,h}
\]

铰接角：
\[
\dot\psi = r_s - r
\]

其中 \(F_{y,h}\) 为牵引销横向力，可取
\[
F_{y,h} = k_h \psi + c_h \dot\psi
\]

### 3.3 矢量状态形式

设状态向量  
\[
\mathbf x = 
\begin{bmatrix}
V_x & V_y & r & \psi & \dot\psi
\end{bmatrix}^T
\]
则可写成  
\[
\dot{\mathbf x} = \mathbf f(\mathbf x, \mathbf u, \mathbf p) + \mathbf w
\]
并在 \(T_s\) 采样周期内使用四阶 Runge‑Kutta 离散化。

---

<a name="section4"></a>
## 4. 轮胎侧向力模型

#### 4.1 侧偏角
\[
\alpha_f = \delta_f - \frac{V_y + a r}{V_x}, \quad
\alpha_r = -\frac{V_y - b_t r}{V_x}, \quad
\alpha_s = \psi - \frac{V_{y,s} - b_s r_s}{V_{x,s}}
\]

#### 4.2 线性轮胎模型
\[
F_{y_i} = -C_{\alpha i}\, \alpha_i, \quad i \in \{f,r,s\}
\]

> 若需高精度，可替换为 Pacejka “魔术公式” 并限制在低纵向滑移区间。

---

<a name="section5"></a>
## 5. 离散化与 UKF 接口

1. **预测方程**  
   \[
   \mathbf x_{k+1} = \mathbf f_d(\mathbf x_k, \mathbf u_k) + \mathbf w_k
   \]
   过程噪声 \(\mathbf w_k\sim\mathcal N(0, \mathbf Q)\)。  
2. **观测方程**（示例）  
   \[
   \mathbf z_k =
   \begin{bmatrix}
   V_{x,\text{wheel}} \\ r_{\text{IMU}} \\ \psi_{\text{encoder}}
   \end{bmatrix}
   = \mathbf h(\mathbf x_k) + \mathbf v_k
   \]
   测量噪声 \(\mathbf v_k\sim\mathcal N(0, \mathbf R)\)。  
3. **采样点**：可用 Merwe scaled sigma‑points，参数 \(\alpha=10^{-3},\ \kappa=0,\ \beta=2\)。

---

<a name="section6"></a>
## 6. 参数标定与辨识

| 参数 | 典型值 (6×4 车) | 快速测量 | 精细测量 |
|------|-----------------|----------|----------|
| \(m_t\) | 8–11 t | 单轴称重 | IMU‑GNSS 联合估计 |
| \(m_s\) | 25–30 t (满载) | 过磅 | 载荷传感器 |
| \(I_{z_t}\) | 4–6×10³ kg·m² | 回正摆试验 | MBS 反向识别 |
| \(I_{z_s}\) | 2–4×10⁴ kg·m² | 蛇形试验 | MBS 反向识别 |
| \(C_{\alpha f,r}\) | 150–200 kN/rad | 推拉试验 | 轮胎试验机 |

---

<a name="section7"></a>
## 7. Simulink/Matlab 实现建议

1. **Subsystem** “Vehicle Dynamics”  
   - *Fcn* block 计算 \(\alpha_i\) 与 \(F_{y_i}\)  
   - *MATLAB Function* 实现状态微分  
   - *Integrator* 离散时间 RK4  
2. **UKF**：使用 *`unscentedKalmanFilter`*（R2021a 及以上）  
3. **测试工况**：双移线, ISO 3888‑2, 定稳圆周

---

<a name="section8"></a>
## 8. 参考文献

1. Rajamani, R. **Vehicle Dynamics and Control**, 2nd ed., Springer, 2012. Chapter 13.  
2. Xu, M. *et al.* “Modelling and Stability Analysis of Articulated Vehicles”, **Applied Sciences**, 2021.  
3. Kiefer, R. “Heavy Truck Modelling and Estimation”, PhD Thesis, UMich, 2014.  
4. SAE Paper 2017‑01‑1833. “A 5‑DOF Articulated EV Model for Lateral Control”.  
5. ISO 14791:2020. *Road Vehicles – Heavy Vehicle Combinations – Dynamic Test Methods*.  

---

> **许可证**：本文档采用 CC BY‑NC‑SA 4.0 共享许可。
