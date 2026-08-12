# AGV-SFPPB-RL

本工程在原 AGV-TFS 的 Simulink 结构上实现：

\[
\boxed{\text{SFPPB}+\text{PI transformed error}+\text{ICAS-RL}}
\]

控制链为

```text
(e_y,e_phi) -> SFPPB/NMT -> z1 -> s1 -> ICAS-RL -> alpha1
            -> z2 -> s2 -> ICAS-RL -> delta -> sat(delta) -> AGV
```

PI 只重新定义 RL 使用的变换误差，不是第二个方向盘控制器。工程中没有 LQR、Safety Filter、残差 RL 或第二个转向通道。

## 运行

在 MATLAB 当前目录进入本目录后运行：

```matlab
scenario_name = 'Nominal curved path';
out = sim('AGV_simulate','StopTime','20');
run('AGV_plot.m');
```

`AGV_plot.m` 按原 AGV-TFS 风格生成 13 幅图，Figure 13 是由 `rho_0` 恢复的曲线路径和实际 AGV 轨迹。结果图保存在 `fig3`。

学习开关消融直接运行：

```matlab
run('AGV_learning_ablation.m');
```

脚本会比较 `learning_on=true/false`，并打印两种工况的误差、方向盘速率、总变差以及五组权重范数。

主模型 `AGV_simulate.slx` 保留 20 s 小曲率基准工况。真正的 U 形路径验证放在 `AGV_simulate_U.slx`：

```matlab
scenario_name = 'U-shaped path';
out = sim('AGV_simulate_U','ReturnWorkspaceOutputs','on');
run('AGV_plot.m');
```

U 形工况使用 `v_x=20 m/s`、`rho_0=0.003 1/m`，在 `10~62.36 s` 保持曲率，航向变化约为
`20*0.003*(62.36-10)=pi`，因此是完整的 180° 半圆掉头；总仿真时间为 70 s。对应结果图命名为
`fig3/sfppb_pi_u_01~13`。这组曲率对应半径约 333 m，既能形成清楚的 U 形，又不会产生瞬时大转向。

## 参数位置

不使用参数文件。所有控制器参数都在 `AGV_ctrl.m` 文件顶部：

```matlab
k1y = 0.10;   k1phi = 0.20;
k2y = 0.01;   k2phi = 0.01;
c1y = 5;      c1phi = 35;
c2y = 5;      c2phi = 8;
Upsilon2 = 0.04;
sigma2 = 0.08;
gamma_c1 = 0.004; gamma_c2 = 0.004;
gamma_a1 = 0.012; gamma_a2 = 0.012;
learning_on = true;              % false时冻结五组NN权重
u_d = 0.5;
```

`AGV_plant.m` 顶部还给出 `vx_vehicle` 和默认扰动幅值：横向加速度为
`0.5 m/s^2`，横摆角加速度为 `0.1 rad/s^2`。压力测试时再在命令行临时覆盖，不改默认物理工况。

控制器连续状态顺序为

```text
[WC1; WA1; WF2; WC2; WA2; O; alpha1_f; I1; I2]
```

因此状态数为 `10*N+8`。由于 `dot(z1)=varsigma*chi2-Gamma` 已由 NMT 和车辆运动学明确给出，第一层采用 `F1=0`，不再保留没有明确数学定义的 `WF1`。`alpha1_f` 是虚拟控制的一阶滤波状态，显式给出 `alpha1_f_dot`，PI 状态满足 `I1_dot=z1`、`I2_dot=z2`，并使用 `s1=z1+K1*I1`、`s2=z2+K2*I2`。

SFPPB 初始边界不对称系数在代码中写作 `nu_y`、`nu_phi`；柔性放宽状态只写作 `rho`，两者不混用。边界放宽系数写作 `lambda_lower_y/lambda_upper_y` 和 `lambda_lower_phi/lambda_upper_phi`，分别对应下界和上界。

`assist1.m` 的柔性状态参数写作 `k_rho`（衰减系数）和 `k_delta`（饱和超限增益），不再使用含义不清的 `p1/p2`。

第二层控制量明确使用 PI 导数项和饱和补偿项：

```matlab
F2_PI = F2_hat - alpha1_f_dot + O + K2.*z2;
p_a2 = 2*C2.*s2 + 2*F2_PI + WA2'*Phi_J2;
```

这里的 `+O` 由 `z2=chi2-alpha1_f-O` 直接求导得到。

Critic/Actor 不再只把 `s1/s2` 当成完整状态。当前网络输入显式包含 PI 积分状态：

```text
Z_J1 = [Z_F; z1; I1]
Z_J2 = [Z_F; z1; I1; z2; I2; O; alpha1_f]
```

这样不同的 `(z1,I1)` 或 `(z2,I2)` 不会被同一个 `s1/s2` 混成同一 Markov 状态；时间变化的性能边界仍需在完整 HJB 推导中显式处理。

## 物理输入增益

`AGV_ctrl.m` 和 `AGV_plant.m` 使用同一个物理输入增益：

```matlab
g_delta = [cf/m; lf*cf/Iz];
r_delta = norm([cf0/m; lf*cf0/Iz]);
delta = -(g_delta'*p_a2)/(2*r_delta);
```

本工程明确采用“已知时变输入增益”假设：

```text
dot(chi2) = F2(X,t) + g_delta(t)*delta_sat
F2(X,t)  = dot(chi2) - g_delta(t)*delta_sat
```

因此 Controller 和 Plant 使用同一个 `cf(t)` 与 `g_delta(t)`。`F2` 包含去掉实际转向输入后的轮胎漂移、外部扰动、道路曲率滤波状态及未建模耦合；`WF2` 只逼近这个 `F2`，不承担 `O`、PI 或虚拟控制滤波项。当前 `Z_F` 是可测的简化回归量；将所有未知参数和完整外部状态纳入严格闭合的 Identifier 仍属于 `THEORY GAP`。

曲线路径的第 13 路输入是 `rho_0`，控制器增加简单的车辆曲率前馈
`delta_feedforward = rho_ff_gain*rho_0`，当前默认 `rho_ff_gain=5.0`；直线路径时该项为零。

控制器输出 `delta` 和唯一一次饱和后的 `delta_sat`，Simulink 将 `delta_sat` 送入 plant，plant 不再重复设置第二个饱和上限。

`assist1.m` 的 `rho_dot` 使用连续滤波状态输出，当前模型代数环诊断为 0 个环。

当前第二层用一阶滤波状态 `alpha1_f` 显式处理 `-dot(alpha1_f)`，避免直接把未建模的 `dot(alpha1)` 当成零。

## 结果

结构审计后的基准工况（`u_d=0.5`，20 s，默认物理扰动）结果为：

```text
RMS(e_y)       = 0.0142989 m
RMS(e_phi)     = 0.00125282 rad
RMS_delta_dot  = 0.407808 rad/s
TV_delta       = 0.479402 rad
T_sat          = 0 s
```

本轮在结构审计后重新比较了滤波和曲率前馈参数，当前默认使用 `tau_alpha1=0.015`、`rho_ff_gain=5.0`；其余 PI、RL 和饱和补偿参数保持代码顶部默认值。

饱和验证工况可在 MATLAB 中设置唯一的 `u_d`（并按需设置压力扰动）：

```matlab
global u_d disturbance_y_amplitude disturbance_phi_amplitude
u_d = 0.3;
disturbance_y_amplitude = 18;
disturbance_phi_amplitude = 18;
out = sim('AGV_simulate','StopTime','20','ReturnWorkspaceOutputs','on');
```

恢复默认工况时执行 `clear global u_d disturbance_y_amplitude disturbance_phi_amplitude` 后重新运行。

这是单独的压力测试背景：两个扰动数值虽然都写成 18，但在代码中分别对应 `m/s^2` 和 `rad/s^2`，默认工况不会使用它们。
当前 `u_d=0.3` 压力测试完整通过，结构审计后的饱和持续约 `1.36065 s`，结果图为 `fig3/sfppb_pi_sat03_01~13`。

```text
结果状态          = Pass（无 BoundaryViolation）
RMS(e_y)          ≈ 0.0927116 m
RMS(e_phi)        ≈ 0.0129173 rad
T_sat             ≈ 1.36065 s
min boundary gap  ≈ 0.0782158
```

U 形工况的记录结果为 `RMS(e_y)=0.0243367 m`、`RMS(e_phi)=0.00160697 rad`、`T_sat=0 s`，无边界越界；Figure 13 中红色虚线是参考半圆，蓝色实线是实际 AGV 轨迹。

默认代码已恢复为 `u_d=0.5`。`AGV_plot.m` 的 Figure 12 现在绘制真正的柔性辅助状态 `rho`，Figure 10 单独绘制道路曲率 `rho_0`。

## 文件结构

- `AGV_ctrl.m`：PI、第二层 Identifier、两层 Critic/Actor、O 补偿和唯一方向盘控制律；第一层已采用 `F1=0`。
- `AGV_transfor.m`：SFPPB 边界、边界导数、NMT 和 `Gamma`。
- `assist1.m`：输入饱和补偿状态 `rho`，同时输出经过连续滤波的 `rho_dot` 给 SFPPB。
- `AGV_plant.m`：标准 `[e_y,e_phi,v_y,omega_z]` 横向动力学模型，输入为已经饱和的 `delta_sat`，并用一阶曲率状态避免代数环。
- `AGV_plot.m`：时域结果图、`rho` 柔性状态图和 Figure 13 曲线路径图。
- `AGV_learning_ablation.m`：learning ON/OFF 消融和五组权重范数诊断。
- `AGV_simulate.slx`：原模型结构，增加了 PI 诊断记录通道、`rho/rho_dot` 记录通道，并将 `rho_0` 接入车辆参考曲率输入。
- `AGV_simulate_U.slx`：真正 U 形路径验证模型，使用 70 s 仿真和 180° 半圆参考曲率。

## 当前验证边界

本轮已完成物理输入、plant 接口、代数环、曲率前馈和诊断结构对齐；名义、压力饱和和 U 形三种仿真均无边界越界。第一层采用由 NMT 直接得到的 `F1=0`。完整的双通道 PI-RL composite Lyapunov 推导、时变边界的 HJB 状态扩展和严格的欠驱动稳定性证明仍标记为 `THEORY GAP`，详见 [`docs/theory/AGV_SFPPB_PI_RL_stability_notes.md`](docs/theory/AGV_SFPPB_PI_RL_stability_notes.md)。
