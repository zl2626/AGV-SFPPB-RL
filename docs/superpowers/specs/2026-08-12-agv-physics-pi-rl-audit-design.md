# AGV 物理模型与 PI-RL 理论闭合设计

## 目标

在原 AGV-TFS 的 Level-1 MATLAB S-function 和 Simulink 结构上，逐项修正车辆动力学、SFPPB 辅助状态、饱和补偿、第一层/第二层 Identifier 以及 PI-RL 状态定义。控制器始终只有一个真实转向输入 `delta`，不加入 LQR、Safety Filter、Residual RL、MPC 或第二个转向通道。

## 执行原则

1. 不调参数。所有回归使用当前代码顶部的参数。
2. 一次只修改一个结构问题；修改前先运行一个能暴露问题的诊断，修改后立即运行同一诊断和 nominal/stress/U 回归。
3. Simulink 结构只在接口确实不一致时修改；优先修改五个 MATLAB S-function。
4. 任何无法从当前状态定义和车辆方程闭合的理论，显式记录为 `THEORY GAP`，不把仿真通过当作证明。

## 分阶段设计

### 阶段 1：物理车辆模型

当前状态定义为 `[e_y,e_phi,v_y,omega_z,rho_0_state]`，其中

```text
dot(e_y)   = v_y + vx*e_phi
dot(e_phi) = omega_z - vx*rho_0
```

因此 `v_y` 和 `omega_z` 是车辆物理侧向速度/横摆角速度。道路曲率只通过误差运动学及其导数进入误差状态；不再把同一曲率项额外注入 `dot(v_y)` 和 `dot(omega_z)`。车辆动力学保留轮胎、转向和外部扰动项。

### 阶段 2：SFPPB 辅助状态

`assist1` 使用两个连续状态 `[rho_c,rho]`：

```text
dot(rho_c) = -p1*rho_c + p2*(varpi_pos+varpi_neg)
dot(rho)   = (rho_c-rho)/rho_filter_tau
```

输出为 `[rho,dot(rho)]`，其中 `dot(rho)` 与第二个状态方程完全一致。`AGV_transfor` 只用这同一对量计算柔性边界和边界导数。

### 阶段 3：第二层饱和补偿

保留

```text
z2 = chi2 - alpha1_f - O
```

并从

```text
dot(z2) = dot(chi2) - dot(alpha1_f) - dot(O)
```

直接展开。补偿系统使用实际送入 Plant 的 `delta_sat`：

```text
dot(O) = -O + C*(delta_sat-delta)
```

于是 `-dot(O)` 在 `F2_PI` 中产生明确的 `+O` 项；`F2_hat` 只逼近车辆第二层的未知剩余函数，不承担 `O`、PI 或滤波项。

### 阶段 4：第一层 Identifier

由 NMT 定义严格得到

```text
dot(zeta1) = varsigma*chi2 - Gamma
```

当前 AGV 状态中 `chi2`、`varsigma` 和 `Gamma` 均可用，因此采用 AGV-specific `F1=0`，删除 `WF1` 的状态、更新律和输入。第一层 Critic/Actor 仍可保留为虚拟控制的性能优化项，但不再声称 `WF1` 逼近未知函数。

### 阶段 5：PI 与 RL 的 Markov 状态

保留

```text
xi1 = zeta1 + K1*I1,  dot(I1)=zeta1
xi2 = zeta2 + K2*I2,  dot(I2)=zeta2
```

Critic/Actor 输入必须包含足够状态使 PI 动力学闭合。采用

```text
Z_J1 = [Z_F;zeta1;I1]
Z_J2 = [Z_F;zeta1;I1;zeta2;I2;O;alpha1_f]
```

第二层保留第一层状态，因为 `dot(alpha1_f)` 由第一层虚拟控制产生。边界调度量作为已知时变系数进入当前动力学；完整时间扩展 HJB 仍需在稳定性阶段单独证明。RBF 基函数维数和中心随此接口显式更新，不再把 `s1/s2` 单独当作完整 Markov 状态。

### 阶段 6：第二层未知函数与输入增益假设

采用“已知时变输入增益”版本：Controller 使用与 Plant 相同的 `C(t)=[cf/m;lf*cf/Iz]`，代码和 README 明确不声称 `cf(t)` 未知。`F2` 只表示在已知 `C(t)delta`、`-dot(alpha1_f)`、`+O` 和 PI 项剥离后的车辆动力学剩余项，包括外部扰动、未建模耦合和曲率变化项。

### 阶段 7：稳定性边界

代码与 README 只声明已验证的状态方程和边界保持结果。对于包含 `xi1`、`xi2`、NN 估计误差、`O` 和 `alpha1_f` 的完整 composite Lyapunov 证明，在逐项完成推导前标记 `THEORY GAP`，不直接继承 scalar strict-feedback 的 TCYB 定理。

## 结构验证门槛

每一阶段必须通过：MATLAB/Simulink 编译，nominal/stress/U 回归，无 NaN/Inf、无 `BoundaryViolation`、单一 `delta_sat` 执行入口、单套车辆状态、`rho_dot=dot(rho)`、PI 状态方程与代码一致。`learning_on=false` 只冻结学习权重，不改变控制结构。
