# AGV-SFPPB-PI-RL：实现对应的理论边界

本文档只记录当前 MATLAB/Simulink 实现已经写出的方程，以及尚未完成的理论证明。它不是把标量 strict-feedback 论文的定理直接套到 AGV 上的证明。

## 1. 车辆模型与唯一转向输入

车辆连续状态为

\[
x_p=[e_y,e_\phi,v_y,\omega_z,\rho_0^f]^T,
\]

其中 \(\rho_0^f\) 是输入曲率的平滑状态。模型实际使用

\[
\begin{aligned}
\dot e_y&=v_y+v_xe_\phi,\\
\dot e_\phi&=\omega_z-v_x\rho_0^f,\\
\dot v_y&=A_{11}v_y+A_{12}\omega_z+B_1\delta_{\rm sat}+d_y,\\
\dot\omega_z&=A_{21}v_y+A_{22}\omega_z+B_2\delta_{\rm sat}+d_\phi.
\end{aligned}
\]

道路曲率先经过 \(\rho_0^f\) 的一阶滤波，再进入误差运动学；同一个 \(\rho_0^f\) 也用于 Plant 输出给 SFPPB/Controller 的 \(\dot e_\phi\)。它不作为额外项重复加入 \(\dot v_y\) 或 \(\dot\omega_z\)。控制器输出未饱和请求 \(\delta\)，唯一执行输入为

\[
\delta_{\rm sat}=\operatorname{sat}(\delta,-u_d,u_d).
\]

## 2. SFPPB 辅助状态与 NMT

`assist1.m` 的连续状态是 \([\rho_c,\rho]^T\)。饱和超限量由未饱和请求计算：

\[
\begin{aligned}
\varpi_+&=(\operatorname{sign}(\delta-u_d)+1)(\delta-u_d),\\
\varpi_-&=(\operatorname{sign}(\delta+u_d)-1)(\delta+u_d),\\
\dot\rho_c&=-p_1\rho_c+p_2(\varpi_++\varpi_-),\\
\dot\rho&=(\rho_c-\rho)/\tau_\rho.
\end{aligned}
\]

送入 `AGV_transfor.m` 的第二个量就是上式的同一个 \(\dot\rho\)，所以边界和边界导数使用同一对 \((\rho,\dot\rho)\)。对每个通道 \(i\in\{y,\phi\}\)，代码实现

\[
\underline B_i=B_{i0}^- -\lambda_i^-\tanh\rho,
\qquad
\overline B_i=B_{i0}^+ +\lambda_i^+\tanh\rho,
\]

边界初始宽度的不对称系数使用 \(\nu_y,\nu_\phi\)，不再使用与柔性状态容易混淆的 \(\eta\)。

并使用

\[
\dot{\underline B}_i=\dot B_{i0}^- -\lambda_i^-\operatorname{sech}^2(\rho)\dot\rho,
\quad
\dot{\overline B}_i=\dot B_{i0}^+ +\lambda_i^+\operatorname{sech}^2(\rho)\dot\rho.
\]

归一化误差与 NMT 为

\[
\mu_i=\frac{e_i-\underline B_i}{\overline B_i-\underline B_i},
\qquad
z_{1i}=\log\frac{\mu_i}{1-\mu_i},
\]

\[
\varsigma_i=\frac{\overline B_i-\underline B_i}
{(\overline B_i-e_i)(e_i-\underline B_i)},
\qquad
\Gamma_i=\frac{\dot{\overline B}_i}{\overline B_i-e_i}
 +\frac{\dot{\underline B}_i}{e_i-\underline B_i}.
\]

在边界内部有

\[
\dot z_1=\varsigma\,\chi_2-\Gamma,
\qquad \chi_2=[\dot e_y,\dot e_\phi]^T.
\]

## 3. PI 变换和两层控制器

代码中的 `s1`、`s2` 对应理论中的 PI 变换误差：

\[
\begin{aligned}
I_1&=\int_0^t z_1(\tau)d\tau,& \xi_1=s_1&=z_1+K_1I_1,\\
I_2&=\int_0^t z_2(\tau)d\tau,& \xi_2=s_2&=z_2+K_2I_2.
\end{aligned}
\]

因此

\[
\dot\xi_1=\varsigma\chi_2-\Gamma+K_1z_1.
\]

第一层不再使用没有明确对象的 Identifier：

\[
F_1\equiv0,
\qquad
\alpha_1=\varsigma^{-1}
\left(-C_1\xi_1+\Gamma-K_1z_1-
\tfrac12\hat W_{a1}^{T}\Phi_{J1}\right).
\]

为避免直接使用未计算的 \(\dot\alpha_1\)，代码引入

\[
\dot\alpha_{1f}=\frac{\alpha_1-\alpha_{1f}}{\tau_{\alpha1}}.
\]

第二层定义为

\[
z_2=\chi_2-\alpha_{1f}-O,
\qquad
\dot I_2=z_2.
\]

## 4. 第二层 \(F_2\)、饱和补偿与已知输入增益

本工程选用“已知时变输入增益”假设。令

\[
C(t)=\begin{bmatrix}c_f(t)/m\\l_f c_f(t)/I_z\end{bmatrix},
\]

Controller 和 Plant 使用同一个 \(c_f(t)\) 和 \(C(t)\)。第二层漂移函数被唯一规定为

\[
\boxed{F_2(X,t):=\dot\chi_2-C(t)\delta_{\rm sat}}.
\]

因此 \(F_2\) 包含去掉实际转向输入后的轮胎漂移、后轮参数影响、外部扰动、曲率滤波状态引起的耦合和未建模项；已知的 \(C(t)\delta_{\rm sat}\) 不属于 Identifier 的目标。`WF2` 只逼近 \(F_2\)：

\[
\hat F_2=\hat W_{F2}^{T}\Phi_F(Z_F).
\]

饱和辅助状态的实现为

\[
\dot O=-O+C(t)(\delta_{\rm sat}-\delta).
\]

由定义直接得到

\[
\begin{aligned}
\dot z_2
 &=\dot\chi_2-\dot\alpha_{1f}-\dot O\\
 &=F_2+C\delta-\dot\alpha_{1f}+O.
\end{aligned}
\]

所以代码中的第二层 PI 漂移项是

\[
F_{2,\mathrm{PI}}=\hat F_2-\dot\alpha_{1f}+O+K_2z_2,
\]

并使用

\[
p_{a2}=2C_2\xi_2+2F_{2,\mathrm{PI}}
 +\hat W_{a2}^{T}\Phi_{J2},
\qquad
\delta_{\rm fb}=-\frac{C^Tp_{a2}}{2r_\delta}.
\]

最终请求为 \(\delta=\delta_{\rm fb}+\delta_{\rm ff}\)，其中当前代码的 \(\delta_{\rm ff}=\texttt{rho\_ff\_gain}\,\rho_0\)。

## 5. RL 状态接口的实际含义

为保留 PI 积分状态，当前网络输入为

\[
Z_{J1}=[Z_F;z_1;I_1],
\qquad
Z_{J2}=[Z_F;z_1;I_1;z_2;I_2;O;\alpha_{1f}],
\]

其中 \(Z_F=[e_y,e_\phi,\dot e_y,\dot e_\phi]^T\)。这比只输入 \(s_1\) 或 \(s_2\) 保留了积分状态的信息。

`J1/J2` 的当前输入仍是一个降维的可测回归量：它没有显式加入 \(\rho_c,\rho,\rho_0^f\) 或时间 \(t\)。因此对于带时变性能边界、饱和辅助和轮胎变化的完整 HJB，当前网络接口只能作为实现所用的近似状态。

**THEORY GAP — Markov 完备性：** 若要声称严格的时变 HJB，需要把边界调度状态（至少 \(\rho_c,\rho\)）及必要的曲率滤波状态/时间扩展加入状态，并重新写出对应的价值函数和梯度。当前仿真不等于该闭合证明。

## 6. 稳定性证明边界

AGV 是两个误差通道共享一个真实转向输入的向量系统，且包含 PI 积分器、虚拟控制滤波状态、饱和辅助状态和五组在线权重。后续严格证明至少需要从类似下式的复合候选函数开始：

\[
V=\tfrac12\|\xi_1\|^2+\tfrac12\|\xi_2\|^2
 +\tfrac12\|O\|^2+\tfrac12\|\alpha_{1f}-\alpha_1\|^2
 +V_{\widetilde W_{F2}}+V_{\widetilde W_{C1}}+V_{\widetilde W_{A1}}
 +V_{\widetilde W_{C2}}+V_{\widetilde W_{A2}}.
\]

必须逐项处理：

1. \(C(t)\) 的时变性和单输入投影 \(C^Tp_{a2}\)；
2. \(\xi_1,\xi_2\) 的 PI 交叉项；
3. \(\dot O\) 与饱和误差的抵消；
4. \(\alpha_{1f}-\alpha_1\) 的滤波误差；
5. Identifier、Critic、Actor 权重误差和泄漏项；
6. \(\rho\) 驱动的时变边界导数；
7. 两个误差通道共享一个 \(\delta\) 所产生的欠驱动耦合。

**THEORY GAP — Composite Lyapunov proof：** 当前仓库没有完成上述复合 Lyapunov 推导，也没有得到可审计的

\[
\dot V\le -cV+\Theta
\]

不等式。因此 README 和结果说明只声称“仿真中未发生边界越界”，不声称直接继承 SFPPB-RL 原论文的标量 strict-feedback 稳定性定理。只有在补齐这一步后，才能从 \(z_1\) 有界进一步给出所有 \(e_i\) 严格落在动态 SFPPB 内的理论结论。

## 7. Simulink 接口结论

本轮不需要改 `AGV_simulate.slx` 或 `AGV_simulate_U.slx` 的连线：

- Plant 仍接收 `[delta_sat,rho_0]`，只保留一套车辆状态；
- `assist1` 仍是两状态、输出 `[rho,rho_dot]`，但现在两者来自同一滤波方程；
- `AGV_transfor`、Controller 的输入输出端口数量没有改变；
- Controller 状态数由删除未定义的 `WF1` 后在 S-function 初始化时自动变为 `10*N+8`，模型没有写死旧状态长度；
- 三种回归仿真均无代数环、NaN/Inf 或 `BoundaryViolation`。

因此当前结构问题属于 S-function 内部方程和状态定义，不需要额外增加 Simulink 控制模块或第二条转向路径。
