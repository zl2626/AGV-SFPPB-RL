# AGV 物理模型与 PI-RL 理论闭合 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** 在不调参和不增加控制器的前提下，逐项闭合 AGV-SFPPB-PI-RL 的物理、辅助状态、饱和补偿、Identifier 与 PI-Markov 状态定义。

**Architecture:** 保留两个现有 Simulink 模型和五个 Level-1 MATLAB S-function。先修 `AGV_plant.m`，再修 `assist1.m`，随后修 `AGV_ctrl.m` 的 `O/z2/F1/F2`，最后更新 `AGV_RBF.m` 的状态接口和理论说明。每个任务只修改一个结构边界，并用同一组 MATLAB 回归命令验证。

**Tech Stack:** MATLAB/Simulink Level-1 S-functions、MATLAB batch regression、Git。

## Global Constraints

- 只保留一个车辆转向输入 `delta`，Plant 的唯一执行输入为 `delta_sat`。
- 不加入 LQR、Safety Filter、Residual RL、MPC 或第二个转向通道。
- 不创建参数文件、类、复杂 config 或对象化框架。
- 不调参数；所有回归使用现有 `AGV_ctrl.m` 顶部参数。
- 任何未完成的理论证明必须标记 `THEORY GAP`，不能直接继承 scalar strict-feedback 定理。

### Task 1: 修正物理车辆曲率重复项

**Files:**
- Modify: `AGV_plant.m:80-101`
- Test: MATLAB inline S-function diagnostic plus nominal/stress/U simulation

**Interfaces:**
- Consumes: `[delta_sat,rho_0]` from the existing Simulink plant input.
- Produces: `[e_y,e_phi,de_y,de_phi,v_y,omega_z]` with the existing six-output interface.

- [ ] **Step 1: Run the failing diagnostic**

```matlab
clear global vx_vehicle disturbance_y_amplitude disturbance_phi_amplitude
global vx_vehicle disturbance_y_amplitude disturbance_phi_amplitude
vx_vehicle = 20; disturbance_y_amplitude = 0; disturbance_phi_amplitude = 0;
x = [-0.1;0.01;0.02;0.03;0];
d0 = AGV_plant(0,x,[0;0],1);
d1 = AGV_plant(0,x,[0;0.003],1);
assert(norm(d0(3:4)-d1(3:4)) < 1e-12, ...
    'Physical v_y/omega_z dynamics still depend on rho_0.');
```

Expected before the fix: FAIL because the current physical acceleration rows contain `rho_0`.

- [ ] **Step 2: Remove only the curvature terms from `D(3:4)`**

Use

```matlab
D = [0;0;disturbance_y;disturbance_phi];
```

Keep `de_phi = omega_z-vx*rho_0` and the `rho_0_state` filter unchanged in this task.

- [ ] **Step 3: Run the diagnostic again**

Expected: PASS with `norm(d0(3:4)-d1(3:4)) < 1e-12`.

- [ ] **Step 4: Run the three regression scenarios**

Run the existing `AGV_simulate` nominal 20 s, `AGV_simulate` stress (`u_d=0.3`, disturbances 18), and `AGV_simulate_U` 70 s without changing parameters. Record NaN/Inf, boundary errors, and the printed metrics.

- [ ] **Step 5: Commit the isolated change**

```bash
git add AGV_plant.m
git commit -m "remove duplicated curvature forcing from plant"
```

### Task 2: Make `rho_dot` the true derivative of `rho`

**Files:**
- Modify: `assist1.m:1-81`
- Test: direct state-derivative identity plus the three regression scenarios

**Interfaces:**
- Consumes: unsaturated controller command `delta`.
- Produces: `[rho,rho_dot]`, with `rho_dot=dot(rho)` by construction and the same two-output Simulink interface.

- [ ] **Step 1: Run the failing identity diagnostic**

Use a state with a nonzero filtered derivative and compare the first state derivative with the output derivative. Expected before the fix: the current state `[rho,rho_dot]` does not satisfy the identity for arbitrary state values.

- [ ] **Step 2: Change state meaning to `[rho_c,rho]`**

Implement

```matlab
rho_c_dot = -p1*rho_c+p2*(varpi1+varpi2);
rho_dot = (rho_c-rho)/rho_filter_tau;
sys = [rho_c_dot;rho_dot];
```

Return `[rho,rho_dot]` from the second state and keep `DirFeedthrough=0`.

- [ ] **Step 3: Run the identity diagnostic again**

Expected: PASS for zero and nonzero saturation commands, with the reported `rho_dot` equal to the derivative of the `rho` state equation.

- [ ] **Step 4: Run all three regression scenarios**

Expected: no algebraic-loop error, NaN/Inf, or `BoundaryViolation`.

- [ ] **Step 5: Commit the isolated change**

```bash
git add assist1.m
git commit -m "make flexible state derivative mathematically consistent"
```

### Task 3: Derive `O` and `F2_PI` from `dot(z2)`

**Files:**
- Modify: `AGV_ctrl.m:150-190,245-275`
- Test: symbolic/vector identity check and three regression scenarios

**Interfaces:**
- Consumes: `chi2`, `alpha1_f`, `O`, `delta`, and the exact `delta_sat` sent to Plant.
- Produces: one `delta`, one `delta_sat`, and a second-layer residual with explicit `+O`.

- [ ] **Step 1: Check the current algebra**

For `z2=chi2-alpha1_f-O`, verify by hand and in a MATLAB scalar/vector check that `-dot(O)` contributes `+O` when `dot(O)=-O+C*(delta_sat-delta)`.

- [ ] **Step 2: Use the exact applied input in `dot(O)`**

Replace the smooth-only compensation with

```matlab
delta_sat = min(max(delta,-u_d),u_d);
dO = -O+C*(delta_sat-delta);
```

and define

```matlab
F2_PI = F2_hat-dalpha1_f+O+K2.*z2;
```

`F2_hat` remains the identifier output only.

- [ ] **Step 3: Run the algebraic identity check**

Expected: the residual expression obtained from `dot(z2)` contains the known `+O` term exactly once and the applied-input mismatch cancels.

- [ ] **Step 4: Run all three regression scenarios**

Expected: one saturation execution path and no boundary/NaN failures.

- [ ] **Step 5: Commit the isolated change**

```bash
git add AGV_ctrl.m
git commit -m "derive second-layer saturation compensation from z2"
```

### Task 4: Remove the undefined first-layer Identifier

**Files:**
- Modify: `AGV_ctrl.m:state layout and first-layer calculations`
- Modify: `AGV_RBF.m:remove unused F1 basis branch only if no other consumer needs it`
- Modify: `AGV_learning_ablation.m:weight names and state slices`
- Modify: `README.md:state order and F1 definition`
- Test: input/state dimension check and three regression scenarios

**Interfaces:**
- Consumes: `z1`, `varsigma`, `Gamma`, `chi2`, `I1`.
- Produces: first-layer virtual control with `F1=0`, while retaining first-layer Critic/Actor correction.

- [ ] **Step 1: Write and run the failing interface check**

Assert that the current controller state contains `WF1` even though the first-layer kinematics are exactly computable; record the current state length `12*N+8`.

- [ ] **Step 2: Remove `WF1` states and update `x0`/unpacking**

Use `F1_hat=zeros(2,1)` and remove only `WF1` and `dWF1`; do not change the Critic/Actor update in this task.

- [ ] **Step 3: Run the new state-length and S-function output checks**

Expected: controller state length decreases by `2*N`, outputs remain nine, and the first-layer virtual control is finite.

- [ ] **Step 4: Run all three regression scenarios**

Expected: no interface mismatch, NaN/Inf, or boundary error.

- [ ] **Step 5: Commit the isolated change**

```bash
git add AGV_ctrl.m AGV_RBF.m
git commit -m "remove undefined first-layer identifier"
```

### Task 5: Close the PI-RL Markov state

**Files:**
- Modify: `AGV_ctrl.m:Z_J1/Z_J2 construction and state comments`
- Modify: `AGV_RBF.m:explicit J input dimensions and centers`
- Test: direct basis-dimension checks and three regression scenarios

**Interfaces:**
- Consumes: `[Z_F,z1,I1]` for layer 1 and `[Z_F,z2,I2,O,alpha1_f]` for layer 2.
- Produces: basis vectors whose input contains every integral/filter state used by the control dynamics.

- [ ] **Step 1: Run the failing Markov closure check**

Hold `Z_F` and `s1` fixed while changing `I1`; verify that the current J input is unchanged, exposing the missing PI state.

- [ ] **Step 2: Add the missing states explicitly**

Use separate state vectors for the two networks and update `AGV_RBF` dimensions/centers rather than silently reusing the old six-dimensional J basis.

- [ ] **Step 3: Run basis-size and finite-output checks**

Expected: each basis function accepts the declared dimension and returns finite values for nominal simulation states.

- [ ] **Step 4: Run all three regression scenarios**

Expected: no RBF input errors, NaN/Inf, or boundary failures.

- [ ] **Step 5: Commit the isolated change**

```bash
git add AGV_ctrl.m AGV_RBF.m
git commit -m "close PI integral states in RL Markov inputs"
```

### Task 6: Make the `F2` and tire-gain assumption explicit

**Files:**
- Modify: `AGV_ctrl.m:comments and known-gain definitions`
- Modify: `README.md`
- Test: grep/interface audit and three regression scenarios

**Interfaces:**
- Consumes: known time-varying `C(t)` and the plant’s physical parameters.
- Produces: documentation that `F2_hat` is only the residual unknown function and `C(t)` is known to the controller.

- [ ] **Step 1: Run the assumption audit**

Compare controller and plant definitions of `cf(t)`, `cr(t)`, and `C(t)`; record the current equality.

- [ ] **Step 2: Document the known-gain model**

State that the controller uses the same known `cf(t)` input gain and does not claim unknown tire stiffness. Define the residual `F2` in comments and README.

- [ ] **Step 3: Run the audit and all regressions**

Expected: no conflicting “unknown tire” claims remain and simulations still pass.

- [ ] **Step 4: Commit the isolated documentation change**

```bash
git add AGV_ctrl.m README.md
git commit -m "document known time-varying input gain and F2 residual"
```

### Task 7: Record the stability-proof boundary

**Files:**
- Modify: `README.md`
- Create: `docs/theory/AGV_SFPPB_PI_RL_stability_notes.md`
- Test: notation/claim grep and regression summary

**Interfaces:**
- Consumes: the finalized equations from Tasks 1–6.
- Produces: an explicit composite-Lyapunov checklist and `THEORY GAP` markers for any unproved statements.

- [ ] **Step 1: Write the equations actually implemented**

List `xi1`, `xi2`, `O`, `alpha1_f`, `F1=0`, and the residual `F2` using the final code symbols.

- [ ] **Step 2: Mark unproved claims**

Do not state that the scalar TCYB theorem directly proves the two-channel AGV result. Mark the missing composite Lyapunov cross-term proof as `THEORY GAP` until it is derived.

- [ ] **Step 3: Run final structure verification**

Check no NaN/Inf, no boundary errors, one steering path, one plant state set, consistent `rho/rho_dot`, complete PI-RL state inputs, and `learning_on=false` behavior.

- [ ] **Step 4: Commit the theory notes and README update**

```bash
git add README.md docs/theory/AGV_SFPPB_PI_RL_stability_notes.md
git commit -m "document AGV-specific stability proof boundary"
```
