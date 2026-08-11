close all;

% 从工作区读取原模型的记录信号。
t = out.t(:);
simulation_time = t(end);
e_y = out.e_y;
eyu = out.eyu;
eyl = out.eyl;
e_phi = out.e_phi;
ephiu = out.ephiu;
ephil = out.ephil;
eyu0 = out.eyu_(:);
eyl0 = out.eyl_(:);
ephiu0 = out.ephiu_(:);
ephil0 = out.ephil_(:);
v_y = out.v_y;
omega_z = out.omega_z;
s1y = out.s1y;
z1y = out.z1y;
s1phi = out.s1phi;
z1phi = out.z1phi;
s2y = out.s2y;
z2y = out.z2y;
s2phi = out.s2phi;
z2phi = out.z2phi;
delta = out.delta;
delta1 = out.delta1;
W = out.W;
rho_transform = out.rho_transform;
rho_0_signal = out.rho_0;
if isa(rho_0_signal,'timeseries')
    rho_0_data = rho_0_signal.Data(:);
    t_rho = rho_0_signal.Time(:);
elseif isstruct(rho_0_signal) && isfield(rho_0_signal,'signals')
    rho_0_data = rho_0_signal.signals.values(:);
    t_rho = rho_0_signal.time(:);
else
    rho_0_data = rho_0_signal(:);
    t_rho = [];
end
rho = out.rho(:);
rho_dot = out.rho_dot(:);

% 优先使用Simulink实际记录的rho_0时间；只有旧Array结果才重构时间轴。
if isempty(t_rho) && numel(rho_0_data) == numel(t)
    t_rho = t;
elseif isempty(t_rho)
    warning('rho_0没有对应时间向量，暂用线性时间轴重构。');
    t_rho = linspace(0,simulation_time,numel(rho_0_data)).';
end

% 方向盘饱和上限，与 AGV_ctrl.m 保持一致。
% 每次直接读取当前工况的 global u_d，避免连续画图时沿用上一个工况的阈值。
global u_d
if isempty(u_d)
    u_d = 0.5;
end

% 在固定时间网格上计算方向盘平滑性，避免variable-step采样影响结果。
t_fixed = linspace(0,simulation_time,numel(t)).';
delta_fixed = interp1(t,delta,t_fixed,'linear','extrap');
d_delta = gradient(delta_fixed,t_fixed);
J_delta = sqrt(mean(d_delta.^2));
TV_delta = sum(abs(diff(delta_fixed)));
RMS_e_y = sqrt(trapz(t,e_y.^2)/max(simulation_time,eps));
RMS_e_phi = sqrt(trapz(t,e_phi.^2)/max(simulation_time,eps));
T_sat = trapz(t,double(abs(delta)>u_d+1e-8));
gap_y_low = min(e_y-eyl);
gap_y_high = min(eyu-e_y);
gap_phi_low = min(e_phi-ephil);
gap_phi_high = min(ephiu-e_phi);
fprintf(['RMS(e_y)=%.6g, RMS(e_phi)=%.6g, RMS_delta_dot=%.6g, ' ...
    'TV_delta=%.6g, T_sat=%.6g s, ' ...
    'min_gap_y=[%.6g, %.6g], min_gap_phi=[%.6g, %.6g]\n'], ...
    RMS_e_y,RMS_e_phi,J_delta,TV_delta,T_sat, ...
    gap_y_low,gap_y_high,gap_phi_low,gap_phi_high);

% Figure 1：横向误差与性能边界。
figure(1);
plot(t, e_y, 'b', t, eyu, 'r', t, eyl, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$e_y$','FontSize', 16, 'Interpreter', 'latex');
legend('$e_y$', '$\bar B_y$', '$\underline B_y$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim([e_y;eyu;eyl]);

% Figure 2：航向误差与性能边界。
figure(2);
plot(t, e_phi, 'b', t, ephiu, 'r', t, ephil, 'r', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$e_\phi$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$e_\phi$', '$\bar B_\phi$', '$\underline B_\phi$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim([e_phi;ephiu;ephil]);

% Figure 3：横向速度。
figure(3);
plot(t, v_y, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$v_y$', 'FontSize', 16, 'Interpreter', 'latex');
legend('$v_y$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim(v_y);

% Figure 4：横摆角速度。
figure(4);
plot(t, omega_z, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$\omega_z$(rad/s)','FontSize', 16, 'Interpreter', 'latex');
legend('$\omega_z$', 'FontSize', 24, 'FontAngle', 'italic', ...
    'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim(omega_z);

% Figure 5：第一层横向 PI 变换误差。
figure(5);
plot(t, z1y, 'r--', t, s1y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{1y}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{1y}$', '$s_{1y}=z_{1y}+k_{1y}I_{1y}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim([z1y;s1y]);

% Figure 6：第二层横向 PI 变换误差。
figure(6);
plot(t, z2y, 'r--', t, s2y, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{2y}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{2y}$', '$s_{2y}=z_{2y}+k_{2y}I_{2y}$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim([z2y;s2y]);

% Figure 7：第一层航向 PI 变换误差。
figure(7);
plot(t, z1phi, 'r--', t, s1phi, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{1\phi}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{1\phi}$', '$s_{1\phi}=z_{1\phi}+k_{1\phi}I_{1\phi}$', ...
    'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex', ...
    'IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim([z1phi;s1phi]);

% Figure 8：第二层航向 PI 变换误差。
figure(8);
plot(t, z2phi, 'r--', t, s2phi, 'b-', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$s_{2\phi}$','FontSize', 16, 'Interpreter', 'latex');
legend('$z_{2\phi}$', '$s_{2\phi}=z_{2\phi}+k_{2\phi}I_{2\phi}$', ...
    'FontSize', 24, 'FontAngle', 'italic', 'Interpreter', 'latex', ...
    'IconColumnWidth',50);
grid on;
xlim([0, simulation_time]);
setYLim([z2phi;s2phi]);

% Figure 9：方向盘请求与饱和后的实际输入。
figure(9);
subplot(2,1,1);
plot(t, delta, 'b-', t, delta1, 'r--', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$\delta$ (rad)','FontSize', 14, 'Interpreter', 'latex');
legend('$\delta$', '$sat(\delta)$', 'FontSize', 24, ...
    'FontAngle', 'italic', 'Interpreter', 'latex','IconColumnWidth',50);
yline(u_d, 'k:', 'u_d', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(-u_d, 'k:', '-u_d', 'LineWidth', 1.2, 'HandleVisibility', 'off');
grid on; xlim([0,simulation_time]); setYLim([delta;delta1;u_d;-u_d]);
subplot(2,1,2);
plot(t_fixed,d_delta,'k','linewidth',1.5);
xlabel('Time (sec)','FontSize', 14);
ylabel('$\dot\delta$ (rad/s)','FontSize', 14, 'Interpreter', 'latex');
grid on; xlim([0,simulation_time]); setYLim(d_delta);

% Figure 10：道路参考曲率 rho_0，不是柔性辅助状态 rho。
figure(10);
stairs(t_rho, rho_0_data, 'linewidth', 2);
xlabel('Time (sec)','FontSize', 16);
ylabel('$\rho_0$ (m$^{-1}$)','FontSize', 16, 'Interpreter', 'latex');
legend('$\rho_0$', 'FontSize', 24, 'Interpreter', 'latex', ...
    'IconColumnWidth',50);
grid on;
setYLim(rho_0_data);

% Figure 11：扰动和权重范数分开显示，避免混用物理量纲。
figure(11);
subplot(2,1,1);
plot(t,rho_transform,'m','linewidth',2);
xlabel('Time (sec)','FontSize', 14);
ylabel('$\rho$ (SFPPB)','FontSize', 14, 'Interpreter', 'latex');
grid on; xlim([0,simulation_time]); setYLim(rho_transform);
subplot(2,1,2);
plot(t,W,'b','linewidth',2);
xlabel('Time (sec)','FontSize', 14);
ylabel('$\|W\|$','FontSize', 14, 'Interpreter', 'latex');
grid on; xlim([0,simulation_time]); setYLim(W);

% Figure 12：rho和性能边界扩张比例分开显示。
figure(12);
subplot(2,1,1);
plot(t, rho, 'm', 'linewidth', 2);
xlabel('Time (sec)','FontSize', 14);
ylabel('$\rho$', 'FontSize', 14, 'Interpreter', 'latex');
grid on; xlim([0,simulation_time]); setYLim(rho);
subplot(2,1,2);
if numel(eyu0) == numel(t)
    y_width0 = eyu0-eyl0;
    phi_width0 = ephiu0-ephil0;
    y_widening = max(0,(eyu-eyl-y_width0)./max(y_width0,eps));
    phi_widening = max(0,(ephiu-ephil-phi_width0)./max(phi_width0,eps));
    plot(t,y_widening,'r',t,phi_widening,'b','linewidth',2);
    legend('y boundary','phi boundary','Location','best');
    setYLim([y_widening;phi_widening]);
else
    plot(t,zeros(size(t)),'k');
    legend('boundary data unavailable','Location','best');
end
xlabel('Time (sec)','FontSize',14);
ylabel('Relative boundary widening','FontSize',14);
grid on; xlim([0,simulation_time]);

% Figure 13：由 rho_0 恢复的曲线路径与实际 AGV 轨迹。
global vx_vehicle
if isempty(vx_vehicle)
    vx = 20;
else
    vx = vx_vehicle;
end
rho_ref = interp1(t_rho, rho_0_data, t, 'previous', 'extrap');
psi_r = cumtrapz(t, vx*rho_ref);
X_r = cumtrapz(t, vx*cos(psi_r));
Y_r = cumtrapz(t, vx*sin(psi_r));
X = X_r-e_y.*sin(psi_r);
Y = Y_r+e_y.*cos(psi_r);

figure(13);
plot(X_r, Y_r, 'r--', X, Y, 'b-', 'linewidth', 2);
xlabel('$X$ (m)','FontSize', 16, 'Interpreter', 'latex');
ylabel('$Y$ (m)','FontSize', 16, 'Interpreter', 'latex');
if exist('scenario_name','var')
    reference_name = scenario_name;
else
    reference_name = 'Reference path';
end
legend(reference_name, 'Actual AGV path', ...
    'FontSize', 18, 'Interpreter', 'none');
grid on;
axis equal;

% 统一给曲线留出少量上下边距，避免边界或权重被坐标轴裁掉。
function setYLim(data)
data = data(isfinite(data));
if isempty(data)
    return;
end
low = min(data);
high = max(data);
span = max(high-low,1e-3);
margin = 0.10*span;
ylim([low-margin,high+margin]);
end
