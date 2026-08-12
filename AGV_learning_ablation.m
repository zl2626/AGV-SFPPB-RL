% AGV_LEARNING_ABLATION  比较在线学习和冻结权重两种工况。
% 只冻结五组NN权重，控制器结构和方向盘上限保持不变。

clear global learning_on u_d k1y k1phi k2y k2phi c1y c2y c1phi c2phi ...
    p1 p2 tau_alpha1 rho_ff_gain ...
    lambda_lower_y lambda_upper_y lambda_lower_phi lambda_upper_phi vx_vehicle ...
    disturbance_y_amplitude disturbance_phi_amplitude
global learning_on u_d
u_d = 0.5;

load_system('AGV_simulate');
set_param('AGV_simulate','SaveState','on','StateSaveName','xout');

learning_on = true;
out_on = sim('AGV_simulate','StopTime','20','ReturnWorkspaceOutputs','on');
learning_on = false;
out_off = sim('AGV_simulate','StopTime','20','ReturnWorkspaceOutputs','on');

on = getMetrics(out_on);
off = getMetrics(out_off);

fprintf('\n学习开关消融结果\n');
fprintf('                 RMS(e_y)      RMS(e_phi)     RMS(delta_dot)   TV(delta)\n');
fprintf('learning ON   %12.6g %12.6g %15.6g %12.6g\n', ...
    on.basic(1),on.basic(2),on.basic(3),on.basic(4));
fprintf('learning OFF  %12.6g %12.6g %15.6g %12.6g\n', ...
    off.basic(1),off.basic(2),off.basic(3),off.basic(4));

fprintf('\n五组权重范数（初值 -> 终值，learning ON）\n');
names = {'WC1','WA1','WF2','WC2','WA2'};
for k = 1:numel(names)
    fprintf('%s: %.6g -> %.6g\n',names{k},on.weight_norms(1,k),on.weight_norms(end,k));
end

% 恢复默认状态，避免影响后续普通仿真。
learning_on = true;
set_param('AGV_simulate','SaveState','off');
close_system('AGV_simulate',0);
clear global learning_on u_d k1y k1phi k2y k2phi c1y c2y c1phi c2phi ...
    p1 p2 tau_alpha1 rho_ff_gain ...
    lambda_lower_y lambda_upper_y lambda_lower_phi lambda_upper_phi vx_vehicle ...
    disturbance_y_amplitude disturbance_phi_amplitude

function metrics = getMetrics(out)
t = out.t(:);
delta = out.delta(:);
t_fixed = linspace(0,t(end),numel(t)).';
delta_fixed = interp1(t,delta,t_fixed,'linear','extrap');
d_delta = gradient(delta_fixed,t_fixed);
metrics.basic = [ ...
    rmsTime(t,out.e_y(:)), ...
    rmsTime(t,out.e_phi(:)), ...
    sqrt(mean(d_delta.^2)), ...
    sum(abs(diff(delta_fixed)))];

% 控制器状态中的五组NN权重，顺序与AGV_ctrl.m一致。
ctrl = out.xout{3}.Values.Data;
metrics.weight_norms = [ ...
    vecnorm(ctrl(:,1:14),2,2), ...
    vecnorm(ctrl(:,15:28),2,2), ...
    vecnorm(ctrl(:,29:42),2,2), ...
    vecnorm(ctrl(:,43:56),2,2), ...
    vecnorm(ctrl(:,57:70),2,2)];
end

function value = rmsTime(t,x)
value = sqrt(trapz(t,x.^2)/max(t(end)-t(1),eps));
end
