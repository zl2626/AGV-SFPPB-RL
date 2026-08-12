function [sys,x0,str,ts] = assist1(t,x,u,flag)
% ASSIST1  输入饱和补偿状态 rho。
% 内部状态为 [rho_c; rho]：rho_c 是饱和驱动的放松指令，rho 是平滑状态。
% 输出 [rho;rho_dot]，其中 rho_dot 始终由同一个 rho 状态方程计算。

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(x);
    case {2,4,9}
        sys = [];
    otherwise
        error('assist1:UnhandledFlag','Unhandled flag = %d.',flag);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
global u_d k_rho k_delta rho_filter_tau

% u_d 由 AGV_ctrl.m 统一设置；直接运行 assist1 时才使用默认值。
if isempty(u_d)
    u_d = 0.5;
end
if isempty(k_rho)
    k_rho = 2;                      % rho 衰减系数
end
if isempty(k_delta)
    k_delta = 5;                    % 饱和超限增益
end
if isempty(rho_filter_tau)
    rho_filter_tau = 0.02;          % rho_dot 滤波时间常数(s)
end

sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0;0];
str = [];
ts = [0 0];
end

function sys = mdlDerivatives(~,x,u)
global u_d k_rho k_delta rho_filter_tau

% 两层连续状态：先得到饱和驱动的指令 rho_c，再滤波得到 rho。
rho_c = max(x(1),0);
rho = max(x(2),0);
delta = u(1);

% 论文中的两个饱和超限项。
varpi1 = (sign(delta-u_d)+1)*(delta-u_d);
varpi2 = (sign(delta+u_d)-1)*(delta+u_d);

rho_c_dot = -k_rho*rho_c+k_delta*(varpi1+varpi2);
if x(1) <= 0 && rho_c_dot < 0
    rho_c_dot = 0;
end

% 这里的 rho_dot 就是 rho 的真实导数，供 SFPPB 边界导数使用。
rho_dot = (rho_c-rho)/rho_filter_tau;

sys = [rho_c_dot;rho_dot];
end

function sys = mdlOutputs(x)

global rho_filter_tau
rho_c = max(x(1),0);
rho = max(x(2),0);
rho_dot = (rho_c-rho)/rho_filter_tau;

% rho_c 和 rho 从零开始且 rho_c 非负，因此 rho 不会穿过零点。
if x(2) <= 0 && rho_dot < 0
    rho_dot = 0;
end

% 输出只依赖连续状态，因此本模块没有直接馈通。
sys = [rho;rho_dot];
end
