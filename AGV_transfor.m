function [sys,x0,str,ts] = AGV_transfor(t,~,u,flag)
% AGV_TRANSFOR  SFPPB性能边界和NMT变换
% 只做：
%   kappa(t) -> S(t) -> SFPPB -> NMT -> z1和Gamma
% 不保留PI、不在这里写控制律。

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = [];
    case 3
        sys = mdlOutputs(t,u);
    case {2,4,9}
        sys = [];
    otherwise
        error('AGV_transfor:UnhandledFlag','Unhandled flag = %d.',flag);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
% ========================== SFPPB参数 ==========================
% 横向误差 y
global kappa0_y kappaT_y T_y l_kappa_y l_s_y nu_y
global lambda1_y lambda2_y
% 航向误差 phi
global kappa0_phi kappaT_phi T_phi l_kappa_phi l_s_phi nu_phi
global lambda1_phi lambda2_phi
global nmt_margin

kappa0_y = 0.28;                 % kappa_y(0)
kappaT_y = 0.24;                 % kappa_y(T)
T_y = 5;
l_kappa_y = 1;
l_s_y = 1;
nu_y = 1;                         % 横向边界不对称系数
if isempty(lambda1_y), lambda1_y = 1.00; end
if isempty(lambda2_y), lambda2_y = 1.00; end

kappa0_phi = 0.10;               % kappa_phi(0)
kappaT_phi = 0.08;               % kappa_phi(T)
T_phi = 5;
l_kappa_phi = 1;
l_s_phi = 1;
nu_phi = 1;                       % 航向边界不对称系数
if isempty(lambda1_phi), lambda1_phi = 0.80; end
if isempty(lambda2_phi), lambda2_phi = 0.80; end

nmt_margin = 1e-10;              % 只避免浮点数把点判到端点

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 21;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [0 0];
end

function sys = mdlOutputs(t,u)
global kappa0_y kappaT_y T_y l_kappa_y l_s_y nu_y
global lambda1_y lambda2_y
global kappa0_phi kappaT_phi T_phi l_kappa_phi l_s_phi nu_phi
global lambda1_phi lambda2_phi nmt_margin

persistent e0_y0 e0_phi0

% Mux4输入：[e_y,de_y,rho,e_phi,de_phi,rho_dot]
e_y = u(1);
e_phi = u(4);
rho = max(0,u(3));
rho_dot_now = u(6);

% 初始误差直接读取车辆当前初值，避免SFPPB和plant各写一份e(0)。
if isempty(e0_y0) || t <= 1e-12
    e0_y0 = e_y;
    e0_phi0 = e_phi;
end

% ------------------- kappa(t)和S(t) -------------------
if t < T_y
    q_y = sin(pi*t/(2*T_y));
    q_dot_y = pi/(2*T_y)*cos(pi*t/(2*T_y));
    kappa_y = kappa0_y+(kappaT_y-kappa0_y)*q_y^l_kappa_y;
    S_y = (1-q_y)^l_s_y;
    kappa_dot_y = (kappaT_y-kappa0_y)*l_kappa_y* ...
        q_y^(l_kappa_y-1)*q_dot_y;
    S_dot_y = -l_s_y*(1-q_y)^(l_s_y-1)*q_dot_y;
else
    kappa_y = kappaT_y;
    S_y = 0;
    kappa_dot_y = 0;
    S_dot_y = 0;
end

if t < T_phi
    q_phi = sin(pi*t/(2*T_phi));
    q_dot_phi = pi/(2*T_phi)*cos(pi*t/(2*T_phi));
    kappa_phi = kappa0_phi+(kappaT_phi-kappa0_phi)*q_phi^l_kappa_phi;
    S_phi = (1-q_phi)^l_s_phi;
    kappa_dot_phi = (kappaT_phi-kappa0_phi)*l_kappa_phi* ...
        q_phi^(l_kappa_phi-1)*q_dot_phi;
    S_dot_phi = -l_s_phi*(1-q_phi)^(l_s_phi-1)*q_dot_phi;
else
    kappa_phi = kappaT_phi;
    S_phi = 0;
    kappa_dot_phi = 0;
    S_dot_phi = 0;
end

% -------------------- 名义边界和柔性边界 --------------------
% 根据实际初始误差方向选择初始性能边界。
if e0_y0 < 0
    B_under_y0 = -kappa_y+e0_y0*S_y;
    B_bar_y0 = nu_y*kappa_y+e0_y0*S_y;
    B_under_y0_dot = -kappa_dot_y+e0_y0*S_dot_y;
    B_bar_y0_dot = nu_y*kappa_dot_y+e0_y0*S_dot_y;
else
    B_under_y0 = -nu_y*kappa_y+e0_y0*S_y;
    B_bar_y0 = kappa_y+e0_y0*S_y;
    B_under_y0_dot = -nu_y*kappa_dot_y+e0_y0*S_dot_y;
    B_bar_y0_dot = kappa_dot_y+e0_y0*S_dot_y;
end

if e0_phi0 < 0
    B_under_phi0 = -kappa_phi+e0_phi0*S_phi;
    B_bar_phi0 = nu_phi*kappa_phi+e0_phi0*S_phi;
    B_under_phi0_dot = -kappa_dot_phi+e0_phi0*S_dot_phi;
    B_bar_phi0_dot = nu_phi*kappa_dot_phi+e0_phi0*S_dot_phi;
else
    B_under_phi0 = -nu_phi*kappa_phi+e0_phi0*S_phi;
    B_bar_phi0 = kappa_phi+e0_phi0*S_phi;
    B_under_phi0_dot = -nu_phi*kappa_dot_phi+e0_phi0*S_dot_phi;
    B_bar_phi0_dot = kappa_dot_phi+e0_phi0*S_dot_phi;
end

B_under_y = B_under_y0-lambda1_y*tanh(rho);
B_bar_y = B_bar_y0+lambda2_y*tanh(rho);
B_under_phi = B_under_phi0-lambda1_phi*tanh(rho);
B_bar_phi = B_bar_phi0+lambda2_phi*tanh(rho);

% 柔性边界导数：tanh'(rho)=1-tanh(rho)^2。
sech2_rho = 1-tanh(rho)^2;
B_under_y_dot = B_under_y0_dot-lambda1_y*sech2_rho*rho_dot_now;
B_bar_y_dot = B_bar_y0_dot+lambda2_y*sech2_rho*rho_dot_now;
B_under_phi_dot = B_under_phi0_dot-lambda1_phi*sech2_rho*rho_dot_now;
B_bar_phi_dot = B_bar_phi0_dot+lambda2_phi*sech2_rho*rho_dot_now;

% NMT中的已知边界变化项：Gamma=B_bar_dot/(B_bar-e)
%                         +B_under_dot/(e-B_under)。
Gamma_y = B_bar_y_dot/(B_bar_y-e_y)+B_under_y_dot/(e_y-B_under_y);
Gamma_phi = B_bar_phi_dot/(B_bar_phi-e_phi)+ ...
    B_under_phi_dot/(e_phi-B_under_phi);

% -------------------------- NMT --------------------------
% 调试阶段不把越界误差夹回边界；越界就直接报错。
if e_y <= B_under_y+nmt_margin || e_y >= B_bar_y-nmt_margin
    error('AGV_transfor:BoundaryViolation', ...
        'y边界在t=%.6f被越过：e_y=%.6g, [%.6g, %.6g].', ...
        t,e_y,B_under_y,B_bar_y);
end
if e_phi <= B_under_phi+nmt_margin || e_phi >= B_bar_phi-nmt_margin
    error('AGV_transfor:BoundaryViolation', ...
        'phi边界在t=%.6f被越过：e_phi=%.6g, [%.6g, %.6g].', ...
        t,e_phi,B_under_phi,B_bar_phi);
end

z1_y = log((e_y-B_under_y)/(B_bar_y-e_y));
z1_phi = log((e_phi-B_under_phi)/(B_bar_phi-e_phi));
varsigma_y = (B_bar_y-B_under_y)/ ...
    ((B_bar_y-e_y)*(e_y-B_under_y));
varsigma_phi = (B_bar_phi-B_under_phi)/ ...
    ((B_bar_phi-e_phi)*(e_phi-B_under_phi));

% 保持原Demux5/Mux1接口不变；第11、12路原来是空位，现在传Gamma。
zero = 0;
sys = [B_bar_y;B_under_y;B_bar_phi;B_under_phi; ...
       varsigma_y;z1_y;z1_phi;varsigma_phi; ...
       zero;zero;Gamma_y;Gamma_phi;rho; ...
       z1_y;z1_phi;zero;zero; ...
       B_bar_y0;B_under_y0;B_bar_phi0;B_under_phi0];
end
