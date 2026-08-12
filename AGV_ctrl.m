function [sys,x0,str,ts] = AGV_ctrl(t,x,u,flag)
% AGV_CTRL  SFPPB-PI-RL 控制器
% 按照原 AGV-TFS 的写法组织：初始化、状态导数、输出。
% 参数都在下面的初始化函数里，调参时直接修改数值即可。

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {2,4,9}
        sys = [];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
% ========================== 参数区 ===========================
% RBF 节点数
global N
N = 7;

% PI 参数：s1=z1+K1*I1，s2=z2+K2*I2
global k1y k1phi k2y k2phi
global tau_alpha1 vx_vehicle rho_ff_gain
if isempty(k1y), k1y = 0.10; end   % 第一层横向误差积分系数
if isempty(k1phi), k1phi = 0.20; end % 第一层航向误差积分系数
if isempty(k2y), k2y = 0.01; end   % 第二层横向误差积分系数
if isempty(k2phi), k2phi = 0.01; end % 第二层航向误差积分系数
if isempty(tau_alpha1)
    tau_alpha1 = 0.01;              % 虚拟控制一阶滤波时间常数(s)
end
if isempty(vx_vehicle)
    vx_vehicle = 20;                % 纵向速度(m/s)
end
if isempty(rho_ff_gain)
    rho_ff_gain = 4.6;              % 道路曲率前馈系数（当前综合最优）
end

% 两层控制器参数
global c1y c1phi c2y c2phi
if isempty(c1y)
    c1y = 5;                        % 第一层横向稳定系数
end
if isempty(c1phi)
    c1phi = 35;                      % 第一层航向稳定系数
end
if isempty(c2y)
    c2y = 5;                        % 第二层横向稳定系数
end
if isempty(c2phi)
    c2phi = 8;                       % 第二层航向稳定系数
end

% RBF 自适应参数
global Upsilon2 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2
global learning_on
Upsilon2 = 0.04;                    % 第二层辨识增益
sigma2 = 0.08;                      % 第二层泄漏系数
gamma_c1 = 0.004;                   % 第一层 Critic 增益
gamma_c2 = 0.004;                   % 第二层 Critic 增益
gamma_a1 = 0.012;                   % 第一层 Actor 增益
gamma_a2 = 0.012;                   % 第二层 Actor 增益
if isempty(learning_on)
    learning_on = true;             % true在线学习，false冻结全部NN权重
end

% 车辆和方向盘参数
global u_d m Iz lf lr cf0 cf_rate r_delta
if isempty(u_d)
    u_d = 0.5;                       % 方向盘最大输入（唯一来源）
end
m = 1832;                           % 车辆质量
Iz = 2488;                          % 横摆转动惯量
lf = 1.18;                          % 前轴到质心距离
lr = 1.77;                          % 后轴到质心距离
cf0 = 80000;                        % 初始前轮侧偏刚度
cf_rate = 0.10;                     % 侧偏刚度变化幅度
r_delta = norm([cf0/m;lf*cf0/Iz]);   % HJB转向输入代价权重

% S-function 接口
sizes = simsizes;
sizes.NumContStates  = 10*N+8;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 13;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);

% 状态顺序：[WC1;WA1;WF2;WC2;WA2;O;alpha1_f;I1;I2]
W0 = 0.4;
w0 = W0*[-1;-1;-1;0;1;1;1];
WC10 = repmat(w0,1,2);
WA10 = repmat(w0,1,2);
WF20 = repmat(w0,1,2);
WC20 = repmat(w0,1,2);
WA20 = repmat(w0,1,2);
O0 = zeros(2,1);
alpha10 = zeros(2,1);
I10 = zeros(2,1);
I20 = zeros(2,1);
x0 = [WC10(:);WA10(:);WF20(:);WC20(:);WA20(:);O0;alpha10;I10;I20];

str = [];
ts = [0 0];
end

function sys = mdlDerivatives(t,x,u)
% 这里按顺序完成：解包状态、计算控制量、更新权重和 PI 积分器。
global N c1y c1phi c2y c2phi
global k1y k1phi k2y k2phi
global tau_alpha1
global Upsilon2 sigma2
global gamma_c1 gamma_c2 gamma_a1 gamma_a2 learning_on
global u_d m Iz lf lr cf0 cf_rate r_delta rho_ff_gain

% ------------------------- 解包状态 --------------------------
i = 0;
WC1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WF2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WC2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
O = x(i+1:i+2); i = i+2;
alpha1_f = x(i+1:i+2); i = i+2;
I1 = x(i+1:i+2); i = i+2;
I2 = x(i+1:i+2);

% ------------------------- 解包输入 --------------------------
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Gamma = [u(11);u(12)];
Z_F = [u(7);u(8);u(9);u(10)];
rho_0 = u(13);

C1 = [c1y;c1phi];
C2 = [c2y;c2phi];
K1 = [k1y;k1phi];
K2 = [k2y;k2phi];

% ------------------------- 第一层 ----------------------------
s1 = z1+K1.*I1;
Z_J1 = [Z_F;z1;I1];
Phi_J1 = AGV_RBF(Z_J1,'J1');
% NMT给出 dot(z1)=varsigma*chi2-Gamma，第一层没有需要辨识的未知函数。
F1_hat = zeros(2,1);
alpha1 = varsigma\(-C1.*s1+Gamma-K1.*z1-F1_hat-0.5*WA1'*Phi_J1);

% 用连续滤波器承接虚拟控制，显式得到dot(alpha1_f)。
dalpha1_f = (alpha1-alpha1_f)/tau_alpha1;

% ------------------------- 第二层 ----------------------------
z2 = chi2-alpha1_f-O;
s2 = z2+K2.*I2;
Phi_F2 = AGV_RBF(Z_F,'F');
Z_J2 = [Z_F;z1;I1;z2;I2;O;alpha1_f];
Phi_J2 = AGV_RBF(Z_J2,'J2');
F2_hat = WF2'*Phi_F2;

% 输入增益假设：dot(chi2)=F2(X,t)+g_delta(t)*delta_sat。
% F2 是去掉实际转向输入后的车辆第二层漂移，包含轮胎漂移、外部扰动、
% rho_0及其滤波状态引起的项；WF2 只逼近这个 F2，不承担 O、PI 或滤波项。
% 第二层动力学中的-alpha1_dot现在由滤波器显式给出。
% 由 z2=chi2-alpha1_f-O 可知，-dot(O)在漂移项中留下明确的 +O。
% F2_hat只辨识车辆第二层的未知剩余项，PI积分项为K2*z2。
F2_PI = F2_hat-dalpha1_f+O+K2.*z2;

% 已知时变输入增益：Controller 与 Plant 使用同一个 cf(t) 和 g_delta(t)。
cf = cf0*(1+cf_rate*sin(0.01*t));
g_delta = [cf/m;lf*cf/Iz];

% 方向盘控制量和输入饱和补偿状态
p_a2 = 2*C2.*s2+2*F2_PI+WA2'*Phi_J2;
delta_feedback = -(g_delta'*p_a2)/(2*r_delta);
delta_feedforward = rho_ff_gain*rho_0; % 车辆模型的曲率前馈系数
delta = delta_feedback+delta_feedforward;
delta_sat = min(max(delta,-u_d),u_d);
% O 使用与 Plant 完全相同的实际执行输入，保证饱和项在 dot(z2) 中抵消。
dO = -O+g_delta*(delta_sat-delta);

% ------------------------- 权重更新 --------------------------
if learning_on
    dWF2 = Upsilon2*(Phi_F2*s2'-sigma2*WF2);
    dWC1 = -gamma_c1*(Phi_J1*Phi_J1')*WC1;
    dWC2 = -gamma_c2*(Phi_J2*Phi_J2')*WC2;
    dWA1 = -(Phi_J1*Phi_J1')*(gamma_a1*(WA1-WC1)+gamma_c1*WC1);
    dWA2 = -(Phi_J2*Phi_J2')*(gamma_a2*(WA2-WC2)+gamma_c2*WC2);
else
    dWF2 = zeros(size(WF2));
    dWC1 = zeros(size(WC1));
    dWC2 = zeros(size(WC2));
    dWA1 = zeros(size(WA1));
    dWA2 = zeros(size(WA2));
end

% PI 积分器
dI1 = z1;
dI2 = z2;

sys = [dWC1(:);dWA1(:);dWF2(:);dWC2(:);dWA2(:);dO; ...
       dalpha1_f;dI1;dI2];
end

function sys = mdlOutputs(t,x,u)
% 输出实际方向盘请求、饱和后的请求、权重范数以及调试信号。
global N c1y c1phi c2y c2phi
global k1y k1phi k2y k2phi
global tau_alpha1
global u_d m Iz lf lr cf0 cf_rate r_delta rho_ff_gain

% ------------------------- 解包状态 --------------------------
i = 0;
WC1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA1 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WF2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WC2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
WA2 = reshape(x(i+1:i+2*N),N,2); i = i+2*N;
O = x(i+1:i+2); i = i+2;
alpha1_f = x(i+1:i+2); i = i+2;
I1 = x(i+1:i+2); i = i+2;
I2 = x(i+1:i+2);

% ------------------------- 解包输入 --------------------------
varsigma = diag([max(u(1),eps),max(u(4),eps)]);
z1 = [u(2);u(3)];
chi2 = [u(9);u(10)];
Gamma = [u(11);u(12)];
Z_F = [u(7);u(8);u(9);u(10)];
rho_0 = u(13);

C1 = [c1y;c1phi];
C2 = [c2y;c2phi];
K1 = [k1y;k1phi];
K2 = [k2y;k2phi];

% 第一层 PI 和 RBF
s1 = z1+K1.*I1;
Z_J1 = [Z_F;z1;I1];
Phi_J1 = AGV_RBF(Z_J1,'J1');
F1_hat = zeros(2,1);
alpha1 = varsigma\(-C1.*s1+Gamma-K1.*z1-F1_hat-0.5*WA1'*Phi_J1);

dalpha1_f = (alpha1-alpha1_f)/tau_alpha1;

% 第二层 PI 和 RBF
z2 = chi2-alpha1_f-O;
s2 = z2+K2.*I2;
Phi_F2 = AGV_RBF(Z_F,'F');
Z_J2 = [Z_F;z1;I1;z2;I2;O;alpha1_f];
Phi_J2 = AGV_RBF(Z_J2,'J2');
% 与 mdlDerivatives 相同：F2_hat 只表示 dot(chi2)-g_delta(t)*delta_sat。
F2_hat = WF2'*Phi_F2;
F2_PI = F2_hat-dalpha1_f+O+K2.*z2;

% 车辆真实输入增益和最终控制量
cf = cf0*(1+cf_rate*sin(0.01*t));
g_delta = [cf/m;lf*cf/Iz];
p_a2 = 2*C2.*s2+2*F2_PI+WA2'*Phi_J2;
delta_feedback = -(g_delta'*p_a2)/(2*r_delta);
delta_feedforward = rho_ff_gain*rho_0; % 车辆模型的曲率前馈系数
delta = delta_feedback+delta_feedforward;
delta_sat = min(max(delta,-u_d),u_d);

W = norm([WC1(:);WA1(:);WF2(:);WC2(:);WA2(:)]);

% 前三个是控制器主输出，后六个供 plot 记录误差变量。
sys = [delta;delta_sat;W;s1;s2;z2];
end
