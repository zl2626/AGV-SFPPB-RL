function [sys,x0,str,ts] = AGV_plant(t,x,u,flag)
% AGV_PLANT  车辆横向误差模型。
% 输入顺序：[delta_sat,rho_0]。

switch flag
    case 0  % 初始化
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1  % 连续状态导数
        sys = mdlDerivatives(t,x,u);
    case 3  % 输出
        sys = mdlOutputs(t,x,u);
    case 9  % 终止
        sys = [];
    otherwise
        sys = [];
end

%% 初始化函数
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 5;   % [e_y,e_phi,v_y,omega_z,rho_0]
sizes.NumDiscStates  = 0;   % 0个离散状态
sizes.NumOutputs     = 6;   % 6个输出
sizes.NumInputs      = 2;   % 2个输入 (delta_sat, rho_0)
sizes.DirFeedthrough = 0;   % 输出只使用状态，避免车辆-控制器代数环
sizes.NumSampleTimes = 1;   % 1个采样时间

sys = simsizes(sizes);
x0  = [-0.1; 0.01; 0; 0; 0];     % 初始误差、侧向速度、横摆角速度和曲率状态
str = [];
ts  = [0 0];  % 连续系统

%% 导数计算函数
function sys = mdlDerivatives(t,x,u)
global vx_vehicle
if isempty(vx_vehicle)
    vx_vehicle = 20;
end
% 状态解包：车辆状态使用侧向速度和横摆角速度。
e_y = x(1);       e_phi = x(2);
v_y = x(3);       omega_z = x(4);
rho_0_state = x(5);

% 输入解包：上游控制器已经完成唯一一次饱和。
delta_sat = u(1);
rho_0 = u(2);
rho_0_filter_tau = 0.001;

% 车辆物理参数
m = 1832;
Iz = 2488;
lf = 1.18;
lr = 1.77;
vx = vx_vehicle;

% 时变轮胎侧偏刚度
cf_nom = 80000;
cr_nom = 120000;
cf = cf_nom * (1 + 0.1*sin(0.01*t));
cr = cr_nom * (1 + 0.1*sin(0.01*t));

% 动力学模型参数
A11 = -(cf + cr)/(m*vx);
A12 = (-lf*cf + lr*cr)/(m*vx) - vx;
A21 = (-lf*cf + lr*cr)/(Iz*vx);
A22 = -(lf^2*cf + lr^2*cr)/(Iz*vx);
B1 = cf/m;
B2 = lf*cf/Iz;

% 误差运动学：横向误差导数包含纵向速度与航向误差耦合。
% 控制器看到的曲率也是同一个滤波状态，避免 de_phi 出现两套定义。
de_y = v_y + vx*e_phi;
de_phi = omega_z - vx*rho_0_state;

% 扰动项。横向加速度和横摆角加速度使用各自的量纲参数。
% 默认值采用有量纲的物理扰动；压力测试时由命令行临时覆盖。
global disturbance_y_amplitude disturbance_phi_amplitude
if isempty(disturbance_y_amplitude)
    disturbance_y_amplitude = 0.5;  % m/s^2
end
if isempty(disturbance_phi_amplitude)
    disturbance_phi_amplitude = 0.1; % rad/s^2
end
disturbance_period = 8;
current_phase = mod(t, disturbance_period);
disturbance_y = 0;
disturbance_phi = 0;
if current_phase > (disturbance_period - 1)
    disturbance_y = disturbance_y_amplitude*sin(2*pi*2*t);
    disturbance_phi = disturbance_phi_amplitude*sin(2*pi*2*t);
end

% 物理车辆动力学中的外部扰动。
% 道路曲率只通过上面的误差运动学进入，不在 v_y、omega_z 动力学中重复加入。
D = [0;
     0;
     disturbance_y;
     disturbance_phi];

% 侧向速度和横摆角速度的动力学。
dv_y = A11*v_y + A12*omega_z + B1*delta_sat + D(3);
domega_z = A21*v_y + A22*omega_z + B2*delta_sat + D(4);

% 状态导数顺序仍为 [e_y,e_phi,v_y,omega_z]。
d_rho_0_state = (rho_0-rho_0_state)/rho_0_filter_tau;
dX_dt = [de_y; de_phi; dv_y; domega_z; d_rho_0_state];

% 只积分一套车辆状态，避免再建立并行的误差/车辆动力学。
sys = dX_dt;

%% 输出函数
function sys = mdlOutputs(~,x,~)
% 根据同一套状态给出误差导数和车辆可视化量。
global vx_vehicle
if isempty(vx_vehicle)
    vx_vehicle = 20;
end
vx = vx_vehicle;
v_y = x(3);
omega_z = x(4);
rho_0_state = x(5);
de_y = v_y + vx*x(2);
de_phi = omega_z - vx*rho_0_state;
sys = [x(1);x(2);de_y;de_phi;v_y;omega_z];
