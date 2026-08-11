function S = AGV_RBF(Z,type)
% AGV_RBF  论文中的普通Gaussian RBF基函数
% F网络输入：Z_F=[e_y,e_phi,de_y,de_phi]
% J1网络输入：Z_J1=[Z_F;z1;I1]
% J2网络输入：Z_J2=[Z_F;z1;I1;z2;I2;O;alpha1_f]
% 公式：S_j(Z)=exp(-(Z-c_j)'(Z-c_j)/a^2)

if nargin < 2
    type = 'F';
end
Z = Z(:);

% ========================== RBF参数 ==========================
N = 7;
a = 1.2;                         % Gaussian宽度

% 物理状态的7个中心
c_F = [ ...
    -1.0, -0.5, -0.2, 0, 0.2, 0.5, 1.0;
    -0.3, -0.1, -0.05, 0, 0.05, 0.1, 0.3;
    -0.5, -0.2, 0, 0, 0, 0.2, 0.5;
    -0.2, -0.05, 0, 0, 0, 0.05, 0.2];

if strcmpi(type,'F')
    if numel(Z) ~= 4
        error('AGV_RBF:Input','F网络输入必须是4维物理状态。');
    end
    c = c_F;
    % 让e_phi和de_phi的量纲与论文中的归一化状态一致。
    scale = [1;0.1;1;0.3];
elseif strcmpi(type,'J1')
    if numel(Z) ~= 8
        error('AGV_RBF:Input','J1网络输入必须是[Z_F;z1;I1]八维状态。');
    end
    grid = [-1.0,-0.5,-0.2,0,0.2,0.5,1.0];
    c = [c_F;repmat(grid,4,1)];
    scale = [1;0.1;1;0.3;1;1;1;1];
elseif strcmpi(type,'J2')
    if numel(Z) ~= 16
        error('AGV_RBF:Input','J2网络输入必须是16维闭合状态。');
    end
    grid = [-1.0,-0.5,-0.2,0,0.2,0.5,1.0];
    c = [c_F;repmat(grid,12,1)];
    scale = ones(16,1);
    scale(1:4) = [1;0.1;1;0.3];
else
    error('AGV_RBF:Type','type只能是F、J1或J2。');
end

Z = Z./scale;
c = c./scale;
S = zeros(N,1);
for j = 1:N
    d = Z-c(:,j);
    S(j) = exp(-(d'*d)/(a^2));
end
