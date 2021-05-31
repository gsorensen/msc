% Utilises the Matlab symbolic toolbox 
% and the MSS toolbox from Fossen and Perez (2004)
clear all;
syms dx dy dz qw qx qy qz real;
q = [qw qx qy qz ]';
da = [dx dy dz ]';
R_hat = eye(3) + 2 * q(1) * Smtrx(q(2:4)) + 2 * Smtrx(q(2:4))^2;
R = R_hat*(eye(3) + Smtrx(da));
yaw = atan2(R(2,1),R(1,1))
H_yaw = jacobian(yaw, da)