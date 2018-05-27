function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Linearised 3D controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

yaw = des_state.yaw;
yaw_dot = des_state.yawdot;
g = params.gravity;

% =================== Your code goes here ===================

% Calculate Translational Errors (3x1 vectors for brevity)

e = des_state.pos - state.pos;
e_dot = des_state.vel - state.vel;
%e_ddot = des_state.acc - state.acc;

% Define gains

ky_p = 1.25*4.872;
ky_d = 2*sqrt(ky_p);

kx_p= ky_p;
kx_d= ky_d;

kz_p=ky_p*1000;
kz_d=2*sqrt(kz_p);

kphi_p = ky_p*82;
kphi_d =0.95*2*0.9103*sqrt(kphi_p);

ktheta_p=ky_p*82;
ktheta_d=0.95*2*0.9103*sqrt(kphi_p);

kpsi_p=kphi_p*0.5;
kpsi_d=0.95*2*0.9103*sqrt(kpsi_p);


% Calculate desired angles (Inner Loop Controller) NOTE
% THESE ARE IN BODY-FIXED AXES

phi_des = (1/g) * ((des_state.acc(1)+kx_p*e(1)+kx_d*e_dot(1))*sin(yaw) - ...
    (des_state.acc(2)+ky_p*e(2)+ ky_d*e_dot(2))*cos(yaw));

theta_des = (1/g) * ((des_state.acc(1)+kx_p*e(1)+kx_d*e_dot(1))*cos(yaw) + ...
    (des_state.acc(2)+ky_p*e(2)+ ky_d*e_dot(2))*sin(yaw));
               
% Thrust
F = params.mass*(params.gravity + des_state.acc(3) +kz_p *e(3) + kz_d*e_dot(3));

% Moment
M = zeros(3,1);

Theta_dot = zeros(3,1);

Theta_dot(1) = kphi_p*(phi_des-state.rot(1))-kphi_d*(state.omega(1));
Theta_dot(2) = ktheta_p*(theta_des-state.rot(2))-ktheta_d*(state.omega(2));
Theta_dot(3) = kpsi_p*(yaw-state.rot(3))+kpsi_d*(yaw_dot-state.omega(3));

M = params.I*Theta_dot;


% =================== Your code ends here ===================

end
