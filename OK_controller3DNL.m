function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  non-linear 3D controller for the quadrotor
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

state.rot(1);
yaw = des_state.yaw;
yaw_dot = des_state.yawdot;
g = params.gravity;
R = RPYtoRot_ZXY(state.rot(1),state.rot(2),state.rot(3));
% =================== Your code goes here ===================

% Calculate Translational Errors (3x1 vectors for brevity)

e = des_state.pos - state.pos;
e_dot = des_state.vel - state.vel;
%e_ddot = des_state.acc - state.acc;

%%
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

%% OLD GAINS
%ky_p = 45*params.mass;
%ky_d = 0.9103*sqrt(ky_p);
%
%kx_p= ky_p;
%kx_d= ky_d;
%
%kz_p=ky_p*3;
%kz_d=2*sqrt(kz_p);
%
%kphi_p = 150*ky_p;
%kphi_d =0.95*2*0.9103*sqrt(kphi_p);
%
%ktheta_p=kphi_p;
%ktheta_d=0.95*2*0.9103*sqrt(kphi_p);
%
%kpsi_p=0.5*kphi_p;
%kpsi_d=0.95*2*0.9103*sqrt(kpsi_p);

KpNL = [kx_p,0,0;0,ky_p,0;0,0,kz_p];
KdNL = [kx_d,0,0;0,ky_d,0;0,0,kz_d];

%% Controller

% First we define the direction of ideal thrust (assuming
% our drone is fully actuated in INERTIAL FRAME

t_err =  params.mass*([0;0;g]  + des_state.acc) + KpNL*e + KdNL*e_dot;

% Calculating Thrust by projecting t_err onto b3 = R*[0;0;1]

F = t_err'*R*[0;0;1];

% Now we obtain the rotation required to align t_err 
% with the body_fixed pure thrust axis b3 = R*[0;0;1]

% First get the axis of rotation
%axis = -cross(t_err,R*[0;0;1]);
% Second get the angle
%angle = acos(dot(t_err,R*[0;0;1])/norm(t_err))
% Now the rotation matrix from the axis and angle
%R_des = axisrot(axis,angle);

R_des = desrot(t_err,yaw);

% Finally we obtain the delta pitch and roll angles in the
% body frame by calculating the angle through which we'd
% need to turn to get from R to R_des

R_del = R*R_des



%F = params.mass*(params.gravity + des_state.acc(3) +kz_p *e(3) + kz_d*e_dot(3)));

%% Calculating Moment
state.rot(2)
M_controlled = zeros(3,1);

M_coupled = cross(state.omega,params.I*state.omega);

[roll,pitch,yaw] = RotToRPY_ZXY(R_del);

K_rot = [kphi_p, 0, 0;0, ktheta_p, 0;0, 0, kpsi_p];
K_rotd = [kphi_d, 0, 0;0, ktheta_d, 0;0, 0, kpsi_d];

%Theta_dot(1) = kphi_p*(phi_des)-kphi_d*(state.omega(1));
%Theta_dot(2) = ktheta_p*(theta_des)-ktheta_d*(state.omega(2));
%Theta_dot(3) = kpsi_p*(psi_des)+kpsi_d*(yaw_dot-state.omega(3));

% Remember here that we must move from inertial angles to body-fixed
% ones in order to get the correct moments

M_controlled = R'*params.I*(K_rot*[roll;pitch;yaw]+K_rotd*state.omega);

M = M_coupled - M_controlled;


% =================== Your code ends here ===================

end
