function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 xcoffs ycoffs zcoffs

if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 0.4*sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2)
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    xcoffs = GetCoff(waypoints0(1,:),traj_time,7)
    ycoffs = GetCoff(waypoints0(2,:),traj_time,7);
    zcoffs = GetCoff(waypoints0(3,:),traj_time,7);
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    %if(t_index > 1)
    %    t = t - traj_time(t_index-1);
    %end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    else
        tmat = [1,t,t^2,t^3,t^4,t^5,t^6,t^7]';
        tvmat = [0,1,2*t,3*t^2,4*t^3,5*t^4,6*t^5,7*t^6]';
        tamat = [0,0,2,6*t,12*t^2,20*t^3,30*t^4,42*t^5]';
        seg_s = 1+(t_index-2) * 8
        seg_e = seg_s + 7
        
        desired_state.pos = [xcoffs(seg_s:seg_e)'*tmat;...
                            ycoffs(seg_s:seg_e)'*tmat;...
                            zcoffs(seg_s:seg_e)'*tmat];
                        
        %desired_state.pos(1)
        %desired_state.pos(2)
        %desired_state.pos(3)
        
        desired_state.vel = [xcoffs(seg_s:seg_e)'*tvmat;...
                            ycoffs(seg_s:seg_e)'*tvmat;...
                            zcoffs(seg_s:seg_e)'*tvmat];
                        
        desired_state.acc = [xcoffs(seg_s:seg_e)'*tamat;...
                            ycoffs(seg_s:seg_e)'*tamat;...
                            zcoffs(seg_s:seg_e)'*tamat];
                        
        desired_state.yaw = 0;
        
        %scale = t/d0(t_index-1);
        %desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%%

