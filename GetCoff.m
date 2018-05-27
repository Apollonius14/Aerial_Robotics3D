function [coffs] = GetCoff(waypoints,times,degree)

%% To-Do: Generalise for more than five waypoints

%%
% A function that returns the coefficients of a piecewise-assembled
% 7th order (minimum jerk) polynomial trajectory

% Params:
% waypoints: 1 dimensional waypoints in a row vector
% times: 1 dimensional times corresponding to waypoints
% degree (7 for mininum jerk) of polynomial

% number of segments
n = size(waypoints,2)-1
% degree of polynomial + 1 to yield coefficients
m = degree+1;

% We want a an equation A * coff = b where 
% A is a matrix of time values size ([n*m] x [n*m])
% coff are the polynomial coefficients we're looking for
% b are the set of constrains from our waypoints

A = zeros(n*m,n*m);
b = zeros(n*m,1);

size(A)
%% 
% First off we assemble our matrix of time values A

% Position constraints (2 per segment - so 2n in total)

for i = 1:n
    A(i,(i-1)*m+1:(i*m)) = time_vec(m,0,times(i));
    b(i) = waypoints(i);
    A(i+n,(i-1)*m+1:(i*m)) = time_vec(m,0,times(i+1));
    b(i+n) = waypoints(i+1);
end

% Velocity constraints
% First waypoint velocity = 0
A(2*n+1,1:m) = time_vec(m,1,times(1));
% Final wapoint velocity = 0
A(3*n+1,(n-1)*m+1:n*m) = time_vec(m,1,times(n+1));

for i = 1:n-1
    A(2*n+1+i,(i-1)*m+1:i*m) = time_vec(m,1,times(i+1));
    A(2*n+1+i,(i*m)+1:(i+1)*m) = -1*time_vec(m,1,times(i+1));
end

% Accelerations

% First waypint acc = 0
A(3*n+2,1:m) = time_vec(m,2,times(1));
%Final waypoint acc = 0
A(4*n+2,(n-1)*m+1:n*m) = time_vec(m,2,times(n+1));

for i = 1:n-1
    A(3*n+2+i,(i-1)*m+1:i*m) = time_vec(m,2,times(i+1));
    A(3*n+2+i,(i*m)+1:(i+1)*m) = -1*time_vec(m,2,times(i+1));
end


% Third Derivatives

%First position
A(4*n+3,1:m) = time_vec(m,3,times(1));
%Final position
A(5*n+3,(n-1)*m+1:n*m) = time_vec(m,3,times(n+1));

for i = 1:n-1
    A(4*n+3+i,(i-1)*m+1:i*m) = time_vec(m,3,times(i+1));
    A(4*n+3+i,(i*m)+1:(i+1)*m) = -1*time_vec(m,3,times(i+1));
end

% 4th derivatives
for i = 1:n-1
    A(5*n+3+i,(i-1)*m+1:i*m) = time_vec(m,4,times(i+1));
    A(5*n+3+i,(i*m)+1:(i+1)*m) = -1*time_vec(m,4,times(i+1));
end

% 5th derivatives
for i = 1:n-1
    A(6*n+2+i,(i-1)*m+1:i*m) = time_vec(m,5,times(i+1));
    A(6*n+2+i,(i*m)+1:(i+1)*m) = -1*time_vec(m,5,times(i+1));
end

% 6th derivatives
for i = 1:n-1
    A(7*n+1+i,(i-1)*m+1:i*m) = time_vec(m,6,times(i+1));
    A(7*n+1+i,(i*m)+1:(i+1)*m) = -1*time_vec(m,6,times(i+1));
end

A;
size(A)
%%
coffs = inv(A)*b;

end