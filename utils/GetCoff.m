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
n = size(waypoints,2)-1;
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

% First position
A(1,1:m) = time_vec(m,0,times(1));
b(1) = waypoints(1);
A(n+1,1:m) = time_vec(m,0,times(2));
b(n+1) = waypoints(2);

% 2 Equations down

% Remaining positions
for i = 2:n
    A(i,(i-1)*m+1:(i*m)) = time_vec(m,0,times(i));
    b(i) = waypoints(i);
    A(i+n,(i-1)*m+1:(i*m)) = time_vec(m,0,times(i+1));
    b(i+n) = waypoints(i+1);
end
b;
% 2n equations down

%Now for first derivatives (2n-2)

% First position
A(2*n+1,1:m) = time_vec(m,1,times(1));

% Middle positions
A(2*n+2,1:m) = time_vec(m,1,times(2));
A(2*n+2,m+1:2*m) = -1*time_vec(m,1,times(2));

A(2*n+3,m+1:2*m) = time_vec(m,1,times(3));
A(2*n+3,2*m+1:3*m) = -1*time_vec(m,1,times(3));

A(2*n+4,2*m+1:3*m) = time_vec(m,1,times(4));
A(2*n+4,3*m+1:4*m) = -1*time_vec(m,1,times(4));

%Final position
A(2*n+5,3*m+1:4*m) = time_vec(m,1,times(5));

% Accelerations

% First position
A(2*n+6,1:m) = time_vec(m,2,times(1));

% Middle positions
A(2*n+7,1:m) = time_vec(m,2,times(2));
A(2*n+7,m+1:2*m) = -1*time_vec(m,2,times(2));

A(2*n+8,m+1:2*m) = time_vec(m,2,times(3));
A(2*n+8,2*m+1:3*m) = -1*time_vec(m,2,times(3));

A(2*n+9,2*m+1:3*m) = time_vec(m,2,times(4));
A(2*n+9,3*m+1:4*m) = -1*time_vec(m,2,times(4));

%Final position
A(2*n+10,3*m+1:4*m) = time_vec(m,2,times(5));

% Third Derivatives

%First position
A(2*n+11,1:m) = time_vec(m,3,times(1));

% Middle positions
A(2*n+12,1:m) = time_vec(m,3,times(2));
A(2*n+12,m+1:2*m) = -1*time_vec(m,3,times(2));

A(2*n+13,m+1:2*m) = time_vec(m,3,times(3));
A(2*n+13,2*m+1:3*m) = -1*time_vec(m,3,times(3));

A(2*n+14,2*m+1:3*m) = time_vec(m,3,times(4));
A(2*n+14,3*m+1:4*m) = -1*time_vec(m,3,times(4));

%Final position
A(2*n+15,3*m+1:4*m) = time_vec(m,3,times(5));

% 4th derivatives
A(2*n+16,1:m) = time_vec(m,4,times(2));
A(2*n+16,m+1:2*m) = -1*time_vec(m,4,times(2));

A(2*n+17,m+1:2*m) = time_vec(m,4,times(3));
A(2*n+17,2*m+1:3*m) = -1*time_vec(m,4,times(3));

A(2*n+18,2*m+1:3*m) = time_vec(m,4,times(4));
A(2*n+18,3*m+1:4*m) = -1*time_vec(m,4,times(4));

% 5th derivatives
A(2*n+19,1:m) = time_vec(m,5,times(2));
A(2*n+19,m+1:2*m) = -1*time_vec(m,5,times(2));

A(2*n+20,m+1:2*m) = time_vec(m,5,times(3));
A(2*n+20,2*m+1:3*m) = -1*time_vec(m,5,times(3));

A(2*n+21,2*m+1:3*m) = time_vec(m,5,times(4));
A(2*n+21,3*m+1:4*m) = -1*time_vec(m,5,times(4));

%6th derivatives
A(2*n+22,1:m) = time_vec(m,6,times(2));
A(2*n+22,m+1:2*m) = -1*time_vec(m,6,times(2));

A(2*n+23,m+1:2*m) = time_vec(m,6,times(3));
A(2*n+23,2*m+1:3*m) = -1*time_vec(m,6,times(3));

A(2*n+24,2*m+1:3*m) = time_vec(m,6,times(4));
A(2*n+24,3*m+1:4*m) = -1*time_vec(m,6,times(4));

A;
size(A)
%%
coffs = inv(A)*b;

end