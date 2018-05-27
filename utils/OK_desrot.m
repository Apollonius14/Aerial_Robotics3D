function drot = desrot( thrust, yaw )
% Returns the desired rotation matrix given a thrust
%   error vector and desired yaw angle
%   Detailed explanation goes here

% Need to solve equation Rdes.[0 0 1] = thrust / ||thrust||

% Evaluate RHS (t)

t = thrust / norm(thrust);

% Solve for phi, theta and yaw

% Put x1 = Sin(theta) and x(2) = Sin(phi)Cos(theta)

Coeff = [cos(yaw) sin(yaw);sin(yaw) -cos(yaw)];
x = [t(1) t(2)]/Coeff;

theta = asin(x(1));
phi = asin(x(2)/cos(theta));

% Reconstruct the rotation matrix

drot = [cos(yaw)*cos(theta)-sin(phi)*sin(yaw)*sin(theta),...
    -cos(phi)*sin(yaw),...
    cos(yaw)*sin(theta)+cos(theta)*sin(phi)*sin(yaw);...
    cos(theta)*sin(yaw)+cos(yaw)*sin(phi)*sin(theta),...
    cos(phi)*cos(yaw),...
    sin(yaw)*sin(theta)-cos(theta)*sin(phi)*cos(yaw);...
    -cos(phi)*sin(theta),...
    sin(phi),...
    cos(theta)*cos(phi)];

end