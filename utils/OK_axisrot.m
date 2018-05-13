function Rotmat = axisrot(u,phi)

% Returns Rotation Matrix Given Axis and Angle
%   Expects u as vertical column vector 3x1

Rotmat = eye(3)*cos(phi)+u*u'*(1-cos(phi))+uhat(u)*sin(phi);

end