function [axis,angle] = Rotmat_to_axis(R)

% Returns angle and axis from a rotation matrix
%   Expects R as a 3x3 matrix

axis = [R(3,2)-R(2,3);...
        R(1,3)-R(3,1);...
        R(2,1)-R(1,2)];
    
angle = acos((trace(R)-1)/2);

end