function R = rpy_rotation(axes, angles)
% inputs:
%   -axes: the sequence of axis 
%   -angles: the radiants (or a symbolic) we should perform the rotation of
% outputs:
%   -R: The desired rpy rotation

	R = rotation(axes(3), angles(3)) * rotation(axes(2), angles(2)) * rotation(axes(1), angles(1));
	
end