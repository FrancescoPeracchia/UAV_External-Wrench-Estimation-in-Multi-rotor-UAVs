function rpy = abg2rpy(abg)
% inputs:
%   -abg: euler orientation angles 
% outputs:
%   -rpy: roll, pitch, yaw orientation angles
    R = euler_rotation('xyz',abg);
    r = atan2(R(3,2),R(3,3));
    p = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    y = atan2(R(2,1),R(1,1));
    rpy = [r p y];
end

