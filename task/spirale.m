function trajectory = spirale()
    % ELICOIDAL TRAJECOTRY
    % RETRUN:
    %   - quadcopter state (position, velocity, acceleration) at time @t
    %     along the desired trajectory
    
    pos = @(t)[cos(t)-1, sin(t),  0.2+t/5];
    vel = @(t)[-sin(t), cos(t), 1/5];
    acc = @(t)[-cos(t), -sin(t), 0];
    att = zeros(1,3);
    
    trajectory = @(t)[pos(t); att; vel(t); acc(t)];
end
