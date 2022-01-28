function trajectory = spirale()
    
    pos = @(t)[cos(t), sin(t),  t/5];
    vel = @(t)[-sin(t), cos(t), 1/5];
    acc = @(t)[-cos(t), -sin(t), 0];
    att = zeros(1,3);
    
    trajectory = @(t)[pos(t); att; vel(t); acc(t)];
end
