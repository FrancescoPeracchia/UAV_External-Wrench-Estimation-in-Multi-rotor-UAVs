function trajectory = spirale()
    
    pos = @(t)[cos(t), sin(t), 2*t];
    vel = @(t)[-sin(t), cos(t), 2];
    acc = @(t)[-cos(t), -sin(t), 0];
    att = zeros(1,3);
    
    trajectory = [pos; att; vel; acc];
end

