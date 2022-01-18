function trajectory = hovering(h, yaw)
    position = [0, 0, h];
    if ~exist('yaw','var')
        yaw = 0;
    end
    trajectory = @(t)[position; [0 0 yaw]; zeros(1,3); zeros(1,3)];
end
