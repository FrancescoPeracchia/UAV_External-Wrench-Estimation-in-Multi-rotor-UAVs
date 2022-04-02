function trajectory = hovering(h)
    % ELICOIDAL TRAJECOTRY
    % ARGS:
    %   - height of the vertical path
    % RETRUN:
    %   - quadcopter state at time @t: only the position changes, while
    %     velocity and accelerarion remain constants
    position = [0, 0, h];
    trajectory = @(t)[position; zeros(1,3); zeros(1,3); zeros(1,3)];
end
