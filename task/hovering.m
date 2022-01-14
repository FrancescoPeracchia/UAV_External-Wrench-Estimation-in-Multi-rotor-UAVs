function trajectory = hovering(h)
    position = [0, 0, h];
    
    trajectory = @(t)[position; zeros(1,3); zeros(1,3); zeros(1,3)];
end
