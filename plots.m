figure(1)
plot(t,wrench(1,:),t,wrench(2,:),'LineWidth',1)
title('Estimated External Force (blue) and Torque (red)')
xlabel('time')


if isequal(gt_position(:,1),gt_position(:,end))
    % hovering
    figure(2)
    scatter3(gt_position(1,1), gt_position(2,1), gt_position(3,1), 'filled') 
    hold on
    plot3(pr_position(1,:), pr_position(2,:), pr_position(3,:))
    hold off
    title('Ground truth (blue) and real (red) trajectories')
    grid on
else
    % spirale
    figure(2)
    plot3(gt_position(1,:), gt_position(2,:), gt_position(3,:), pr_position(1,:), pr_position(2,:), pr_position(3,:))
    title('Ground truth (blue) and real (red) trajectories')
    grid on
end