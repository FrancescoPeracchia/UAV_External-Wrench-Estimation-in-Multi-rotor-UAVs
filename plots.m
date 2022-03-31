figure(1)
plot(t,wrench(1,:),t,wrench(2,:),'LineWidth',1)
title('Estimated External Force (blue) and Torque (red)')
xlabel('time')

figure(2)
plot3(gt_position(1,:), gt_position(2,:), gt_position(3,:), pr_position(1,:), pr_position(2,:), pr_position(3,:))
title('Ground truth position (blue) and real position (red)')
grid on