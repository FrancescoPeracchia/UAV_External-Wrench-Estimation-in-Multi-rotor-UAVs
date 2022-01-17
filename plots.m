figure(1)
plot3(Position.signals.values(:,1),Position.signals.values(:,2),Position.signals.values(:,3));
hold on
%uncomment for spiral tracking task
plot3(cos(Attitude.time),sin(Attitude.time),2*Attitude.time,'r')

grid on
figure(2)
hold on
plot(Attitude.time,Attitude.signals.values(:,1),'r');
plot(Attitude.time,Attitude.signals.values(:,2),'g');
plot(Attitude.time,Attitude.signals.values(:,3),'b');
grid on
hold off