clear all
clc

% simulation step
step = 0.05;
% sampling time and duration of experiment
T = 20;

% init robot model
robot = Quadcopter();
initial_state = zeros(1,12);
robot.state = initial_state;
robot.F_hat(3) = -3;

% initialization of task
trajectory = hovering(1);

% control loop
t = 0:step:T;
for i = 1:length(t)
    
    % simulation time
    current_time = t(i);
    
    % get next desired position and orientation 
    traj = trajectory(current_time);
    
    % compute the control
    u = robot.control(traj);
    
    % send command to robot
    state = robot.command(u,step);
    
    disp(["time: ", current_time, "state: ", state]);
end