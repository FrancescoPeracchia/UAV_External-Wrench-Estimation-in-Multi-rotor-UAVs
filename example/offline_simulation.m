clear all
clc
cd('..')

% simulation step
step = 0.05;
% sampling time and duration of experiment
T = 10;

% init robot model
robot = Quadcopter();
initial_state = zeros(1,12);
robot.update(initial_state);

% initialization of task
trajectory = hovering(1,pi);

% control loop
for current_time = 0:step:T
    
    disp(["time: ", current_time]);
    
    % get next desired position and orientation 
    traj = trajectory(current_time);
    
    % compute the control
    u = robot.control(traj);
    disp(["control: ", u]);
    
    % feed the new input
    state = robot.command(u,step);
    
    disp(["state: ", state]);
end