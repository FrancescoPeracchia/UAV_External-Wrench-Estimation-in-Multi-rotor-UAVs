clear all
clc

% coppelia sim step
step = 0.05;
% sampling time and duration of experiment
T = 10;

% init connection
[sim, id] = API.simConnect();

% init robot model
robot = SimQuadcopter(sim, id, step);
initial_state = zeros(1,12);
robot.init(initial_state);

% initialization of task
trajectory = hovering(6);

% control loop
for current_time = 0:step:T
    
    disp(["time: ", current_time]);
    
    % get next desired position and orientation 
    traj = trajectory(current_time);
    
    % compute the control
    u = robot.control(traj);
    
    % feed the new input
    state = robot.command(u,step);
    
    disp(["error: ", state]);
end

% close connection
API.simClose(sim,id)

% plot results
%plots;