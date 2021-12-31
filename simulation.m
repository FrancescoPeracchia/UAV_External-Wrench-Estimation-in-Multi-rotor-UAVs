clear all
clc

% coppelia sim step
step = 0.05;
% sampling time and duration of experiment
T = 30;

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
    pose_d = trajectory(current_time);
    
    % compute the control
    u = robot.control(pose_d);
    
    % feed the new input
    robot.command(u);
    
    e = 0;
    disp(["error: ", e]);
end

% close connection
API.simClose(sim,id)

% plot results
%plots;