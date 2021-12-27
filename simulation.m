clear
clc

% coppelia sim step
step = 0.05;
% sampling time and duration of experiment
T = 30;

% init connection
[sim, id] = API.simConnect();
if (id <0) 
    return;
end

% init robot model
robot = SimQuadcopter(sim, id, step);

% initialization of task
trajectory = hovering(6);

% control loop
for current_time = 0:step:T
    
    disp(["time: ", current_time]);
    
    % get next desired configuration 
    q_des = trajectory(current_time);
    
    % compute the control
    u = robot.control(q_des);
    
    % feed the new input
    robot.command(u);
    
    e = 0;
    disp(["error: ", e]);
end

% close connection
API.simClose(sim,id)

% plot results
plots;