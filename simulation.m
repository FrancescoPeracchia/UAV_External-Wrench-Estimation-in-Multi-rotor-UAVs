clear all
clc

% init connection
[sim, id] = API.simConnect();

% coppeliasim step
[~,step] = sim.simxGetFloatSignal(id, 'Simulation/step', sim.simx_opmode_blocking);
sim.simxSetFloatSignal(id, 'Simulation/start', 0, sim.simx_opmode_oneshot);

% sampling time and duration of experiment
T = 30;
    
% init robot model
robot = SimQuadcopter(sim, id);
initial_state = [0 0 0.175 0 0 0 0 0 0 0 0 0];
robot.init(initial_state);

% initialization of task
%trajectory = spirale();
trajectory = hovering(1);

% control loop
for current_time = 0:step:T
    
    %disp(["time: ", current_time]);
    
    % get next desired position and orientation 
    traj = trajectory(current_time);
    
    % compute the control
    u = robot.control(traj);
    
    % send command to robot
    state = robot.command(u,step);
    
    %disp(["state: ", state]);
    disp(["wrench: ", robot.quadcopter.F_hat]);
end

% close connection
API.simClose(sim,id)

% plot results
%plots;