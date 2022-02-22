clear all
clc

% init connection
[sim, id] = API.simConnect();

% coppeliasim step
[~,step] = sim.simxGetFloatSignal(id, 'Simulation/step', sim.simx_opmode_blocking);
sim.simxSetFloatSignal(id, 'Simulation/start', 0, sim.simx_opmode_oneshot);

% sampling time and duration of experiment
T = 20;
    
% init robot model
robot = SimQuadcopter(sim, id);
initial_state = [0 0 0.2 0 0 0 0 0 0 0 0 0];
robot.init(initial_state);

% initialization of task
%trajectory = spirale();
trajectory = hovering(1);

% control loop
wrench = zeros(2,T/step);
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
    
    % display
    F_e = robot.quadcopter.F_hat; 
    disp(["time: ", current_time, "wrench: ", F_e]);
    wrench(:,i) = [norm(F_e(1:3)); norm(F_e(4:6))];
end

% close connection
API.simClose(sim,id)

% plot results
plots;