clear all
clc

% init connection
[sim, id] = API.simConnect();

% coppeliasim step
[~,step] = sim.simxGetFloatSignal(id, 'Simulation/step', sim.simx_opmode_blocking);
sim.simxSetFloatSignal(id, 'Simulation/start', 0, sim.simx_opmode_oneshot);

% sampling time and duration of experiment
T = 10;
    
% init robot model
robot = SimQuadcopter(sim, id);
initial_state = [0 0 0.1 0 0 0 0 0 0 0 0 0];
robot.init(initial_state);

% initialization of task
trajectory = spirale();

% control loop
for current_time = 0:step:T
    
    disp(["time: ", current_time]);
    
    % get next desired position and orientation 
    traj = trajectory(current_time);
    
    % compute the control
    u = robot.control(traj);
    disp(["control: ", u]);
    
    % send command to robot
    state = robot.command(u);
   
    disp(["state: ", state]);
end

% close connection
API.simClose(sim,id)

% plot results
%plots;