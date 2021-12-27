clear
clc

% init connection
[sim, id] = API.simConnect();
if (id <0) 
    return;
end

% Now step a few times:
for i=0:10
    sim.simxSetFloatSignal(id, 'u', i+90, sim.simx_opmode_oneshot);
    pause;
    sim.simxSynchronousTrigger(id);
end

% close connection
API.simClose(sim,id)