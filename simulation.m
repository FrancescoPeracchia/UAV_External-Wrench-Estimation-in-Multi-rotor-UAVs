% init connection
[sim, id] = simConnect();

% Now step a few times:
for i=0:10
    disp('Press a key to step the simulation!');
    pause;
    sim.simxSynchronousTrigger(id);
end

% close connection
simClose(sim,id)