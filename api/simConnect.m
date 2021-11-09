% in a child script of a CoppeliaSim scene, add following command
% simRemoteApi.start(19999)


function [sim, id] = simConnect()
    sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    id = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (id>-1)

        % enable the synchronous mode on the client:
        sim.simxSynchronous(id, true);

        % start the simulation:
        sim.simxStartSimulation(id, sim.simx_opmode_blocking);
        
    else
        disp('Failed connecting to remote API server');
    end
end