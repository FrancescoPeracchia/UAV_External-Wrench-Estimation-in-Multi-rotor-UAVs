% in a child script of a CoppeliaSim scene, add following command
% simRemoteApi.start(19999)

classdef API
    methods(Static)
          
        function [sim, id] = simConnect()
            sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
            sim.simxFinish(-1); % just in case, close all opened connections
            id = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

            if (id>-1)

                % enable the synchronous mode on the client:
                sim.simxSynchronous(id, true);

                % start the simulation:
                sim.simxStartSimulation(id, sim.simx_opmode_blocking);
                
                disp('Connected to remote API server');

            else
                error('Failed connecting to remote API server');
                
            end
        end
        
        function simClose(sim, id)
            % stop the simulation:
            sim.simxStopSimulation(id, sim.simx_opmode_blocking);
            % Now close the connection to CoppeliaSim:    
            sim.simxFinish(id);
            sim.delete(); % call the destructor!
        end
    end
end