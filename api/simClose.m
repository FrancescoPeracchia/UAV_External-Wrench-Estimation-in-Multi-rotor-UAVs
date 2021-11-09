function simClose(sim, id)
    % stop the simulation:
    sim.simxStopSimulation(id, sim.simx_opmode_blocking);
    % Now close the connection to CoppeliaSim:    
    sim.simxFinish(id);
    sim.delete(); % call the destructor!
end