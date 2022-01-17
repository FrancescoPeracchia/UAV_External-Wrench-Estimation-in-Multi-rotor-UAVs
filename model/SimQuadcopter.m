classdef SimQuadcopter < handle
    % SIMQUADCOPTER
    %   handles data exchange between matlab and coppelia
    %   inheriting the Quadcopter class functions
    
    properties
        sim
        id
        handle
        quadcopter
    end
    
    methods
        
        function obj = SimQuadcopter(sim,id)
            % SIMQUADCOPTER constructor
            obj.sim = sim;
            obj.id = id;
            [~,obj.handle] = sim.simxGetObjectHandle(id, 'Quadcopter', sim.simx_opmode_blocking);
            params = obj.getSimParams(sim,id);
            obj.quadcopter = Quadcopter(params);
        end
        
        function obj = init(obj,state)
            % INIT the intial state
            obj.quadcopter.update(state);
            [x,y,z,phi,theta,psi] = deal(state(1),state(2),state(3),state(7),state(8),state(9));
            obj.sim.simxSetObjectPosition(obj.id, obj.handle, -1, [x y z], obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetObjectOrientation(obj.id, obj.handle, -1, [phi theta psi], obj.sim.simx_opmode_oneshot);
        end
        
        function obj = update(obj,state)
            % UPDATE inherits the update function
            obj.quadcopter.update(state);
        end
        
        function u = control(obj,traj)
            % CONTROL inherits the control function
            u = obj.quadcopter.control(traj);
        end
        
        function state = command(obj,u)
            % COMMAND inherits the command function 
            % and send commands to simulator
            [T,tau_phi,tau_theta,tau_psi] = deal(u(1),u(2),u(3),u(4));
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/thrust', T, obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/torque/phi', tau_phi, obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/torque/theta', tau_theta, obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/torque/psi', tau_psi, obj.sim.simx_opmode_oneshot);
            
            [~,p] = obj.sim.simxGetObjectPosition(obj.id, obj.handle, -1, obj.sim.simx_opmode_blocking);
            [~,xyz] = obj.sim.simxGetObjectOrientation(obj.id, obj.handle, -1, obj.sim.simx_opmode_blocking);
            [~,v,w] = obj.sim.simxGetObjectVelocity(obj.id, obj.handle, obj.sim.simx_opmode_blocking);
            state = [p v xyz2rpy(xyz) w];
            obj.quadcopter.update(state);
        end
    end
    
    methods(Static)
        
        function params = getSimParams(sim,id)
            % GETPARAMS gets quadrotor parameters from coppelia
            [~,g] = sim.simxGetFloatSignal(id,'Physics/gravity',sim.simx_opmode_blocking);
            [~,m] = sim.simxGetFloatSignal(id,'UAV/mass',sim.simx_opmode_blocking);    
            [~,Ix] = sim.simxGetFloatSignal(id,'UAV/inertia/x',sim.simx_opmode_blocking);    
            [~,Iy] = sim.simxGetFloatSignal(id,'UAV/inertia/y',sim.simx_opmode_blocking);    
            [~,Iz] = sim.simxGetFloatSignal(id,'UAV/inertia/z',sim.simx_opmode_blocking);    
    
            params = [g;m;0;Ix;Iy;Iz;0;0];        
        end
    
    end
end