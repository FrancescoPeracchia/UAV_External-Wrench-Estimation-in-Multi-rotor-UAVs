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
            [~,g] = sim.simxGetFloatSignal(id,'Physics/gravity',sim.simx_opmode_blocking);
            [~,m] = sim.simxGetFloatSignal(id,'UAV/mass',sim.simx_opmode_blocking);    
            [~,Ix] = sim.simxGetFloatSignal(id,'UAV/inertia/x',sim.simx_opmode_blocking);    
            [~,Iy] = sim.simxGetFloatSignal(id,'UAV/inertia/y',sim.simx_opmode_blocking);    
            [~,Iz] = sim.simxGetFloatSignal(id,'UAV/inertia/z',sim.simx_opmode_blocking);    
            params = [g;m;0;Ix;Iy;Iz;0;0];        
            obj.quadcopter = Quadcopter(params);
        end
        
        function obj = init(obj,state)
            % INIT the intial state
            obj.quadcopter.state = state;
            [x,y,z,phi,theta,psi] = deal(state(1),state(2),state(3),state(7),state(8),state(9));
            obj.sim.simxSetObjectPosition(obj.id, obj.handle, -1, [x y z], obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetObjectOrientation(obj.id, obj.handle, -1, [phi theta psi], obj.sim.simx_opmode_oneshot);
        end
        
        function u = control(obj,traj)
            % CONTROL inherits the control function
            u = obj.quadcopter.control(traj);
        end
        
        function state = command(obj,u,t)
            % COMMAND send commands to quadrotor and retrive state
            
            % send commands to simulator
            [T,tau_phi,tau_theta,tau_psi] = deal(u(1),u(2),u(3),u(4));
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/thrust', T, obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/torque/phi', tau_phi, obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/torque/theta', tau_theta, obj.sim.simx_opmode_oneshot);
            obj.sim.simxSetFloatSignal(obj.id, 'UAV/control/torque/psi', tau_psi, obj.sim.simx_opmode_oneshot);
            
            % read state from simulator
            [~,x] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/position/x', obj.sim.simx_opmode_streaming);
            [~,y] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/position/y', obj.sim.simx_opmode_streaming);
            [~,z] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/position/z', obj.sim.simx_opmode_streaming);
            [~,vx] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/velocity/linear/x', obj.sim.simx_opmode_streaming);
            [~,vy] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/velocity/linear/y', obj.sim.simx_opmode_streaming);
            [~,vz] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/velocity/linear/z', obj.sim.simx_opmode_streaming);
            [~,a] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/orientation/alpha', obj.sim.simx_opmode_streaming);
            [~,b] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/orientation/beta', obj.sim.simx_opmode_streaming);
            [~,g] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/orientation/gamma', obj.sim.simx_opmode_streaming);
            rpy = abg2rpy([a b g]);
            [~,wx] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/velocity/angular/x', obj.sim.simx_opmode_streaming);
            [~,wy] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/velocity/angular/y', obj.sim.simx_opmode_streaming);
            [~,wz] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/state/velocity/angular/z', obj.sim.simx_opmode_streaming);
            
            state = [x y z vx vy vz rpy wx wy wz];
            obj.quadcopter.state = state;
            
            % read acceleration from accelerometer
            [~,ax] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/sensor/acceleration/x', obj.sim.simx_opmode_streaming);
            [~,ay] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/sensor/acceleration/y', obj.sim.simx_opmode_streaming);
            [~,az] = obj.sim.simxGetFloatSignal(obj.id, 'UAV/sensor/acceleration/z', obj.sim.simx_opmode_streaming);
            
            obj.quadcopter.a_hat = [ax ay az];
            
            % external forces applied to quadrotor
            [m,I,k_wr,w,a_hat,F_hat] = deal(obj.quadcopter.m,obj.quadcopter.I,obj.quadcopter.k_wr,obj.quadcopter.state(10:12),obj.quadcopter.a_hat,obj.quadcopter.F_hat);      
            [f_e, m_e] = Aerodynamics.ExternalWrenchEstimator(m,I,k_wr,u,w',a_hat',F_hat');
            obj.quadcopter.F_hat = obj.quadcopter.F_hat +  [f_e(t)' m_e(t)'];  
            
            % trigger simulation
            obj.sim.simxSynchronousTrigger(obj.id);
        end
    end
end