classdef SimQuadcopter < handle
    % SIMQUADCOPTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sim
        id
        step
        quadcopter
    end
    
    methods
        
        function obj = SimQuadcopter(sim,id,step)
            % SIMQUADCOPTER constructor
            obj.sim = sim;
            obj.id = id;
            obj.step = step;
            params = obj.getParams();
            gains = obj.setGains();
            obj.quadcopter = Quadcopter(params,gains);
        end
        
        function u = control(obj,traj)
            % CONTROL inherits the control function
            u = obj.quadcopter.control(traj);
        end
        
        function state = command(obj,u,t)
            % COMMAND inherits the command function
            state = obj.quadcopter.command(u,t);
        end
        
        function obj = init(obj,state)
            % INIT inherits the setState function
            obj.quadcopter.state = state;
        end
    end
    
    methods(Static)
        
        function params = getParams()
            % GETPARAMS gets quadrotor parameters from coppelia
            g = 9.81;   % [ms^-2]
            l = 0.25;    % [m]
            m = 2;      % [kg]
            Ix = 0.5;   % [kg*m^2]
            Iy = 0.5;
            Iz = 0.9;
            Ir = 0.005;
            Om_r = 70; % [s^-2]
            
            params = [g;m;l;Ix;Iy;Iz;Ir;Om_r];
        end
        
        function gains = setGains()
            % SETGAINS set gains values for control
            kx_p = 0.1;
            kx_d = 0.54;

            ky_p = 0.1;
            ky_d = 0.5;

            kz_p = 2;
            kz_d = 3;

            kphi_p = 20;
            kphi_d = 25;

            ktheta_p = 10;
            ktheta_d = 15;

            kpsi_p = 10;
            kpsi_d = 10;

            gains=[kx_p,kx_d,ky_p,ky_d,kz_p,kz_d,...
                kphi_p,kphi_d,ktheta_p,ktheta_d,kpsi_p,kpsi_d];
        end
    end
end

