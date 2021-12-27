classdef SimQuadcopter
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
            % QUADCOPTER Construct an instance of this class
            obj.sim = sim;
            obj.id = id;
            obj.step = step;
            obj.quadcopter = Quadcopter();
        end
        
        function u = control(q_des)
            % CONTROL Summary of this method goes here
            %   Detailed explanation goes here
            u = 0;
        end
        
        function state = command(u)
            % COMMAND Summary of this method goes here
            %   Detailed explanation goes here
            state = [];
        end
    end
end

