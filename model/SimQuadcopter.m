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
            % SIMQUADCOPTER constructor
            obj.sim = sim;
            obj.id = id;
            obj.step = step;
            obj.quadcopter = Quadcopter();
        end
        
        function obj = init(obj,state)
            % INIT inherits the setState function
            obj.quadcopter.setState(state);
        end
        
        function u = control(obj,pose)
            % CONTROL inherits the control function
            u = obj.quadcopter.control(pose);
        end
        
        function state = command(obj,u)
            % COMMAND inherits the command function
            state = obj.quadcopter.command(u);
        end
    end
end

