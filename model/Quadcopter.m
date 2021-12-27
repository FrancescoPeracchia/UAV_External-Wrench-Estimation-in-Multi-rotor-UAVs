classdef Quadcopter
    %QUADCOPTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = Quadcopter()
            % QUADCOPTER Construct an instance of this class
            
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

