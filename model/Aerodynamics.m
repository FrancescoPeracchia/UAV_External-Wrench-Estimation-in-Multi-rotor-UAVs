classdef Aerodynamics
    %AERODYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    methods(Static)
        function [f_hat,m_hat] = ExternalWrenchEstimator(mass,I,k,u,w,a,f)
            % EXTERNALWRENCHESTIMATOR estimates the external wrench
            %   input 
            %    - m: object mass
            %    - I: inertia matrix
            %    - k: force and torque gains
            %    - u: controls
            %    - w: angular velocity of the body
            %    - a: acceleration from accelerometer sensor
            %    - f: current wrench
            %   output
            %    - F_hat:   estimated external force 
            %    - tau_hat: estimated external torque

            [kf_i,km_i] = deal(k(1),k(2));
            [f_e, m_e] = deal(f(1:3),f(4:6));
            
            f = [0 0 u(1)]';
            m = u(2:4)';         
            
            f_hat = @(t) kf_i*(-mass*a - f  - f_e)*t;
            m_hat = @(t) km_i*(I*w - (m + cross(I*w,w) - m_e)*t);
        end
    end
end

