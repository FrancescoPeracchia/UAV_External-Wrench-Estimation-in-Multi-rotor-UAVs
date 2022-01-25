classdef Aerodynamics
    %AERODYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    methods(Static)
        function [F_hat,tau_hat] = ExternalWrenchEstimator(m,I,k,T,w,a,f,t)
            % EXTERNALWRENCHESTIMATOR estimates the external wrench
            %   input 
            %    - m: object mass
            %    - I: inertia matrix
            %    - k: force and torque gains
            %    - T: control trust
            %    - w: angular velocity of the body
            %    - a: acceleration from accelerometer sensor
            %    - f: current wrench
            %    - t: time step
            %   output
            %    - F_hat:   estimated external force 
            %    - tau_hat: estimated external torque

            [kf_i,kt_i] = deal(k(1),k(2));
            [F_e, tau_e] = deal(f(1:3),f(4:6));
            F = [0 0 T]';
            
            F_hat = kf_i*(m*a - F - F_e)*t;
            tau_hat = kt_i*(I*w -(m + cross(I*w,w) - tau_e)*t);
        end
    end
end

