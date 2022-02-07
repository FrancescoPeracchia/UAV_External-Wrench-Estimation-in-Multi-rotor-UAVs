classdef contact_detection
    %AERODYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    methods(Static)
        function [residual_tau] = contact_det(tau_hat,tau_input)
            % Contact detection 
            %   input 
            %    - tau_input: total wrench
            %    - tau_hat: estimated aerodinamic wrench
            %   output
            %    - residual_tau : aerodinatic residual wrech
    
 

         

            residual_tau=zeros(1:3);
            residual_tau(1)=tau_input(1)-tau_hat(1);
            residual_tau(2)=tau_input(2)-tau_hat(2);
            residual_tau(3)=tau_input(3)-tau_hat(3);


           
 


            

           
        end
    end
end