classdef Aerodynamics
    %AERODYNAMICS Static methods based on aerodynamics 
    %   ExternalWrenchEstimator
    %   ContactDetection
    
    methods(Static)
        function [f_hat,m_hat] = ExternalWrenchEstimator(mass,I,k,u,w,a,F_e,t)
            % EXTERNALWRENCHESTIMATOR estimates the external wrench
            %   input 
            %    - mass: object mass
            %    - I: inertia matrix
            %    - k: force and torque gains
            %    - u: controls
            %    - w: angular velocity of the body
            %    - a: acceleration from accelerometer sensor
            %    - t: dt
            %   output
            %    - f_hat:   estimated external force
            %    - tau_hat: estimated external torque       
            
            persistent f_dt m_dt
            
            if isempty(f_dt) && isempty(m_dt)
                f_dt = 0;
                m_dt = 0;
            end

            [kf_i,km_i] = deal(k(1),k(2));
            [f,m] = deal([0 0 u(1)]',u(2:4)'); 
            [f_e,m_e] = deal(F_e(1:3),F_e(4:6));
            
            f_dt = f_dt - kf_i*(mass*a + f + f_e)*t;
            m_dt = m_dt + (cross(I*w,w) + m + m_e)*t;
            
            f_hat = f_dt;
            m_hat = km_i*(I*w - m_dt);             
        end
        
        function [residual_tau, detected] = ContactDetection(m_hat,tau)
            % Contact detection 
            %   input 
            %    - tau_hat: estimated aerodinamic torque
            %    - tau: control torques
            %   output
            %    - residual_tau : aerodinatic residual torque
            
            threshold = 0.5;
            detected = false;
            
            residual_tau = tau - m_hat;
            
            if norm(residual_tau) > threshold
                detected = true;
            end
        end
    end
end


