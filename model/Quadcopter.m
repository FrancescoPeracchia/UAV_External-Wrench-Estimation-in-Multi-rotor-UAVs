classdef Quadcopter < handle
    % QUADCOPTER methods for control and command a quadrotor dymamic model
    %   the system is xi_dot = f(xi) + g(xi)*u where
    %   xi: the state [position, velocity, attitude, angular velocity]
    %   u: the control inputs is the trust and roll pitch yaw torques
    
    properties
        % model parameters
        g       % gravity acceleration
        m       % mass
        l       % distance from com
        I       % 3x3 inertia matrix
        Ir      % blades inertia
        Om_r    % avg blades rotational velocity
        
        % gain values (1x2 vector [kp kd])
        k_x     % x 
        k_y     % y 
        k_z     % z 
        k_phi   % roll 
        k_theta % pitch
        k_psi   % yaw
        k_wr    % wrench
        
        % save last model values
        state           % [x y z vx vy vz phi theta psi p q r] 1x12
        controls        % [T tau_phi tau_theta tau_psi] 1x4
        a_hat           % [ax ay az] 1x3 acceleration from accelerometer
        F_hat           % [f_e m_e] 1x6 estimated wrench
    end
    
    methods
        function obj = Quadcopter(params)            
            % QUADCOPTER constructor with optional value params
            if ~exist('params','var')
                params = obj.getDefaultParams();
            end
            gains = obj.getGains();
            
            obj.g = params(1);
            obj.m = params(2);
            obj.l = params(3);
            obj.I = [params(4) 0 0; 0 params(5) 0; 0 0 params(6)];
            obj.Ir = params(7);
            obj.Om_r = params(8);
            obj.k_x = [gains(1),gains(2)];
            obj.k_y = [gains(3),gains(4)];
            obj.k_z = [gains(5),gains(6)];
            obj.k_phi = [gains(7),gains(8)];
            obj.k_theta = [gains(9),gains(10)];
            obj.k_psi = [gains(11),gains(12)];
            obj.k_wr = [gains(13),gains(14)];
            obj.F_hat = zeros(1,6);
            obj.a_hat = zeros(1,3);
            obj.controls = zeros(1,4);
        end
        
        function u = control(obj,traj)
            % CONTROL PD controller of quadrotor 
            %   input 
            %    - pose: desired position and orientation
            %   output
            %   - u:   trust and torques commands [T, tau]
            
            % position and orientation
            [pose,att,vel,acc] = deal(traj(1,:),traj(2,:),traj(3,:),traj(4,:));
            [x_d,y_d,z_d] = deal(pose(1),pose(2),pose(3));
            [~,~,psi_d] = deal(att(1),att(2),att(3));
            [vx_d,vy_d,vz_d] = deal(vel(1),vel(2),vel(3));
            [ax_d,ay_d,az_d] = deal(acc(1),acc(2),acc(3));
            
            % state
            [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r] = deal(...
                obj.state(1),obj.state(2),obj.state(3),obj.state(4),...
                obj.state(5),obj.state(6),obj.state(7),obj.state(8),...
                obj.state(9),obj.state(10),obj.state(11),obj.state(12));
            
            % external forces applied to quadrotor
            [f_e, m_e] = deal(obj.F_hat(1:3),obj.F_hat(4:6));
            
            % inertia
            [Ix,Iy,Iz] = deal(obj.I(1,1),obj.I(2,2),obj.I(3,3));
            
            % regulation
            [kx_p,kx_d] = deal(obj.k_x(1),obj.k_x(2));
            [ky_p,ky_d] = deal(obj.k_y(1),obj.k_y(2));
            [kz_p,kz_d] = deal(obj.k_z(1),obj.k_z(2));
            [kphi_p,kphi_d] = deal(obj.k_phi(1),obj.k_phi(2));
            [ktheta_p,ktheta_d] = deal(obj.k_theta(1),obj.k_theta(2));
            [kpsi_p,kpsi_d] = deal(obj.k_psi(1),obj.k_psi(2));
            
            % desired thrust direction
            fx = obj.m*(kx_p*(x_d-x)+kx_d*(vx_d-vx)+ax_d) - f_e(1);
            fy = obj.m*(ky_p*(y_d-y)+ky_d*(vy_d-vy)+ay_d) - f_e(2);            
            fz = obj.m*(kz_p*(z_d-z)+kz_d*(vz_d-vz)+az_d) + obj.m*obj.g - f_e(3);

            % desired attitude from desired thrust direction
            phi_d = real(asin(fx/fz*sin(psi)-fy/fz*cos(psi))); 
            theta_d = real(asin(fx/fz*cos(psi)+fy/fz*sin(psi)/cos(phi_d)));
            
            % control torques
            T = norm([fx fy fz]);
            tau_phi = Ix*(kphi_p*(phi_d-phi)+kphi_d*(-p))-m_e(1);
            tau_theta = Iy*(ktheta_p*(theta_d-theta)+ktheta_d*(-q))-m_e(2);
            tau_psi = Iz*(kpsi_p*(psi_d-psi)+kpsi_d*(-r))-m_e(3);
            
            u = [T,tau_phi,tau_theta,tau_psi];
            obj.controls = u;
        end
        
        function state = command(obj,u,t)
            % COMMAND send trust commands to the propellers
            %   input 
            %    - u:       trust and torques commands [T, tau]
            %    - t:       time step
            %   output
            %    - state:   computes [x y z vx vy vz phi theta psi p q r]
            
            % retrieve values
            [T, tau_phi, tau_theta, tau_psi] = deal(u(1),u(2),u(3),u(4));
            [Ix,Iy,Iz] = deal(obj.I(1,1),obj.I(2,2),obj.I(3,3));
            
            [~,~,~,vx,vy,vz,phi,theta,psi,p,q,r] = deal(...
                obj.state(1),obj.state(2),obj.state(3),obj.state(4),...
                obj.state(5),obj.state(6),obj.state(7),obj.state(8),...
                obj.state(9),obj.state(10),obj.state(11),obj.state(12));
            
            % external forces applied to quadrotor
            fz = obj.F_hat(3);
        
            % compute dxi/dt
            ax = (T/obj.m)*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
            ay = (T/obj.m)*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
            az = fz - obj.g + (T/obj.m)*(cos(phi)*cos(theta));

            pdot = ((Iy-Iz)/Ix)*q*r + (obj.Ir/Ix)*q*obj.Om_r + (obj.l/Ix)*tau_phi;
            qdot = ((Iz-Ix)/Iy)*p*r - (obj.Ir/Iy)*p*obj.Om_r + (obj.l/Iy)*tau_theta;
            rdot = ((Ix-Iy)/Iz)*p*q + (1/Iz)*tau_psi;

            % integrate and update state
            state_dot = [vx,vy,vz,ax,ay,az,p,q,r,pdot,qdot,rdot];
            state = obj.state + state_dot*t;
            obj.state = state;
        end
    end
    
    methods(Static)
       
        function params = getDefaultParams()
            % GETPARAMS set quadrotor parameters
            g = 9.81;       % [ms^-2]
            l = 0.25;       % [m]
            m = 0.5;        % [kg]
            Ix = 0.0019;    % [kg*m^2]
            Iy = 0.0019;
            Iz = 0.0033;
            Ir = 0.005;
            Om_r = 70;      % [s^-2]
            
            params = [g;m;l;Ix;Iy;Iz;Ir;Om_r];
        end
        
        function gains = getGains()
            % SETGAINS set gains values for control
            kx_p = 2;
            kx_d = 4;

            ky_p = 2;
            ky_d = 4;

            kz_p = 4;
            kz_d = 8;

            kphi_p = 70;
            kphi_d = 60;

            ktheta_p = 60;
            ktheta_d = 50;

            kpsi_p = 5;
            kpsi_d = 10;
            
            kf_i = 2;
            km_i = 3;

            gains=[kx_p,kx_d,ky_p,ky_d,kz_p,kz_d,...
                kphi_p,kphi_d,ktheta_p,ktheta_d,kpsi_p,kpsi_d,...
                kf_i,km_i];
        end
    end
end

