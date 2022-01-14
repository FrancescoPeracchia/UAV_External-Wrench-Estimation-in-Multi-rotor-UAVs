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
        I       % 3x1 inertia vector [Ix Iy Iz]
        Ir     % blades inertia
        Om_r    % avg blades rotational velocity
        
        % gain values (1x2 vector [kp kd])
        k_x     % x 
        k_y     % y 
        k_z     % z 
        k_phi   % roll 
        k_theta % pitch
        k_psi   % yaw
        
        % state of dynamic model (1x12 vector)
        state % [x y z Vx Vy Vz phi theta psi p q r]
        
    end
    
    methods
        function obj = Quadcopter(params,gains)
            % QUADCOPTER constructor
            obj.g = params(1);
            obj.m = params(2);
            obj.l = params(3);
            obj.I = [params(4) params(5) params(6)];
            obj.Ir = params(7);
            obj.Om_r = params(8);
            obj.k_x = [gains(1),gains(2)];
            obj.k_y = [gains(3),gains(4)];
            obj.k_z = [gains(5),gains(6)];
            obj.k_phi = [gains(7),gains(8)];
            obj.k_theta = [gains(9),gains(10)];
            obj.k_psi = [gains(11),gains(12)];
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
            [ax_d,ay_d,~] = deal(acc(1),acc(2),acc(3));
            
            % state
            [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r] = deal(...
                obj.state(1),obj.state(2),obj.state(3),obj.state(4),...
                obj.state(5),obj.state(6),obj.state(7),obj.state(8),...
                obj.state(9),obj.state(10),obj.state(11),obj.state(12));
            
            % inertia
            [Ix,Iy,Iz] = deal(obj.I(1),obj.I(2),obj.I(3));
            
            % regulation
            [kx_p,kx_d] = deal(obj.k_x(1),obj.k_x(2));
            [ky_p,ky_d] = deal(obj.k_y(1),obj.k_y(2));
            [kz_p,kz_d] = deal(obj.k_z(1),obj.k_z(2));
            [kphi_p,kphi_d] = deal(obj.k_phi(1),obj.k_phi(2));
            [ktheta_p,ktheta_d] = deal(obj.k_theta(1),obj.k_theta(2));
            [kpsi_p,kpsi_d] = deal(obj.k_psi(1),obj.k_psi(2));
            
            % height control
            stab_z = (obj.g+kz_p*(z_d-z)+kz_d*(vz_d-vz));
            fl_z = obj.m/(cos(phi)*cos(theta));
            T = fl_z*stab_z;
            
            % desired thrust direction
            Tx = (obj.m/T)*(kx_p*(x_d-x)+kx_d*(vx_d-vx)+ax_d);
            Ty = (obj.m/T)*(ky_p*(y_d-y)+ky_d*(vy_d-vy)+ay_d);

            % desired attitude from desired thrust direction
            phi_d = asin(Tx*sin(psi)-Ty*cos(psi));
            theta_d = asin((Tx*cos(psi)+Ty*sin(psi))/cos(phi_d));
            
            % control torques
            tau_phi = Ix*(kphi_p*(phi_d-phi)+kphi_d*(-p));
            tau_theta = Iy*(ktheta_p*(theta_d-theta)+ktheta_d*(-q));
            tau_psi = Iz*(kpsi_p*(psi_d-psi)+kpsi_d*(-r));

            u = [T,tau_phi,tau_theta,tau_psi];
        end
        
        function state = command(obj,u,t)
            % COMMAND send trust commands to the propellers
            %   input 
            %    - u:       trust and torques commands [T, tau]
            %    - t:       time
            %   output
            %    - state:   computes [x y z vx vy vz phi theta psi p q r]
            
            % retrieve values
            [T, tau_phi, tau_theta, tau_psi] = deal(u(1),u(2),u(3),u(4));
            [Ix,Iy,Iz] = deal(obj.I(1),obj.I(2),obj.I(3));
            
            [x,y,z,vx,vy,vz,phi,theta,psi,p,q,r] = deal(...
                obj.state(1),obj.state(2),obj.state(3),obj.state(4),...
                obj.state(5),obj.state(6),obj.state(7),obj.state(8),...
                obj.state(9),obj.state(10),obj.state(11),obj.state(12));
            
            % compute dxi/dt
            ax = (T/obj.m)*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
            ay = (T/obj.m)*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
            az = (-obj.g + (T/obj.m)*(cos(phi)*cos(theta)));

            pdot =  ((Iy-Iz)/Ix)*q*r + (obj.Ir/Ix)*q*obj.Om_r +(obj.l/Ix)*tau_phi;
            qdot = ((Iz-Ix)/Iy)*p*r - (obj.Ir/Iy)*p*obj.Om_r + (obj.l/Iy)*tau_theta;
            rdot =  ((Ix-Iy)/Iz)*p*q + (1/Iz)*tau_psi;

            % integrate TODO ?????
            state = [x+vx*t,y+vy*t,z+vz*t,vx+ax*t,vy+ay*t,vz+az*t,phi+p*t,theta+q*t,psi+r*t,p+pdot*t,q+qdot*t,r+rdot*t];
            obj.state = state;
        end
    end
end

