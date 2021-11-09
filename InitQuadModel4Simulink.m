%Quadrotor model
%
clear all;close all;
clc;

% 
% 
% %% Parameters
% 
% %vehicle
g = 9.81;   % [ms^-2]
Ix = 0.5;   % [kg*m^2]
Iy = 0.5;
Iz = 0.9;
Ir = 0.005;
Om_r = 70; % [s^-2]
m = 2;      % [kg]
l = 0.25;    % [m]
d = 0.03;
m1=0.1;
% 

 % %% Initial state
x0 = 0;
y0 = 0;
z0 = 0;
phi0 = 0;
theta0 = 0;
psi0 = 0;
vx0 = 0;
vy0 = 0;
vz0 = 0;
p0 = 0;
q0 = 0;
r0 = 0;

initialCondition = [x0 vx0 y0 vy0 z0 vz0 phi0 p0 theta0 q0 psi0 r0]';

% %desired position
x_d=0;
y_d=0;
z_d =3;

% hovering task
phi_d = 0;
theta_d =0;
%  regulation value depends on the task
psi_d =0;

% %model parameters
 par=[m;m1;Ix;Iy;Iz;Ir;Om_r;d;l;g];
% %controller gains
   
% %regulation values
 regval=[x_d;y_d;z_d;phi_d;theta_d;psi_d];
 
 
 
 % %control
%%% Students control parameters
%lunch the script used to initialize the parameters gains for the PID0
kx = 0.1;
kx_d = 0.54;
kx_i = 0.0;

ky = 0.1;
ky_d = 0.5;
ky_i=5;

kz = 2;
kz_d = 3;
kz_i=1;

k_phi = 20;
kphi_d = 25;
kphi_i=0;

k_theta = 10;
ktheta_d = 15;
ktheta_i=0.01;

k_psi = 10;
kpsi_d = 10;
kpsi_i=0.1;

% 
gain=[kx;kx_d;kx_i;...
       ky;ky_d;ky_i;...
       kz;kz_d;kz_i;...
       k_phi;kphi_d;kphi_i;...
       k_theta;ktheta_d;ktheta_i;...
       k_psi;kpsi_d;kpsi_i];


