clc;
close all;
clear all;

mc = 0.987; %mass of cart [kg]
mp = 0.172; %mass of pendulum [m]
b= 0.1; %coefficient of friction for cart [N/m/sec]
L = 0.3; %length to pendulum center of mass [m]
g=9.8; %m/s^2
I = 0.063; %mass moment of inertia [kg.m^2]
%F = force applied to cart
%x = cart position coordinate
%d1 = 1e-2; %damping of cart  displacement
%d2 = 1e-2; %damping of pin joint

p = I*(mc+mp)+mc*mp*L^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+mp*L^2)*b/p  (mp^2*g*L^2)/p   0;
     0      0              0           1;
     0 -(mp*L*b)/p       mp*g*L*(mc+mp)/p  0];
B = [     0;
     (I+mp*L^2)/p;
          0;
        mp*L/p];
C = [1 0 0 0;
      0 0 1 0]; %[x, x_dot, theta, theta_dot] so we control x and theta as outputs
%C = [1 0 0 0]; %[x, x_dot, theta, theta_dot] x  as output
D = [0;
     0];
%D = [0];

%% Build System

%Name variables for clarity
states={'x','x_dot','theta','theta_dot'};
inputs={'F'};
% outputs={'x','theta'};
outputs={'x'};
sys=ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%one pole that is unstable
eig(A);
pole(sys);

rank(ctrb(sys)); %if 4 (full rank), then controllable
rank(obsv(sys)); %if 4 (full rnak), then observable

x0=[0; -5*pi/180; 0; 0];

%% Controller

des_pole = [-3;-3;-3;-3]*1; %desired pole placement
K=acker(A,B,des_pole);

%Q=20*eye(4);
Q=C'*C;
R=0.1;
K_lqr=lqr(A,B,Q,R);
%% Discrete Time
Ts=0.020; %sampling time, 100 ms
sys_disc=c2d(sys,Ts);
Ad=sys_disc.a;
Bd=sys_disc.b;
Cd=sys_disc.c;
Dd=sys_disc.d;

des_pole_d = [-3;-3;-3;-3]*0.5; %desired pole placement but this is not lqr
K_d=acker(Ad,Bd,des_pole_d);
K_d_lqr=dlqr(Ad,Bd,Q,R);

%% Observer
des_pole_d = [-3;-3;-3;-3]*0.1; %desired pole placement but this is not lqr
%Ob=acker(A',C',des_pole_d)
R=0.4;
%Ob=dlqr(Ad,Bd,Q,R)
%% Run Sim
simOut=sim("pendModelSimulink")
%% Find Settling  Time
xSimData=simOut.simout.Data(:,1);
xDotSimData=simOut.simout.Data(:,2);
thetaSimData=simOut.simout.Data(:,3);
thetaDotData=simOut.simout.Data(:,4);
timeSimData=simOut.simout.Time;
SimResponse=stepinfo(xSimData);
settlingTime=timeSimData(round(SimResponse.SettlingTime))