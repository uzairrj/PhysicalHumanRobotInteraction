% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Input function parameter (sin or step with low pass filter)
Amp = 15;

% Low pass frequency cuff off
Flp = 5;
% Sin frequency
Fc = 1; 

% Human intention controller (PD)
Ph = 5; 
Dh = 5; 

% Human impedance parameters
Jh = 0;
Bh = 1.5; 

% Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 1.3;
Ds = 1.2;

% Master controller
Bm = 100;
Km = 50;

% Slave controller
Bs = 100; 
Ks = 80; 

% Environment impedance parameters
Be = 10; 
Ke = 30; 
xe = 4;

% Transportation variables
b = 10;
delay = 1;
lambda = 50;

%aditional parameters
Ts = 0.001;