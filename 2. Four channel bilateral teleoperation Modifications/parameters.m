% Sample parameters for four channel bilateral teleoperation
clear all;
close all;

% Input function parameter (sin or step with low pass filter)
A = 200;

% Low pass frequency cuff off
Flp = 0.5;
% Sin frequency
Fc = 1; 

% Human intention controller (PD)
Ph = 10*1; 
Dh = 20*0.8; 

% Human impedance parameters
Jh = 1;
Bh = 1; 

% Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 2.3;
Ds = 1.2;

% Master controller
Bm = 20*0.8;
Km = 10*1;

% Slave controller
Bs = 4*Bm; 
Ks = 4*Km; 

% Environment impedance parameters
Be = 100; 
Ke = 200; 
xe = 300;
