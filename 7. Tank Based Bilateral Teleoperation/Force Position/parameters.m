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
Ph = 10*1; 
Dh = 20*0.8; 

% Human impedance parameters
Jh = 1;
Bh = 1; 

% Inertia/Damping of robot dynamics
Mm = 0.5;
Ms = 2;
Dm = 1.3;
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
xe = 120;

% Transportation variables
delay = 10;

%Tank parameters
alpha = 55;
beta = 0.001;
Hd = 550;

Hm_init = 0;
Hs_init = 0;

%aditional parameters
Ts = 0.001;