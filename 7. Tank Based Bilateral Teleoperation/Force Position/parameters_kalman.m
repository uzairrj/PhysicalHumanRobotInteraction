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
Dm = 5.3;
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
xe = 30;

% Transportation variables
b = 20;
delay = 10;
lambda = 50;

%aditional parameters
variance = 0.000001;
Ts = 0.001;

%kalman parameters
A = [[1, Ts, (Ts^2)/2];
     [0, 1,   Ts     ];
     [0, 0,   1      ]];
B = [(Ts^3)/6; (Ts^2)/2; Ts];
C = [1,0,0];


R = 0.0000001;
q = 10000000;
Q = (B*B')*q;
P = 0.00001;

paramsKalman.A = A;
paramsKalman.B = B;
paramsKalman.C = C;
paramsKalman.R = R;
paramsKalman.Q = Q;
paramsKalman.P = P;