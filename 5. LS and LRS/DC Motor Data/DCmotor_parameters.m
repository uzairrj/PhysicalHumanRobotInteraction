% --- DC MOTOR MAXON + GEARBOX---
Ts = 0.001;

%% Encoder
eQ_s = 2*pi / 4096;

%% Maxon EC 90 flat ∅90 mm, brushless, 90 Watt 323772
LM = 0.000264; % [H]
RM = 0.343; % [Ohm]
KmM = 70.5*0.001; % [Nm / A]
KeM = 1 / ( 135 * (2*pi) / 60 ); % [V / (rad/s)] 135 [rpm/V] %% 1 rpm = 2*pi / 60 rad/s
JM = 20 * 3060/(1000*100^2); % [kg m^2] 3060 [gcm^2] /* x20 */
tau_mechM = 21.1*0.001; % [s] tm = R*J / Km^2 (datasheet maxon)
BM = 0.01; % 

%% Planetary Gearhead GP 52 C ∅52 mm, 4.0–30.0 Nm 223083
N = 12;
Jgb = 17.6/(1000*100^2); % [kg m^2] % 17.6 [gcm^2]



% %% fdt
% s = tf('s');
% G_V2vel = (1/N) * KmM/((LM*s+RM)*((JM+Jgb)*s+BM)+KeM*KmM );
% DCgain_V2vel = dcgain(G_V2vel);
% G_V2pos = G_V2vel/s;


%% Proportional controller
Kp = 5;

