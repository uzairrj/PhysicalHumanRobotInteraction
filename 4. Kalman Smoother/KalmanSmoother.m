Y = out.position_GaussianNoise.signals.values;

Ts = out.position_GaussianNoise.time(2) - out.position_GaussianNoise.time(1);

A = [[1, Ts, (Ts^2)/2];
     [0, 1,   Ts     ];
     [0, 0,   1      ]];
B = [(Ts^3)/6; (Ts^2)/2; Ts];
C = [1,0,0];


R = 0.001;
q = 100000;
Q = (B*B')*q;


%% Kalman Filter (Forward)
x_k_x = [Y(1);0;0];
P_k_km1 = 0.0001*eye(3);

x_k_k = zeros(3,size(Y,1));
p_k_k = zeros(3,3,size(Y,1));

X_Kp1_k = zeros(3,size(Y,1));
p_kp1_k = zeros(3,3,size(Y,1));

X_Kp1_k(:, size(Y,1)) = x_k_x;
x_k_k(:, size(Y,1)) = x_k_x;

p_kp1_k(:,:, size(Y,1)) = P_k_km1;

p_k_k(:,:, size(Y,1)) = P_k_km1;

for i = 2:size(Y,1)
 X_Kp1_k(:,i) = A * x_k_x;
 p_kp1_k(:,:,i) = A*P_k_km1*A'+Q;

 K = p_kp1_k(:,:,i)*C'*pinv(C * p_kp1_k(:,:,i) * C' + R);

 x_kp1_kp1 = X_Kp1_k(:,i) + K * (Y(i) - C*X_Kp1_k(:,i));
 x_k_x = x_kp1_kp1;

 p_kp1_kp1 =  p_kp1_k(:,:,i) - p_kp1_k(:,:,i) * C' * pinv(C * p_kp1_k(:,:,i) * C' + R) * C * p_kp1_k(:,:,i);
 P_k_km1 = p_kp1_kp1;

 x_k_k(:,i) = x_kp1_kp1;
 p_k_k(:,:,i) = p_kp1_kp1;
end

x_smooth = zeros(3,size(Y,1));
x_smooth(:,size(Y,1)) = x_k_x; %last computed x value


%% Kalman Smoother (Backword)
for i=size(Y,1)-1:-1:1
    K = p_k_k(:,:,i) *A'*pinv(p_kp1_k(:,:,i+1));
    x_smooth(:,i) = x_k_k(:,i) + K*(x_smooth(:,i+1) - X_Kp1_k(:,i+1));
end


%% Graphs

time_signal = out.velocity.time;
velocity_true = out.velocity.signals.values;
acceleration_true = out.acceleration.signals.values;

figure(1);
hold on;
plot(time_signal, velocity_true);
hold on;
plot(time_signal, x_smooth(2,:));
hold on;
plot(time_signal, x_k_k(2,:));
hold on;
legend('Velocity True', 'Kalman Filter (Smooth)', 'Kalman Filter' );
