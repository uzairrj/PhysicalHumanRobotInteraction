Y = out.position_GaussianNoise.signals.values;

Ts = out.position_GaussianNoise.time(2) - out.position_GaussianNoise.time(1);

A = [[1, Ts, (Ts^2)/2];
     [0, 1,   Ts     ];
     [0, 0,   1      ]];
B = [(Ts^3)/6; (Ts^2)/2; Ts];
C = [1,0,0];


R = 0.000001;
q = 10;
Q = q*(B*B');


%% Kalman Filter

kalmanFilterData = zeros(3, size(Y,1));

x_k_x = [Y(1);0;0];
P_k_km1 = 0.0001*eye(3);

for i = 2:size(Y,1)
    P_kp1_k = A*P_k_km1*A' - A*P_k_km1*C'*pinv(C*P_k_km1*C'+R)*C*P_k_km1*A'+Q;
    k_kp1 = P_kp1_k*C'*pinv(C*P_kp1_k*C'+R);
    X_Kp1_kp1 = A *x_k_x + k_kp1*(Y(i) - C*A*x_k_x);

    kalmanFilterData(:,i) = X_Kp1_kp1;
    x_k_x = X_Kp1_kp1;
    P_k_km1 = P_kp1_k;

end

