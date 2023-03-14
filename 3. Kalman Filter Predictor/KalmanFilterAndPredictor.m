Y = out.position_GaussianNoise.signals.values;

Ts = out.position_GaussianNoise.time(2) - out.position_GaussianNoise.time(1);

A = [[1, Ts, (Ts^2)/2];
     [0, 1,   Ts     ];
     [0, 0,   1      ]];
B = [(Ts^3)/6; (Ts^2)/2; Ts];
C = [1,0,0];


R = 0.001;
q = 1000;
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


%% Kalman Predictor
kalmanPredData = zeros(3, size(Y,1));

x_km1_k = [Y(1);0;0];
P_k_km1 = 0.0001*eye(3);

for i = 2:size(Y,1)
    P_kp1_k = A*P_k_km1*A' - A*P_k_km1*C'*pinv(C*P_k_km1*C'+R)*C*P_k_km1*A'+Q;

    k_k = A*P_k_km1*C'*pinv(C*P_k_km1*C'+R);

    X_Kp1_k = A *x_km1_k + k_k*(Y(i) - C*x_km1_k);

    x_km1_k = X_Kp1_k;
    kalmanPredData(:,i) = x_km1_k;
    P_k_km1 = P_kp1_k;

end

%% Graphs

time_signal = out.velocity.time;
velocity_true = out.velocity.signals.values;
acceleration_true = out.acceleration.signals.values;

figure(1);
plot(time_signal, velocity_true);
hold on;
plot(time_signal, kalmanFilterData(2,:));
hold on;
plot(time_signal, kalmanPredData(2,:));
legend('Velocity True', 'Kalman Filter', 'Kalman Predictor' );

figure(2);
plot(time_signal, acceleration_true);
hold on;
plot(time_signal, kalmanFilterData(3,:));
hold on;
plot(time_signal, kalmanPredData(3,:));
legend('Acceleration True', 'Kalman Filter', 'Kalman Predictor' );

%% Mean Squared Error

disp("RMSE Predictor: ");
E = RMSE(velocity_true, kalmanFilterData(2,:)');
disp(E);

function E = RMSE(Y, Y_hat)
  square_sum = 0;

  for i = 1: size(Y,1)
      square_sum = square_sum + ((Y(i) - Y_hat(i))^2);
  end
  E = sqrt(square_sum/size(Y,1));
end
