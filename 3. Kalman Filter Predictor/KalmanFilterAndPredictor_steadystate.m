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

kalmanFilterDataSteady = zeros(3, size(Y,1));

x_k_x = [Y(1);0;0];
[P_inf] = idare(A',C', Q, R, [],[]);

k_inf = P_inf * C'*pinv(C*P_inf*C'+R);

for i = 2:size(Y,1)
    X_Kp1_kp1 = A *x_k_x + k_inf*(Y(i) - C*A*x_k_x);

    kalmanFilterDataSteady(:,i) = X_Kp1_kp1;
    x_k_x = X_Kp1_kp1;
end


%% Kalman Predictor
kalmanPredDataSteady = zeros(3, size(Y,1));

x_km1_k = [Y(1);0;0];

k_inf = A*P_inf * C'*pinv(C*P_inf*C'+R);
for i = 2:size(Y,1)
    X_Kp1_k = A *x_km1_k + k_inf*(Y(i) - C*x_km1_k);

    x_km1_k = X_Kp1_k;
    kalmanPredDataSteady(:,i) = x_km1_k;
end

%% Graphs

time_signal = out.velocity.time;
velocity_true = out.velocity.signals.values;
acceleration_true = out.acceleration.signals.values;

figure(1);
plot(time_signal, velocity_true);
hold on;
plot(time_signal, kalmanFilterDataSteady(2,:));
hold on;
plot(time_signal, kalmanPredDataSteady(2,:));
legend('Velocity True', 'Kalman Filter', 'Kalman Predictor' );

figure(2);
plot(time_signal, acceleration_true);
hold on;
plot(time_signal, kalmanFilterDataSteady(3,:));
hold on;
plot(time_signal, kalmanPredDataSteady(3,:));
legend('Acceleration True', 'Kalman Filter', 'Kalman Predictor' );

%% Mean Squared Error

disp("RMSE Filter: ");
E = RMSE(velocity_true, kalmanFilterDataSteady(2,:)');
disp(E);

disp("RMSE Predictor: ");
E = RMSE(velocity_true, kalmanPredDataSteady(2,:)');
disp(E);


disp("RMSE Filter: ");
E = RMSE(acceleration_true, kalmanFilterDataSteady(3,:)');
disp(E);


disp("RMSE Predictor: ");
E = RMSE(acceleration_true, kalmanPredDataSteady(3,:)');
disp(E);


function E = RMSE(Y, Y_hat)
  square_sum = 0;

  for i = 1: size(Y,1)
      square_sum = square_sum + ((Y(i) - Y_hat(i))^2);
  end
  E = sqrt(square_sum/size(Y,1));
end
