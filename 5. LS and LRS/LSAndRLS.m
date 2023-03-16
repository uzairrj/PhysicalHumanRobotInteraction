Y = out.positions.Data;

Ts = out.positions.Time(2) - out.positions.Time(1);

A = [[1, Ts, (Ts^2)/2];
     [0, 1,   Ts     ];
     [0, 0,   1      ]];
B = [(Ts^3)/6; (Ts^2)/2; Ts];
C = [1,0,0];


R = 0.001;
q = 100000;
Q = (B*B')*q;

%% Kalman Smoother
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


for i=size(Y,1)-1:-1:1
    K = p_k_k(:,:,i) *A'*pinv(p_kp1_k(:,:,i+1));
    x_smooth(:,i) = x_k_k(:,i) + K*(x_smooth(:,i+1) - X_Kp1_k(:,i+1));
end

%% Comparizon Real and Predicted velocity and Acceleration

time_signal = out.velocity.Time;
velocity_true = out.velocity.Data;
acceleration_true = out.acceleration.Data;

figure(1);
hold on;
plot(time_signal, velocity_true);
hold on;
plot(time_signal, x_smooth(2,:));
legend('Velocity True', 'Kalman Filter (Smooth)');

figure(2);
plot(time_signal, acceleration_true);
hold on;
plot(time_signal, x_smooth(3,:));
legend('Acceleration True', 'Kalman Filter (Smooth)');

%% Mean Squared Error

disp("RMSE Kalman Filter(Smooth) Velocity: ");
E = RMSE(velocity_true, x_smooth(2,:)');
disp(E);

disp("RMSE Kalman Filter (Smooth) Acceleration: ");
E = RMSE(acceleration_true, x_smooth(3,:)');
disp(E);

%% Least Square Prediction
X = [x_smooth(2,:)', x_smooth(3,:)'];
Y = out.voltages.Data;

beta = pinv(X'*X)*X'*Y;

LS_pred = X*beta;

figure(3);
hold on;
plot(time_signal, Y);
hold on;
plot(time_signal, LS_pred);
legend('Voltage True', 'Voltage Pred');


disp("RMSE LS Voltage: ");
E = RMSE(velocity_true, LS_pred);
disp(E);


%% Recursive Least Square Prediction
X = [x_smooth(2,:)', x_smooth(3,:)'];
Y = out.voltages.Data;


p_pre = 0.0001*eye(2);
beta_pre = [0;0];

lambda = 0.95;

RLS_pred = zeros(1,size(Y,1));

for i = 2:size(Y,1)
    e = Y(i) - X(i,:)*beta_pre;
    P = 1/lambda*(p_pre - (p_pre*X(i,:)'*X(i,:)*p_pre)/(lambda+X(i,:)*p_pre*X(i,:)'));
    K = P*X(i,:)';
    beta = beta_pre +  K*e;

    RLS_pred(i) = X(i,:)*beta+e;
    beta_pre = beta;
    p_pre = P;
end

disp("RMSE LSR Voltage: ");
E = RMSE(Y, RLS_pred);
disp(E);


figure(4);
hold on;
plot(time_signal, Y);
hold on;
plot(time_signal, RLS_pred);
legend('Voltage True', 'Voltage Pred');

%% Function Defination
function E = RMSE(Y, Y_hat)
  square_sum = 0;

  for i = 1: size(Y,1)
      square_sum = square_sum + ((Y(i) - Y_hat(i))^2);
  end
  E = sqrt(square_sum/size(Y,1));
end
