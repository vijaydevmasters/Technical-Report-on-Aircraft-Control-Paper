clear all
% Define the system matrices
A = [-0.0558,-0.9968,0.0802,0.0415;0.5980,-0.1150,-0.0318,0;-3.0500,0.3880,-0.4650,0;0,0.0805,1,0 ];
B = [0.0729,0;-4.75,0.00775;0.15300,0.1430;0,0];
C = [1 0 0 0;0 0 0 1];
D = [0 0; 0 0];


% Define the disturbance matrices
% Assume a small positive value for process noise covariance
Qn = 1; % Modify as per your system's characteristics
Rn = 1; % Measurement noise covariance
Nn = 0; % Cross-correlation between process and measurement noise (assuming zero)

% Define the LQR weighting matrices
Qlqr = [0,0,0,0; 0,0,0,0;0,0,0,0; 0,0,0,500];
Rlqr = 1;

% Compute the LQR gain
K = lqr(A, B, Qlqr, Rlqr);

% Create state-space model Open Loop
sys = ss(A, B, C, D);

% Design the Kalman filter
[kest,L,P] = kalman(sys, Qn, Rn, Nn);

% Form the LQG regulator
Acl = [A-B*K B*K; zeros(size(A)) A-L*C];
Bcl = [B; zeros(size(B))];
Ccl = [C zeros(size(C))];
Dcl = D;

% Create state-space model Closed Loop
sys_cl = ss(Acl, Bcl, Ccl, Dcl);

% Simulate the impulse response
t_impulse = 0:0.01:20;  % Time vector for impulse response

impulse_response_lqg1 = impulse(sys_cl(1,1), t_impulse);
impulse_response_openloop1 = impulse(sys(1,1), t_impulse);


% Plot the impulse response
figure(1);
plot(t_impulse, impulse_response_lqg1, 'b', 'LineWidth', 2);

hold on
plot(t_impulse, impulse_response_openloop1, 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Side Slip Angle');

legend({'lqg','open loop'},'Location','northeast')
grid on;


impulse_response_lqg2 = impulse(sys_cl(2,1), t_impulse);
impulse_response_openloop2 = impulse(sys(2,1), t_impulse);

figure(2);
plot(t_impulse, impulse_response_lqg2, 'b', 'LineWidth', 2);

hold on
plot(t_impulse, impulse_response_openloop2, 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Roll Angle');

legend({'lqg','open loop'},'Location','northeast')