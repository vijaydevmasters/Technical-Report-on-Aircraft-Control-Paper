clear all
% Define the system matrices
A = [-0.3149,235.8928,0; -0.0034,-0.4282,0; 0,1,0];
B = [-5.5079; 0.0021; 0];
C = [0,0,1];
D = 0;

% Define the disturbance matrices
% Assume a small positive value for process noise covariance
Qn = 1; % Modify as per your system's characteristics
Rn = 1; % Measurement noise covariance
Nn = 0; % Cross-correlation between process and measurement noise (assuming zero)

% Define the LQR weighting matrices
Qlqr = [0,0,0; 0,0,0; 0,0,500];
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
impulse_response_lqg = impulse(sys_cl, t_impulse);
impulse_response_openloop = impulse(sys, t_impulse);


% Plot the impulse response
figure;
plot(t_impulse, impulse_response_lqg, 'b', 'LineWidth', 2);

hold on
plot(t_impulse, impulse_response_openloop, 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Pitch Angle');

legend({'lqg','open loop'},'Location','northeast')
grid on;
