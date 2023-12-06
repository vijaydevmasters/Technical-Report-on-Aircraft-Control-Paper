clear all
% Define the state-space system matrices (A, B, C, D)
A = [-0.3149,235.8928 0; -0.0034, -0.4282, 0; 0, 1, 0];
B = [-5.5079; 0.0021; 0];
C = [0 0 1];
D = 0;

% Open Loop System
sys = ss(A, B, C, D);

% Define the Q and R matrices for LQR controller design
R = 1;
Q = 500*(C'*C);           


% Design the LQR controller
K = lqr(A, B, Q, R);

% Calculate the constant gain Nbar to eliminate steady-state error
Nbar = -1 / (C * (A - B * K)^(-1) * B); %For unit reference input

% Create the closed-loop system with Nbar
A_cl = A - B * K;
B_cl = B*Nbar;
C_cl = C;
D_cl = D;

% Define the state-space system for closed-loop control with Nbar
sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

% Generate the impulse response of the closed-loop system
t_impulse = 0:0.01:20;  % Time vector for impulse response
impulse_response_lqr = impulse(sys_cl, t_impulse);
impulse_response_openloop = impulse(sys, t_impulse);


% Plot the impulse response
figure;
plot(t_impulse, impulse_response_lqr, 'b', 'LineWidth', 2);

hold on
plot(t_impulse, impulse_response_openloop, 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Pitch Angle');

legend({'lqr','open loop'},'Location','northeast')
grid on;

% Display the calculated Nbar
fprintf('Constant Gain Nbar: %f\n', Nbar);

