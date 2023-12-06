clear all
% Define the state-space system matrices (A, B, C, D)
A = [-0.0558,-0.9968,0.0802,0.0415;0.5980,-0.1150,-0.0318,0;-3.0500,0.3880,-0.4650,0;0,0.0805,1,0 ];
B = [0.0729,0;-4.75,0.00775;0.15300,0.1430;0,0];
C = [1 0 0 0;0 0 0 1];
D = [0 0; 0 0];

% Open Loop System
sys = ss(A,B,C,D);

% Define the Q and R matrices for LQR controller desig
R = 1;
Q = 500*(C'*C);

% Design the LQR controller
K = lqr(A, B, Q, R);

% Calculate the constant gain Nbar to eliminate steady-state error
Nbar = -inv(C*inv(A-B*K)*B); %For unit reference input

% Create the closed-loop system with Nbar
A_cl = A - B * K;
B_cl = B*100;
C_cl = C;
D_cl = D;

% Define the state-space system for closed-loop control with Nbar
sys_cl = ss(A_cl, B_cl, C_cl, D_cl);
t_impulse = 0:0.01:20;

impulse_response_lqr1 = impulse(sys_cl(1,1), t_impulse);
impulse_response_openloop1 = impulse(sys(1,1), t_impulse);


% Plot the impulse response
figure(1);
plot(t_impulse, impulse_response_lqr1, 'b', 'LineWidth', 2);

hold on
plot(t_impulse, impulse_response_openloop1, 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Side Slip Angle');

legend({'lqr','open loop'},'Location','northeast')
grid on;


impulse_response_lqr2 = impulse(sys_cl(2,1), t_impulse);
impulse_response_openloop2 = impulse(sys(2,1), t_impulse);
figure(2);
plot(t_impulse, impulse_response_lqr2, 'b', 'LineWidth', 2);

hold on
plot(t_impulse, impulse_response_openloop2, 'r', 'LineWidth', 2);
xlabel('Time');
ylabel('Roll Angle');

legend({'lqr','open loop'},'Location','northeast')



