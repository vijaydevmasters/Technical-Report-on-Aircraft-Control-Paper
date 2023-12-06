clear all
% State Space Matrices
A = [-0.3149,235.8928,0; -0.0034,-0.4282,0; 0,1,0];
B = [-5.5079; 0.0021; 0];
C = [0,0,1];
D = 0;
sys = ss(A,B,C,D);

% Time Interval
t_impulse = 0:0.01:20;  % Time vector for impulse response

% Impulse Response
impulse_response_openloop = impulse(sys, t_impulse);
plot(t_impulse, impulse_response_openloop, 'b', 'LineWidth', 2);
xlabel('Time');
ylabel('Pitch Angle');