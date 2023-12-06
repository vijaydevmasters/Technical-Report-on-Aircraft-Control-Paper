clear all
% State Space Matrices
A = [-0.0558,-0.9968,0.0802,0.0415;0.5980,-0.1150,-0.0318,0;-3.0500,0.3880,-0.4650,0;0,0.0805,1,0 ];
B = [0.0729,0;-4.75,0.00775;0.15300,0.1430;0,0];
C = [1 0 0 0;0 0 0 1];
D = [0 0; 0 0];

% State Space Equations
sys = ss(A,B,C,D);
% Time Interval
t_impulse = 0:0.01:20;  % Time vector for impulse response

% Impulse Response
impulse_response_openloop1 = impulse(sys(1,1), t_impulse);
impulse_response_openloop2 = impulse(sys(2,1), t_impulse);

figure(1);
plot(t_impulse, impulse_response_openloop1, 'b', 'LineWidth', 2);
xlabel('Time');
ylabel('Side Slip Angle');

figure(2);
plot(t_impulse, impulse_response_openloop2, 'b', 'LineWidth', 2);
xlabel('Time');
ylabel('Roll Angle');