clear all
% Define the system matrices
A = [-0.3149,235.8928,0; -0.0034,-0.4282,0; 0,1,0];
B = [-5.5079; 0.0021; 0];
C = [0,0,1];
D = 0;

% Sample Time = -1 to mark discrete time
Ts = -1; 
% Discrete Plant Model
sys = ss(A,[B B],C,D,Ts,'InputName',{'u' 'w'},'OutputName','y');  % Plant dynamics and additive input noise w

% noise covariance Q and the sensor noise covariance R are values greater than zero
Q = 2.3; 
R = 1;

% Design the Kalman Filter
[kalmf,L,~,Mx,Z] = kalman(sys,Q,R);
%  discard the state estimates and keep only the first output,y_hat
kalmf = kalmf(1,:);


sys.InputName = {'u','w'};
sys.OutputName = {'yt'};

% sumblk to create an input for the measurement noise v
vIn = sumblk('y=yt+v');

kalmf.InputName = {'u','y'};
kalmf.OutputName = 'ye';

% Using connect to join sys and the Kalman filter together such that u is a shared input and the noisy plant output y feeds into the other filter input
SimModel = connect(sys,vIn,kalmf,{'u','w','v'},{'yt','ye'});

t = (0:100)';
% Sinusoidal input Vector
u = sin(t/5);

rng(10,'twister');
w = sqrt(Q)*randn(length(t),1);
v = sqrt(R)*randn(length(t),1);

% Simulate the response
out = lsim(SimModel,[u,w,v]);

yt = out(:,1);   % true response
ye = out(:,2);  % filtered response
y = yt + v;     % measured response

% Comparing the true response with the filtered response
clf
subplot(211), plot(t,yt,'b',t,ye,'g'), 
xlabel('Number of Samples'), ylabel('Output')
title('Kalman Filter Response')
legend('True','Filtered')
subplot(212), plot(t,yt-y,'b',t,yt-ye,'g'),
xlabel('Number of Samples'), ylabel('Error')
legend('True - measured','True - filtered')