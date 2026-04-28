R = 1; % Resistance (ohms)
L = 0.1; % Inductance (henries)
K = 0.01; % Motor constant
J = 0.05; % Moment of inertia (kg.m^2)
b = 0.1; % Damping coefficient
Ke = 0.01; % Back EMF constant
T_load = 0.1; % Load torque (N.m)
s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K^2)
A = [0 1 0; 0 -b/J K/J; 0 -Ke/L -R/L];
B = [0; 0; 1/L];
C = eye(3);
D = zeros(3,1);
motor_ss=ss(A,B,C,D)
 sys = ss(A,B,C,D);
t = 0:0.01:10; % Time vector
u = ones(size(t)); 
[y,t,x] = lsim(sys, u, t);
subplot(2,1,1);
plot(t, y(:,1), 'b', t, y(:,2), 'r', t, y(:,3), 'g');
xlabel('Time (s)');
ylabel('Motor Variables');
legend({'Position', 'Velocity', 'Current'});
title('Motor Drive Inverter System Response');
