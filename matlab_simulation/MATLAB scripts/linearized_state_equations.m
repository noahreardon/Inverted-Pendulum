% Linearize the dynamic equations about the upright position
%   This means that:
%   theta1 = 0
%   theta2 = pi
%   theta1dot = 0
%   theta2dot = 0
% We want an equation of the form
%   xdot = Ax + Bu,
% where x is the current state and u is the control input

% The state vector has the form:
%   [ theta1;
%     theta2;
%     theta1dot;
%     theta2dot ]

% Around the upright, sin(alpha) ~ alpha, cos(alpha) ~ 1

% define necessary constants here:
g = 9.81; % m/s^s
L2 = 0.07; % m
l2 = L2/2; % center of mass in the middle of the pendulum
m2 = 5e-3; % kg, 5g is the weight of the pendulum
L1 = 0.03; % m (length of arm)
l1 = L1/2; % THIS IS AN APPROXIMATION
m1 = 1e-3; % mass of the arm is 1g
J1 = 1/3 * m1 * L1^2;
J2 = 1/3 * m2 * L2^2;
Jhat0 = J1 + m1*l1^2 + m2*L1^2;
Jhat2 = J2 + m2*l2^2;
b1 = 1e-5; % THIS IS AN APPROXIMATION
b2 = 1e-5; % THIS IS AN APPROXIMATION

% Define the A matrix (from Cazzolato and Prime paper):

A32 = (g * m2^2 * l2^2 * L1) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
A33 = (-b1 * Jhat2) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
A34 = (-b2 * m2 * l2 * L1) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
A42 = (g * m2 * l2 * Jhat0) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
A43 = (-b1 * m2 * l2 * L1) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
A44 = (-b2 * Jhat0) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);

A = [0, 0, 1, 0;
       0, 0, 0, 1;
       0, A32, A33, A34;
       0, A42, A43, A44];

% Define the B matrix

B31 = Jhat2 / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
B41 = (m2 * L1 * l2) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
B32 = (m2 * L2 * l2) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);
B42 = (Jhat0) / (Jhat0 * Jhat2 - m2^2 * L1^2 * l2^2);

% We only care about the torque from the motor so only use one column in
% the B matrix
B = [0;
       0;
       B31;
       B41];

% Now we need to define a Q matrix and an R matrix. The Q matrix defines
% how much a change in each of the state variables "costs" us, and the R
% matrix defines how much each control input "costs" us. In this case,
% there are four states and one control input, so Q will be 4x4 and R will
% be 1x1.

Q_vals = [1, 100, 1, 10];
Q = diag(Q_vals);

R = 0.1; % electricity is cheap

K = lqr(A, B, Q, R);


%% Run the simulation with the control law

load_parameters();


out = sim("C:\Users\REARDONNM24\OneDrive - Grove City College\Projects\Inverted-Pendulum\matlab_simulation\Simulink models\Pendulum models\pendulum_control.slx");

subplot(5, 1, 1)
plot(out.tout, out.motor_torque.Data)
xlabel("Time (s)")
ylabel("(Nm)")
title("Applied torque")

subplot(5, 1, 2)
plot(out.tout, out.arm_angle.Data)
xlabel("Time (s)")
ylabel("(rad)")
title("Arm angle")

subplot(5, 1, 3)
plot(out.tout, out.arm_velocity.Data)
xlabel("Time (s)")
ylabel("(rad/s)")
title("Arm velocity")

subplot(5, 1, 4)
plot(out.tout, out.pendulum_angle.Data)
hold on
plot(out.tout, repmat(pi, size(out.tout)))
hold off
xlabel("Time (s)")
ylabel("(rad)")
title("Pendulum angle")

subplot(5, 1, 5)
plot(out.tout, out.pendulum_velocity.Data)
xlabel("Time (s)")
ylabel("(rad/s)")
title("Pendulum velocity")








