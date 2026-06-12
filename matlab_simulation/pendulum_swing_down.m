% NOTE: need to go into simulink model and set the initial arm angle AND
% the input to the motor

load_parameters();
J = 0;

out = sim("pendulum_model.slx");

plot(out.tout, out.pendulum_angle.Data);
ylabel("Angle (rad)")
xlabel("Time (s)")
title("No actuation pendulum angle")