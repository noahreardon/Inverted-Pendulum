load_parameters();

out = sim("pendulum_model.slx");

subplot(6, 1, 1)
plot(out.tout, out.torque1.Data)
ylim([0, 0.2])
xlabel("Time (s)")
ylabel("(Nm)")
title("Applied torque")

subplot(6, 1, 2)
plot(out.tout, out.arm_angle.Data)
xlabel("Time (s)")
ylabel("(rad)")
title("Arm angle")

subplot(6, 1, 3)
plot(out.tout, out.arm_velocity.Data)
xlabel("Time (s)")
ylabel("(rad/s)")
title("Arm velocity")

subplot(6, 1, 4)
plot(out.tout, out.pendulum_angle.Data)
xlabel("Time (s)")
ylabel("(rad)")
title("Pendulum angle")

subplot(6, 1, 5)
plot(out.tout, out.pendulum_velocity.Data)
xlabel("Time (s)")
ylabel("(rad/s)")
title("Pendulum velocity")

subplot(6, 1, 6)
plot(out.tout, out.input_signal.data)
title("Input signal")
xlabel("Time (s)")
ylabel("Volts (V)")
