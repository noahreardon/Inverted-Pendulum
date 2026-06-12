load_parameters();

out = sim("motor_model.slx");

figure("Name", "Inverted pendulum response to 10V step input");

subplot(5, 1, 1)
plot(out.tout, out.torque1.Data)
ylim([0, 0.2])
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
xlabel("Time (s)")
ylabel("(rad)")
title("Pendulum angle")

subplot(5, 1, 5)
plot(out.tout, out.pendulum_velocity.Data)
xlabel("Time (s)")
ylabel("(rad/s)")
title("Pendulum velocity")


