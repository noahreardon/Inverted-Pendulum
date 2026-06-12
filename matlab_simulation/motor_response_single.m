load_parameters();

Kp = 8;
out = sim("motor_position_control.slx");
figure;
subplot(2, 1, 1)
plot(out.tout, out.motor_position.Data, 'DisplayName', "Motor position")

xlabel("Time (s)")
ylabel("Position (rad)")
grid on
xlim([0 2])
subplot(2, 1, 2)
plot(out.tout, out.motor_current.Data, 'DisplayName', "Motor current")
xlabel("Time (s)")
ylabel("Current (A)")
grid on
xlim([0 2])

sgtitle("DC motor response to position feedback control")
