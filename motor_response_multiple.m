load_parameters();

Kp_vector = 1:5:12;
hold on
for ii=Kp_vector
    Kp = ii;
    out = sim("motor_model.slx");
    plot(out.tout, out.motor_position.Data, 'DisplayName', sprintf("Kp=%d", ii))
    grid on
end
legend;

xlim([0 1])

title("DC motor response to position feedback control")
xlabel("Time (s)")
ylabel("Position (rad)")
