import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def plot_csv(CSV_FILE: str):
    df = pd.read_csv(CSV_FILE)
    # remove the first row
    df = df.iloc[1:]
    # get each variable as a list of values
    raw_data: list[str] = df['Raw Data'].tolist()
    t = []
    motor_angle = []
    pendulum_angle = []
    motor_velocity = []
    pendulum_velocity = []
    motor_command = []
    proportional_error = []
    dt = []

    for i in range(len(raw_data)):
        current_line: str = raw_data[i]
        split_line: list[str] = current_line.split(",")
        print(split_line)
        t.append(float(split_line[0]))
        motor_angle.append(float(split_line[1]))
        pendulum_angle.append(float(split_line[2]))
        motor_velocity.append(float(split_line[3]))
        pendulum_velocity.append(float(split_line[4]))
        motor_command.append(float(split_line[5]))
        proportional_error.append(float(split_line[6]))
        dt.append(float(split_line[7]))

    # shift all the time values by the first value so that the time starts at zero
    t_shifted: list[float] = [i - t[0] for i in t]
    # create a scaled version of the time vector to plot in seconds instead of microseconds
    t_scaled: list[float] = [i / 1e6 for i in t_shifted]
        
    plt.plot(t_scaled, motor_angle, label='Motor angle')
    plt.plot(t_scaled, pendulum_angle, label='Pendulum angle')
    plt.plot(t_scaled, motor_velocity, label='Motor velocity')
    plt.plot(t_scaled, pendulum_velocity, label='Pendulum velocity')
    plt.plot(t_scaled, motor_command, label='Voltage command')
    plt.plot(t_scaled, proportional_error, label='Pendulum angle error')
    plt.plot(t_scaled, dt, label='dt')
    plt.ylim([-50, 50])
    plt.xlabel("Time")
    plt.ylabel("Amplitude")
    plt.title("Outputs from inverted pendulum")
    plt.legend()
    plt.show()
    
    
    
if __name__ == "__main__":
    plot_csv("pendulum_data.csv")
