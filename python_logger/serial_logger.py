import serial
import csv
import time

SERIAL_PORT = 'COM3'
BAUD_RATE = 9600
OUTPUT_FILE = 'pendulum_data.csv'

def log_serial_data():
    print(f"Connecting to arduino on {SERIAL_PORT}")

    try:
        ser = 0

    except:
        pass