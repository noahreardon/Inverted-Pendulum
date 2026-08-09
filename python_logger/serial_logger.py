import serial
import csv
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
OUTPUT_FILE = 'pendulum_data.csv'

def log_serial_data():
    print(f"Connecting to arduino on {SERIAL_PORT}...")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"Connected! Logging data to '{OUTPUT_FILE}'. Press Ctrl+C to stop.")

        with open(OUTPUT_FILE, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)

            if file.tell() == 0:
                writer.writerow(['Timestamp', 'Raw Data'])

            while True:
                if ser.in_waiting > 0:
                    raw_line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if raw_line:
                        timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                        print(f"[{timestamp}] {raw_line}")

                        writer.writerow([timestamp, raw_line])
                        file.flush()
    
    except serial.SerialException as e:
        print(f"Serial Error: {e}. Is the port correct or already open in Arduino IDE?")
    except KeyboardInterrupt:
        print("\nLogging stopped by user. File saved.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    log_serial_data()
