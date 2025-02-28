import serial
import time

def read_arduino_data():
    try:
        ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            timeout=1
        )
        print("Connected to Arduino!")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                try:
                    value = float(line)  # Convert to numerical data
                    yield value  # Yield data for visualization
                except ValueError:
                    print(f"Ignored non-numeric data: {line}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

# Example usage:
data_generator = read_arduino_data()
for value in data_generator:
    print(value)  # Replace with visualization code