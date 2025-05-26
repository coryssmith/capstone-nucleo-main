import serial
import time

def send_command(port, motor, angle):
    command = f"M{motor} {angle}\n"
    port.write(command.encode())
    print(f"Sent: {command.strip()}")

# Main function
def main():
    port_name = 'COM3'  # Replace with your serial port (e.g., /dev/ttyUSB0 on Linux)
    baud_rate = 115200

    try:
        with serial.Serial(port_name, baud_rate, timeout=1) as ser:
            print(f"Connected to {port_name} at {baud_rate} baud.")

            send_command(ser, 1, 90)
            time.sleep(10)
            send_command(ser, 2, 45)
            time.sleep(10)
            send_command(ser, 3, 180)
            time.sleep(10)

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("Program terminated by user.")

if __name__ == "__main__":
    main()
