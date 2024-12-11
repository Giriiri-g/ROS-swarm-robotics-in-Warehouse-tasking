import serial
import time

# Configure the serial connection
PORT = "COM3"  # Replace with your ESP32 port (e.g., "/dev/ttyUSB0" on Linux)
BAUD_RATE = 115200

# Create a serial object
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

def send_string_to_esp32(string):
    try:
        if ser.is_open:
            print(f"Sending string: {string}")
            ser.write(string.encode())  # Send string
            time.sleep(1)  # Wait for ESP32 to process
            print("String sent successfully.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    send_string_to_esp32("Hello ESP32!")
