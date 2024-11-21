import serial
import csv
import time
import os

# Configuration
port = "COM15"  # Replace with your port number
baud_rate = 115200  # Adjust according to your device's configuration
output_dir = r"D:\20192230 Lee Dong-seop\IMU sensor"  # Output directory
output_file = os.path.join(output_dir, "gyro_data.csv")  # Output file path
duration = 10  # Duration to record data (in seconds)

# Ensure the output directory exists
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Initialize the serial connection
try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Connected to {port}")
except Exception as e:
    print(f"Error connecting to {port}: {e}")
    exit()

# Open the CSV file for writing
with open(output_file, mode='w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    # Write the header row
    writer.writerow(["Timestamp (s)", "Gyro X (deg/s)", "Gyro Y (deg/s)", "Gyro Z (deg/s)"])
    
    print("Collecting data...")
    start_time = time.time()
    
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time

        # Stop after the specified duration
        if elapsed_time > duration:
            break

        # Read a line of data from the serial port
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Example parsing: Adjust according to your device's output format
                # Assuming format: "GYRO,X=1.23,Y=4.56,Z=7.89"
                if line.startswith("GYRO"):
                    data = line.split(",")
                    gyro_x = float(data[1].split("=")[1])
                    gyro_y = float(data[2].split("=")[1])
                    gyro_z = float(data[3].split("=")[1])

                    # Write the data to the CSV file
                    writer.writerow([elapsed_time, gyro_x, gyro_y, gyro_z])
                    print(f"Time: {elapsed_time:.2f}s, Gyro X: {gyro_x}, Gyro Y: {gyro_y}, Gyro Z: {gyro_z}")
        except Exception as e:
            print(f"Error reading or parsing line: {e}")

# Close the serial connection
ser.close()
print(f"Data collection completed. File saved at: {output_file}")
