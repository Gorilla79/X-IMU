import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

# Define the folder path
base_path = r"C:\Users\Lab419_6\Documents\x-IMU3 GUI\Data Logger\Logged Data2 2024-11-20 19-33-30\x-IMU3 655166BA (USB)"

# Define file paths for the specific data
file_paths = {
    "HighGAccelerometer": os.path.join(base_path, "HighGAccelerometer.csv"),
    "Inertial": os.path.join(base_path, "Inertial.csv"),
    "Magnetometer": os.path.join(base_path, "Magnetometer.csv"),
    "Quaternion": os.path.join(base_path, "Quaternion.csv")
}


highg_df = pd.read_csv(file_paths['HighGAccelerometer'])
quaternion_df = pd.read_csv(file_paths['Quaternion'])
inertial_df = pd.read_csv(file_paths['Inertial'])

# Convert timestamps to seconds and calculate time differences
timestamps = highg_df['Timestamp (us)'].values
dt = np.diff(timestamps) / 1e6  # Convert microseconds to seconds

# Prepare accelerometer and quaternion data
accel_data = highg_df[['X (g)', 'Y (g)', 'Z (g)']].values * 9.81  # Convert g to m/s²
quaternion_data = quaternion_df[['W', 'X', 'Y', 'Z']].values

# Apply low-pass filter to accelerometer data to reduce noise
def low_pass_filter(data, alpha=0.1):
    filtered_data = np.zeros_like(data)
    filtered_data[0] = data[0]
    for i in range(1, len(data)):
        filtered_data[i] = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
    return filtered_data

accel_data = low_pass_filter(accel_data)

# Initialize position, velocity, and rotation
positions = [np.array([0, 0, 0])]
velocities = [np.array([0, 0, 0])]

# Process data for 3D tracking
for i in range(len(dt)):
    # Normalize quaternion to ensure valid rotation
    quat = quaternion_data[i] / np.linalg.norm(quaternion_data[i])
    rotation = R.from_quat(quat)
    accel_world = rotation.apply(accel_data[i])  # Rotate to world frame

    # Remove gravity component (assuming Z is upward)
    accel_world[2] -= 9.81

    # Integration for velocity and position
    velocity = velocities[-1] + accel_world * dt[i]
    position = positions[-1] + velocity * dt[i]

    # Apply threshold to reduce drift when stationary
    if np.linalg.norm(accel_world) < 0.1:
        velocity = np.array([0, 0, 0])  # Assume stationary

    velocities.append(velocity)
    positions.append(position)

positions = np.array(positions)

# 3D Visualization of the trajectory
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='3D Trajectory', color='b')
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('Improved 3D Tracking Trajectory')
ax.legend()
plt.show()