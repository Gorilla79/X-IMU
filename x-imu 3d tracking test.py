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

# Convert timestamps to seconds and calculate time differences
timestamps = highg_df['Timestamp (us)'].values
dt = np.diff(timestamps) / 1e6  # Convert microseconds to seconds

# Prepare accelerometer and quaternion data
accel_data = highg_df[['X (g)', 'Y (g)', 'Z (g)']].values * 9.81  # Convert g to m/s²
quaternion_data = quaternion_df[['W', 'X', 'Y', 'Z']].values

# Initialize position, velocity, and rotation
positions = [np.array([0, 0, 0])]
velocities = [np.array([0, 0, 0])]

# Process data for 3D tracking
for i in range(len(dt)):
    # Apply quaternion rotation to accelerometer data
    rotation = R.from_quat(quaternion_data[i])
    accel_world = rotation.apply(accel_data[i])  # Rotate to world frame
    
    # Integration for velocity and position
    velocity = velocities[-1] + accel_world * dt[i]
    position = positions[-1] + velocity * dt[i]
    
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
ax.set_title('3D Tracking Trajectory')
ax.legend()
plt.show()