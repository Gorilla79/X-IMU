import pandas as pd
import matplotlib.pyplot as plt
import os

# Define the folder path
base_path = r"C:\Users\Lab419_6\Documents\x-IMU3 GUI\Data Logger\Logged Data3 2024-11-21 18-56-33\x-IMU3 655166BA (USB)"

# Define file paths for the specific data
file_paths = {
    "HighGAccelerometer": os.path.join(base_path, "HighGAccelerometer.csv"),
    "Inertial": os.path.join(base_path, "Inertial.csv"),
    "Magnetometer": os.path.join(base_path, "Magnetometer.csv"),
    "Quaternion": os.path.join(base_path, "Quaternion.csv")
}

# Load data
dataframes = {name: pd.read_csv(path) for name, path in file_paths.items()}

# Function to plot data
def plot_data(df, x_col, y_cols, title, x_label, y_label):
    plt.figure(figsize=(10, 6))
    for y_col in y_cols:
        plt.plot(df[x_col], df[y_col], label=y_col)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()

# Plotting HighGAccelerometer data
df = dataframes['HighGAccelerometer']
plot_data(
    df,
    x_col='Timestamp (us)',
    y_cols=['X (g)', 'Y (g)', 'Z (g)'],
    title='High-G Accelerometer Data',
    x_label='Timestamp (us)',
    y_label='Acceleration (g)'
)

# Plotting Inertial data
df = dataframes['Inertial']
plot_data(
    df,
    x_col='Timestamp (us)',
    y_cols=['Gyroscope X (deg/s)', 'Gyroscope Y (deg/s)', 'Gyroscope Z (deg/s)'],
    title='Inertial Gyroscope Data',
    x_label='Timestamp (us)',
    y_label='Angular Velocity (deg/s)'
)
plot_data(
    df,
    x_col='Timestamp (us)',
    y_cols=['Accelerometer X (g)', 'Accelerometer Y (g)', 'Accelerometer Z (g)'],
    title='Inertial Accelerometer Data',
    x_label='Timestamp (us)',
    y_label='Acceleration (g)'
)

# Plotting Magnetometer data
df = dataframes['Magnetometer']
plot_data(
    df,
    x_col='Timestamp (us)',
    y_cols=['X (a.u.)', 'Y (a.u.)', 'Z (a.u.)'],
    title='Magnetometer Data',
    x_label='Timestamp (us)',
    y_label='Magnetic Field (a.u.)'
)

# Plotting Quaternion data
df = dataframes['Quaternion']
plot_data(
    df,
    x_col='Timestamp (us)',
    y_cols=['W', 'X', 'Y', 'Z'],
    title='Quaternion Data',
    x_label='Timestamp (us)',
    y_label='Quaternion Components'
)
