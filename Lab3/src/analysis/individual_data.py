import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

# Load data from the CSV file
csv_file = '/home/febin/catkin_ws/src/analysis/imu_data.csv'

# Specify column data types based on your data
data_types = {
    'field.IMU.linear_acceleration.x': float,
    'field.IMU.linear_acceleration.y': float,
    'field.IMU.linear_acceleration.z': float,
    'field.IMU.angular_velocity.x': float,
    'field.IMU.angular_velocity.y': float,
    'field.IMU.angular_velocity.z': float,
    'field.MagField.magnetic_field.x': float,
    'field.MagField.magnetic_field.y': float,
    'field.MagField.magnetic_field.z': float,
    'field.IMU.orientation.x': float,
    'field.IMU.orientation.y': float,
    'field.IMU.orientation.z': float,
    'field.IMU.orientation.w': float,
}

try:
    df = pd.read_csv(csv_file, dtype=data_types, low_memory=False)
except pd.errors.DtypeWarning:
    # Handle the mixed data types warning
    print("Warning: Mixed data types in the CSV file. The 'low_memory' option is set to False.")
    df = pd.read_csv(csv_file, dtype=data_types)

# Define dictionaries for accelerometer, gyroscope, and magnetometer data
accel_data = {
    'x': df['field.IMU.linear_acceleration.x'],
    'y': df['field.IMU.linear_acceleration.y'],
    'z': df['field.IMU.linear_acceleration.z']
}

gyro_data = {
    'x': df['field.IMU.angular_velocity.x'],
    'y': df['field.IMU.angular_velocity.y'],
    'z': df['field.IMU.angular_velocity.z']
}

mag_data = {
    'x': df['field.MagField.magnetic_field.x'],
    'y': df['field.MagField.magnetic_field.y'],
    'z': df['field.MagField.magnetic_field.z']
}

# Convert quaternions to Euler angles
euler_angles = []
for row in df.iterrows():
    orientation_list = [row[1]['field.IMU.orientation.x'], row[1]['field.IMU.orientation.y'], row[1]['field.IMU.orientation.z'], row[1]['field.IMU.orientation.w']]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    euler_angles.append((roll, pitch, yaw))

# Create histograms
plt.figure(figsize=(12, 18))  # Adjust the figure size

# Create separate histograms for accelerometer data (X, Y, Z)
plt.subplot(331)
plt.hist(accel_data['x'], bins=20, label='Accel X')
plt.legend()
plt.title('Accelerometer X Histogram')

plt.subplot(332)
plt.hist(accel_data['y'], bins=20, label='Accel Y')
plt.legend()
plt.title('Accelerometer Y Histogram')

plt.subplot(333)
plt.hist(accel_data['z'], bins=20, label='Accel Z')
plt.legend()
plt.title('Accelerometer Z Histogram')

# Create separate histograms for gyroscope data (X, Y, Z)
plt.subplot(334)
plt.hist(gyro_data['x'], bins=20, label='Gyro X')
plt.legend()
plt.title('Gyroscope X Histogram')

plt.subplot(335)
plt.hist(gyro_data['y'], bins=20, label='Gyro Y')
plt.legend()
plt.title('Gyroscope Y Histogram')

plt.subplot(336)
plt.hist(gyro_data['z'], bins=20, label='Gyro Z')
plt.legend()
plt.title('Gyroscope Z Histogram')

# Create separate histograms for magnetometer data (X, Y, Z)
plt.subplot(337)
plt.hist(mag_data['x'], bins=20, label='Mag X')
plt.legend()
plt.title('Magnetometer X Histogram')

plt.subplot(338)
plt.hist(mag_data['y'], bins=20, label='Mag Y')
plt.legend()
plt.title('Magnetometer Y Histogram')

plt.subplot(339)
plt.hist(mag_data['z'], bins=20, label='Mag Z')
plt.legend()
plt.title('Magnetometer Z Histogram')

# Calculate and visualize mean and standard deviation
mean_accel_x = np.mean(accel_data['x'])
std_accel_x = np.std(accel_data['x'])

mean_gyro_x = np.mean(gyro_data['x'])
std_gyro_x = np.std(gyro_data['x'])

mean_mag_x = np.mean(mag_data['x'])
std_mag_x = np.std(mag_data['x'])

print(f"Mean Accel X: {mean_accel_x}, Std Deviation Accel X: {std_accel_x}")
print(f"Mean Gyro X: {mean_gyro_x}, Std Deviation Gyro X: {std_gyro_x}")
print(f"Mean Mag X: {mean_mag_x}, Std Deviation Mag X: {std_mag_x}")

# Save the histograms
plt.tight_layout()
plt.savefig('/home/febin/catkin_ws/src/analysis/imu_data_histograms.png')
plt.show()
