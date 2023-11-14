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

# Update column names based on your CSV file
accel_data = {'x': df['field.IMU.linear_acceleration.x'], 'y': df['field.IMU.linear_acceleration.y'], 'z': df['field.IMU.linear_acceleration.z']}
gyro_data = {'x': df['field.IMU.angular_velocity.x'], 'y': df['field.IMU.angular_velocity.y'], 'z': df['field.IMU.angular_velocity.z']}
mag_data = {'x': df['field.MagField.magnetic_field.x'], 'y': df['field.MagField.magnetic_field.y'], 'z': df['field.MagField.magnetic_field.z']}

# Convert quaternions to Euler angles
euler_angles = []
for row in df.iterrows():
    orientation_list = [row[1]['field.IMU.orientation.x'], row[1]['field.IMU.orientation.y'], row[1]['field.IMU.orientation.z'], row[1]['field.IMU.orientation.w']]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    euler_angles.append((roll, pitch, yaw))

# Create histograms
plt.figure(figsize=(12, 6))

# Accelerometer data histograms
plt.subplot(311)
plt.hist(accel_data['x'], bins=20, label='Accel X')
plt.hist(accel_data['y'], bins=20, label='Accel Y')
plt.hist(accel_data['z'], bins=20, label='Accel Z')
plt.legend()
plt.title('Accelerometer Data Histograms')

# Gyroscope data histograms
plt.subplot(312)
plt.hist(gyro_data['x'], bins=20, label='Gyro X')
plt.hist(gyro_data['y'], bins=20, label='Gyro Y')
plt.hist(gyro_data['z'], bins=20, label='Gyro Z')
plt.legend()
plt.title('Gyroscope Data Histograms')

# Magnetometer data histograms
plt.subplot(313)
plt.hist(mag_data['x'], bins=20, label='Mag X')
plt.hist(mag_data['y'], bins=20, label='Mag Y')
plt.hist(mag_data['z'], bins=20, label='Mag Z')
plt.legend()
plt.title('Magnetometer Data Histograms')

# Calculate and visualize mean and standard deviation
mean_accel_x = np.mean(accel_data['x'])
std_accel_x = np.std(accel_data['x'])
mean_accel_y = np.mean(accel_data['y'])
std_accel_y = np.std(accel_data['y'])
mean_accel_z = np.mean(accel_data['z'])
std_accel_z = np.std(accel_data['z'])

mean_gyro_x = np.mean(gyro_data['x'])
std_gyro_x = np.std(gyro_data['x'])
mean_gyro_y = np.mean(gyro_data['y'])
std_gyro_y = np.std(gyro_data['y'])
mean_gyro_z = np.mean(gyro_data['z'])
std_gyro_z = np.std(gyro_data['z'])

mean_mag_x = np.mean(mag_data['x'])
std_mag_x = np.std(mag_data['x'])
mean_mag_y = np.mean(mag_data['y'])
std_mag_y = np.std(mag_data['y'])
mean_mag_z = np.mean(mag_data['z'])
std_mag_z = np.std(mag_data['z'])

print(f"Mean Accel X: {mean_accel_x}, Std Deviation Accel X: {std_accel_x}")
print(f"Mean Accel Y: {mean_accel_y}, Std Deviation Accel Y: {std_accel_y}")
print(f"Mean Accel Z: {mean_accel_z}, Std Deviation Accel Z: {std_accel_z}")

print(f"Mean Gyro X: {mean_gyro_x}, Std Deviation Gyro X: {std_gyro_x}")
print(f"Mean Gyro Y: {mean_gyro_y}, Std Deviation Gyro Y: {std_gyro_y}")
print(f"Mean Gyro Z: {mean_gyro_z}, Std Deviation Gyro Z: {std_gyro_z}")

print(f"Mean Mag X: {mean_mag_x}, Std Deviation Mag X: {std_mag_x}")
print(f"Mean Mag Y: {mean_mag_y}, Std Deviation Mag Y: {std_mag_y}")
print(f"Mean Mag Z: {mean_mag_z}, Std Deviation Mag Z: {std_mag_z}")

# Save the histograms
plt.tight_layout()
plt.savefig('/home/febin/catkin_ws/src/analysis/imu_data_histograms.png')
plt.show()
