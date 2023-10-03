import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd

# In the line below, 'stationary_data.bag' is a rosbag file that contains GPS readings at a stationary point.
bag_file = '/home/febin/catkin_ws/src/data/stationary_data.bag'

bag = bagreader(bag_file)
data = bag.message_by_topic('/gps_data')
gps_data = pd.read_csv(data)

# Subtract minimum values to normalize the data
gps_data['utm_easting'] = gps_data['utm_easting'] - gps_data['utm_easting'].min()
gps_data['utm_northing'] = gps_data['utm_northing'] - gps_data['utm_northing'].min()
gps_data['longitude'] = gps_data['longitude'] - gps_data['longitude'].min()
gps_data['latitude'] = gps_data['latitude'] - gps_data['latitude'].min()
gps_data['header.stamp.secs'] = gps_data['header.stamp.secs'] - gps_data['header.stamp.secs'].min()

plt.rcParams.update({'font.size': 12})

# Plot UTM Easting vs UTM Northing
plt.figure(figsize=(10, 6))
plt.scatter(x='utm_easting', y='utm_northing', data=gps_data, s=50, label='UTM Easting vs UTM Northing', color='red')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.legend()
plt.title('UTM Easting vs UTM Northing')

# Plot Altitude vs Time
plt.figure(figsize=(10, 6))
plt.scatter(x='header.stamp.secs', y='altitude', data=gps_data, s=50, label='Altitude', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.title('Altitude vs Time')

# Plot Latitude vs Longitude
plt.figure(figsize=(10, 6))
plt.scatter(x='latitude', y='longitude', data=gps_data, s=50, label='Latitude vs Longitude', color='blue')
plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.legend()
plt.title('Latitude vs Longitude')

plt.show()

