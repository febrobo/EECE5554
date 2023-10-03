
import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import pandas as pd

# In the line below, 'walking_data.bag' is a rosbag file that contains GPS readings during walking.
bag = bagreader('/home/febin/catkin_ws/src/data/walking_data.bag')

data = bag.message_by_topic('/gps_data')
gps_readings = pd.read_csv(data)

# Subtract minimum values to normalize the data
gps_readings['utm_easting'] = gps_readings['utm_easting'] - gps_readings['utm_easting'].min()
gps_readings['utm_northing'] = gps_readings['utm_northing'] - gps_readings['utm_northing'].min()
gps_readings['longitude'] = gps_readings['longitude'] - gps_readings['longitude'].min()
gps_readings['latitude'] = gps_readings['latitude'] - gps_readings['latitude'].min()
gps_readings['header.stamp.secs'] = gps_readings['header.stamp.secs'] - gps_readings['header.stamp.secs'].min()

plt.rcParams.update({'font.size': 12})

# Plot utm_easting vs utm_northing
plt.figure(figsize=(10, 6))
plt.scatter(
    x='utm_easting',
    y='utm_northing',
    data=gps_readings,
    s=50,
    c='red',  # Set color to red
    label='utm_northing vs utm_easting'
)
plt.xlabel('utm_easting (m)')
plt.ylabel('utm_northing (m)')
plt.legend()
plt.title('UTM Northing vs UTM Easting')

# Plot altitude vs time
plt.figure(figsize=(10, 6))
plt.scatter(
    x='header.stamp.secs',
    y='altitude',
    data=gps_readings,
    s=50,
    c='green',  # Set color to green
    label='altitude'
)
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.title('Altitude vs Time')

# Plot longitude vs latitude
plt.figure(figsize=(10, 6))
plt.scatter(
    x='latitude',
    y='longitude',
    data=gps_readings,
    s=50,
    c='blue',  # Set color to blue
    label='latitude vs longitude'
)
plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.legend()
plt.title('Longitude vs Latitude')

plt.show()
