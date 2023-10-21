#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from imu_driver.msg import imu_msg

def talker():
    rospy.init_node('imu_publisher')  # Initialize ROS node
    #imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu', MagneticField, queue_size=10)
    imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)

    imu_rate = 40.0  # IMU data rate in Hz

    while not rospy.is_shutdown():
        with serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0) as imu_serial:
            try:
                imu_data = imu_serial.readline()
                if imu_data.startswith(b'$VNYMR'):
                    # Parse IMU data from the $vnymr string
                    imu_values = imu_data.split(b',')
                    if len(imu_values) >= 13:
                        # Extract data from the byte sequence
                        roll, pitch, yaw, ax, ay, az, gx, gy, gz, mx, my, mz_raw = imu_values[1:]
        
        # Extract the numeric part and convert it to a float
                        mz = float(mz_raw.split(b'*')[0])

                        # Create Header
                        header = Header()
                        header.stamp = rospy.Time.now()

                        # Populate IMU message
                        imu_message = Imu()
                        imu_message.header = header
                        imu_message.orientation = Quaternion(0, 0, 0, 0)  # No orientation data available
                        imu_message.orientation_covariance = [-1] * 9  # No orientation data available
                        imu_message.angular_velocity = Vector3(gx, gy, gz)
                        imu_message.angular_velocity_covariance = [-1] * 9  # No covariance information
                        imu_message.linear_acceleration = Vector3(ax, ay, az)
                        imu_message.linear_acceleration_covariance = [-1] * 9  # No covariance information

                        # Publish IMU message
                        imu_pub.publish(imu_message)

                        # Populate MagneticField message
                        mag_message = MagneticField()
                        mag_message.header = header
                        mag_message.magnetic_field = Vector3(mx, my, mz)
                        mag_message.magnetic_field_covariance = [-1] * 9  # No covariance information

                        # Publish MagneticField message
                        mag_pub.publish(mag_message)

                        # Create Custom IMU message
                        custom_message = imu_msg()
                        custom_message.header = header
                        custom_message.roll = roll
                        custom_message.pitch = pitch
                        custom_message.yaw = yaw
                        custom_message.acceleration = Vector3(ax, ay, az)
                        custom_message.angular_velocity = Vector3(gx, gy, gz)
                        custom_message.magnetic_field = Vector3(mx, my, mz)

                        # Publish Custom IMU message
                        custom_pub.publish(custom_message)
            except serial.SerialException as e:
                rospy.logwarn("Serial connection error: {}".format(e))
                rospy.sleep(1.0)  # Wait before retrying

        # Sleep to maintain the desired IMU data rate
        rospy.sleep(1.0 / imu_rate)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
