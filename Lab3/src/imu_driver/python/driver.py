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

    # Create a single publisher for all message types
    imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)

    imu_rate = 40.0  # IMU data rate in Hz

    while not rospy.is_shutdown():
        try:
            with serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0) as imu_serial:
                while not rospy.is_shutdown():
                    imu_data = imu_serial.readline()
                    if imu_data.startswith(b'$VNYMR'):
                        # Parse IMU data from the $vnymr string
                        imu_values = imu_data.split(b',')
                        if len(imu_values) >= 13:
                            # Extract data from the byte sequence
                            roll, pitch, yaw, ax, ay, az, gx, gy, gz, mx, my, mz = map(float, imu_values[1:])

                            # Create Header
                            header = Header()
                            header.stamp = rospy.Time.now()

                            # Create Custom IMU message
                            custom_message = imu_msg()
                            custom_message.header = header
                            custom_message.roll = roll
                            custom_message.pitch = pitch
                            custom_message.yaw = yaw
                            custom_message.acceleration = Vector3(ax, ay, az)
                            custom_message.angular_velocity = Vector3(gx, gy, gz)
                            custom_message.magnetic_field = Vector3(mx, my, mz)

                            # Publish the custom IMU message
                            imu_pub.publish(custom_message)
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
