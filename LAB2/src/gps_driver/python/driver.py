#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
from gps_driver.msg import gps_msg
from std_msgs.msg import Time
import pynmea2
import io
import utm

def talker():
    rospy.init_node('gps_publisher')  # Initialize ROS node
    pub = rospy.Publisher('gps_data', gps_msg, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz publishing rate

    with serial.Serial('/dev/ttyUSB0', 57600, timeout=5.0) as se:
        while not rospy.is_shutdown():
            receive = se.readline().decode()
            if "$GNGGA" in receive:  # Change to GNGGA
                gpsdata = receive.split(",")

                # Check if the GPS data has enough elements
                if len(gpsdata) >= 15:  # Modify this number if needed
                    try:
                        lat_gpsdata = float(gpsdata[2])
                        DD = int(lat_gpsdata / 100)
                        MM = lat_gpsdata - DD * 100
                        lat_decimal = DD + MM / 60
                        long_gpsdata = float(gpsdata[4])
                        long_DD = int(long_gpsdata / 100)
                        long_MM = long_gpsdata - long_DD * 100
                        long_decimal = long_DD + (long_MM / 60)

                        if gpsdata[3] == "S":
                            lat_decimal *= -1

                        if gpsdata[5] == "W":
                            long_decimal *= -1

                        x, y, zone_number, zone_letter = utm.from_latlon(lat_decimal, long_decimal)

                        # Create a GPS message
                        gps_msg_data = gps_msg()
                        gps_msg_data.header.stamp = rospy.Time.now()
                        gps_msg_data.latitude = lat_decimal
                        gps_msg_data.longitude = long_decimal
                        gps_msg_data.utm_easting = x
                        gps_msg_data.utm_northing = y
                        gps_msg_data.zone = zone_number
                        gps_msg_data.letter = zone_letter
                        gps_msg_data.gnss_fix_quality = gpsdata[6]  # Add GNSS fix quality

                        # Publish the message
                        pub.publish(gps_msg_data)
                    except ValueError:
                        rospy.logwarn('Invalid GPS data: %s', receive)

                rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    except serial.SerialException as e:
        pass
