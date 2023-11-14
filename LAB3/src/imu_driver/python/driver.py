#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
import math
from imu_driver.msg import imu_msg  # Import the custom message
import sys

def euler_to_quaternion(roll, pitch, yaw):
    # Convert Euler angles (in radians) to Quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

def parse_sensor_data(data_str):
    if "$VNYMR" in data_str:
        values = data_str.split(',')
        
        if len(values) >= 13:
            try:
                # print("hi")
                ax = float(values[7])
                ay = float(values[8])
                az = float(values[9])
                gx = float(values[10])
                gy = float(values[11])
                gz = float(values[12][:-5])
                roll = float(values[3])
                pitch = float(values[2])
                yaw = float(values[1])
                mx = float(values[4])
                my = float(values[5])
                mz = float(values[6])
                # print(ax, ay, az, gx, gy, gz, roll, pitch, yaw, mx, my, mz)
                return ax, ay, az, gx, gy, gz, roll, pitch, yaw, mx, my, mz

            except (ValueError, IndexError) as e:
                print(e)

    # If the data format is not as expected or parsing fails, return None
    return None

def imu_driver():
    rospy.init_node('imu_driver', anonymous=True)
    imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)
    rate = rospy.Rate(40)  # 40Hz

    # Set the serial port and baud rate
    #serial_port = '/dev/ttyUSB0'
    portname = sys.argv[1]
    serial_port = rospy.get_param("~port", portname)
    baud_rate = rospy.get_param("~baudrate", 115200)
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(portname)

    while not rospy.is_shutdown():
        custom_msg = imu_msg()
        try:
            imu_data_str = ser.readline().decode('utf-8')
            # rospy.loginfo("Received data: %s" % imu_data_str)
            imu_data = parse_sensor_data(imu_data_str)
            print(imu_data)
            if imu_data:
                ax, ay, az, gx, gy, gz, roll, pitch, yaw, mx, my, mz = imu_data

                orientation = euler_to_quaternion(math.radians(roll), math.radians(pitch), math.radians(yaw))

                custom_msg.Header = Header(frame_id="IMU1_Frame")
                custom_msg.IMU.header = custom_msg.Header
                custom_msg.IMU.orientation = orientation
                custom_msg.IMU.angular_velocity = Vector3(gx, gy, gz)
                custom_msg.IMU.linear_acceleration = Vector3(ax, ay, az)

                custom_msg.MagField.header = custom_msg.Header
                custom_msg.MagField.magnetic_field = Vector3(mx, my, mz)

                custom_msg.raw_imu_string = imu_data_str
                # print(custom_msg)
                #rospy.loginfo("Publishing IMU data")
                imu_pub.publish(custom_msg)
                rate.sleep()

        except serial.SerialException:
            rospy.logwarn("Failed to read from the serial port.")


if __name__ == '__main__':
    try:
        imu_driver()
    except rospy.ROSInterruptException:
        pass
    except serial.SerialException as e:
        pass
