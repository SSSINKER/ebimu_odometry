#!/usr/bin/env python

import serial
import rospy
import tf

from sensor_msgs.msg import Imu

def talker():
    rospy.loginfo("Starting sensor publisher...")
    rospy.init_node('imu_publisher')
    imu_port = "/dev/ttyUSB2"
    ser = serial.Serial(imu_port, 115200) # determine manually
    ser.write('<sof2>')
    ser.write('<sog1>')
    ser.write('<soa1>')

    sensor_pub = rospy.Publisher("sensor", Imu, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        str_list = imuserial.readline().rstrip().split(',')
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = frame_id
        imu_data.header.seq = seq
        imu_data.orientation.w = float(str_list[3])
        imu_data.orientation.x = float(str_list[2])
        imu_data.orientation.y = float(str_list[1])
        imu_data.orientation.z = float(str_list[0])
        imu_data.linear_acceleration.x = float(str_list[4])
        imu_data.linear_acceleration.y = float(str_list[5])
        imu_data.linear_acceleration.z = float(str_list[6])
        imu_data.linear_acceleration_covariance[0] = -1
        imu_data.angular_velocity.x = float(str_list[7])
        imu_data.angular_velocity.y = float(str_list[8])
        imu_data.angular_velocity.z = float(str_list[9])
        imu_data.angular_velocity_covariance[0] = -1

        rospy.loginfo(Imu)
        sensor_pub.publish(Imu)
        rate.sleep()

imu_data = Imu()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
