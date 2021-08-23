#!/usr/bin/env python

import math
from math import sin, cos, pi
import serial
import rospy
import tf
import time
import re
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Imu


def talker():
    rospy.loginfo("Starting sensor publisher...")
    rospy.init_node('imu_odom_publisher')

    # get parameter
    imu_port = rospy.get_param('~port', '/dev/ttyUSB1') #"""dmesg grep usb : CP210X"""
    baud = rospy.get_param('~baud', 115200)


    # initialize IMU w/ serial commands
    imu_data = Imu()
    odom_data = Odometry()
    ser = serial.Serial(imu_port, baud)
    ser.write('<sof2>') # set quaternion output
    ser.readline()
    print(ser.readline())
    ser.write('<sog1>') # set gyro (angular velocity) data output
    print(ser.readline())
    ser.write('<soa5>') # set global linear velocity data output
    print(ser.readline())
    ser.write('<sem1>') # set magnetometer on
    print(ser.readline())
    ser.write('<sor100>') # set output rate to 100ms = 10hz
    print(ser.readline())
    #ser.write('<lpf20>')
    #print(ser.readline())

    imu_pub = rospy.Publisher("imu_data", Imu, queue_size=5)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
    odom_broadcaster = tf.TransformBroadcaster()
    stab_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz
    x = 0
    y = 0
    prev_str = ser.readline() # when prev_str is bad input -> :(
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        # start_time = time.time()
        ser.reset_input_buffer()

        str_temp = ser.readline()
        comma_cnt = len([m.start() for m in re.finditer(',', str_temp)])
        if (comma_cnt != 9):
            str_list = prev_str
        else:
            str_list = str_temp
            prev_str = str_temp
        
        str_list = str_list.split(',')
        str_list[0] = str_list[0].split('*')[1]
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "base_link"
        imu_data.orientation.z = float(str_list[0])
        imu_data.orientation.y = float(str_list[1])
        imu_data.orientation.x = float(str_list[2])
        imu_data.orientation.w = float(str_list[3])
        imu_data.linear_acceleration.x = 0
        imu_data.linear_acceleration.y = 0
        imu_data.linear_acceleration.z = 0
        imu_data.linear_acceleration_covariance[0] = -1
        imu_data.angular_velocity.x = 0
        imu_data.angular_velocity.y = 0
        imu_data.angular_velocity.z = 0
        imu_data.angular_velocity_covariance[0] = -1
        imu_pub.publish(imu_data)

        stab_broadcaster.sendTransform(
            (0, 0, 0),
            (float(str_list[2]),float(str_list[1]),float(str_list[0]),float(str_list[3])),
            rospy.Time.now(),
            "base_link",
            "map"
        )

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
