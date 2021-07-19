#!/usr/bin/env python

import math
from math import sin, cos, pi
import serial
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def IMUParser(imuserial):
    str_list = imuserial.readline().rstrip().split(',')
    th = -float(str_list[2])
    vx = float(str_list[6])
    vy = float(str_list[7])
    vth = float(str_list[5])
    return th, vx, vy, vth

print("Starting odometry publisher...")
rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

imu_port = "/dev/ttyUSB0"
ser = serial.Serial(imu_port, 115200) # determine manually
ser.write('<sof2>') # Output format: Quaternion
ser.readline()
print(ser.readline())
ser.write('<sog1>') # Set gyro(angular velocity) (roll, pitch, yaw) data output
print(ser.readline())
ser.write('<soa5>') # set global acceleration(ax, ay, az) data output
print(ser.readline())
ser.write('<sem1>') # enable magnetometer
print(ser.readline())
ser.write('<sor10>')  # output rate: 10ms = 100hz
print(ser.readline())
imu_data = IMUParser(ser)
speed = 1
x = 0.0
y = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100) # 100hz
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    ser.reset_input_buffer()
    imu_data = IMUParser(ser)
    th = imu_data[0]
    vx = imu_data[1]
    vy = imu_data[2]
    vth = imu_data[3]

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt

    x += delta_x
    y += delta_y

    
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
