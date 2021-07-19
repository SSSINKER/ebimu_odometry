# ebimu_odometry
ROS package which publishes odometry message from EBIMU-9DOFV4 IMU </br>

### Recent Changes
* 2021.07.19 </br>
renewed old repository and files.

Installation Guide
--

1. clone this repository at catkin_ws/src
2. ~/catkin_ws $ catkin_make
3. rosrun ebimu_odometry {script_name}

Explanation
--

```
imu_no_odom
    publish only IMU data

odom_publisher
    publish odometry data. Velocity(twist) is based on IMU data and distance(position) is calculated by IMU velocity data with simple integral

imu_odom_pub_v1
    publish both odometry and IMU data.

    IMU data:
        orientation: quaternion from IMU raw data
        linear acceleration: all zero
        angular velocity: all zero
    odometry:
        position: calculated by integral
        twist: linear&angular velocity calculated by IMU

imu_odom_pub_v2
    publish both odometry and IMU data.

    IMU data:
        orientation: quaternion from IMU raw data
        linear acceleration: all zero
        angular velocity: all zero
    odometry:
        position: locally traveled distance calculated by IMU
        twist: linear&angular velocity calculated by IMU

The only difference between v1 and v2 is whether it uses distance data from IMU data or not.
```
