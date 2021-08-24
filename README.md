# ebimu_odometry
ROS package which publishes odometry message from EBIMU-9DOFV4 IMU </br>

### Recent Changes

* 2021.08.25 </br>
publish ```/auv_rpy_stamped``` topic </br>

* 2021.08.24 </br>
c++ version of imupublisher </br>

* 2021.08.15 </br>
revised code, tf, odometry publisher </br>

* 2021.08.06 </br>
updated v2.py code <br/>
changed transformations

* 2021.07.22 </br>
added rosparam input, but not working well.

* 2021.07.19 </br>
renewed old repository and files.

Installation Guide
--

1. clone this repository at catkin_ws/src
2. ~/catkin_ws $ catkin_make
3. rosrun ebimu_odometry imupublisher

Nodes
--

## imu_odom_pub (not converted to c++ yet) <br/>
Publish both odometry and IMU data.
   * Subscribed Topics
      * NaN
   * Published Topics
      * ```/imu_data``` (sensors_msg/Imu) : IMU data.
        * orientation: quaternion from IMU magnetometer data
        * linear acceleration: all zero
        * angular velocity: all zero
      * ```/odom``` (nav_msgs/Odometry) : Odometry data.
         * position: calculated by integral
         * twist: linear&angular velocity calculated by IMU
   * Parameters
      * ```~port``` (```string```, default: '/dev/ttyUSB0') : USB port number
      * ```~baud``` (```integer```, default: 115200) : baudrate
   * Provided tf Transforms
      * ```odom``` -> ```base_footprint```
      * ```base_footprint``` -> ```base_link```
<br/>

## imupublisher <br/>
Publish only IMU data.
   * Subscribed Topics
      * NaN
   * Published Topics
      * ```/imu_data``` (sensors_msg/Imu) : IMU data.
        * orientation: quaternion from IMU magnetometer data
        * linear acceleration: from IMU accelerometer data
        * angular velocity: from IMU gyrometer data
      * ```/auv_rpy_stamped``` (geometry_msg/Vector3Stamped) : roll, pitch, yaw in radians
   * Parameters
      * ```~port``` (```string```, default: '/dev/ttyUSB0') : USB port number
      * ```~baudrate``` (```integer```, default: 115200) : baudrate
   * Provided tf Transforms
      * ```base_link``` -> ```base_footprint```
<br/>

