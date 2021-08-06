# ebimu_odometry
ROS package which publishes odometry message from EBIMU-9DOFV4 IMU </br>
*Recommended: use imu_odom_pub_v1.py<br/>*
### Recent Changes
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
3. rosrun ebimu_odometry {script_name}

Explanation
--

```imu_odom_pub_v1.py```<br/>
* publish both odometry and IMU data.
    * IMU data:
        * orientation: quaternion from IMU raw data
        * linear acceleration: all zero
        * angular velocity: all zero
    * odometry:
        * position: calculated by integral
        * twist: linear&angular velocity calculated by IMU
<br/>

```imu_odom_pub_v2.py```<br/>
* publish both odometry and IMU data.
    * IMU data:
        * orientation: quaternion from IMU raw data
        * linear acceleration: all zero
        * angular velocity: all zero
    * odometry:
        * position: locally traveled distance calculated by IMU
        * twist: linear&angular velocity calculated by IMU
<br/>
The only difference between v1 and v2 is whether it uses distance data from IMU data or not.

