#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <sstream>
#include <algorithm>
#include <math.h>

using namespace std;

serial::Serial ser;

vector<string> split(string input, char delimiter) {
    vector<string> answer;
    stringstream ss(input);
    string temp;
 
    while (getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }
 
    return answer;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "imupublisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_prv("~");

    string imu_port = "/dev/ttyUSB0";
    int baudrate = 115200;

    if (!nh_prv.getParam("port", imu_port)) {
        vector<string> keys;
        ros::param::getParamNames(keys);
        if (find(keys.begin(), keys.end(), "/imu/port") != keys.end()) {
            nh.getParam("/imu/port", imu_port);
        } else {
            ROS_INFO_STREAM("Can't find '/imu/port' on param server. Using default value.");
        }
    }

    if (!nh_prv.getParam("baud", baudrate)) {
        vector<string> keys;
        ros::param::getParamNames(keys);
        if (find(keys.begin(), keys.end(), "/imu/port") != keys.end()) {
            nh.getParam("/imu/baud", baudrate);
        } else {
            ROS_INFO_STREAM("Can't find '/imu/baud' on param server. Using default value.");
        }
    }

    sensor_msgs::Imu imu_data;
    geometry_msgs::Vector3Stamped rpy_data;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 5);
    ros::Publisher rpy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("auv_rpy_stamped", 5);
    tf::TransformBroadcaster base_link_2_base_footprint_br;
    ros::Rate rate(100);

    try {
        ser.setPort(imu_port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");

        /*
            set quaternion output
            <sof1> : euler angle
            <sof2> : quaternion angle
        */
        ser.write("<sof2>");
        ROS_INFO_STREAM("imu setup: quaternion ON");
        ser.readline();
        /*
            set gyro (angular velocity) output
            <sog0> : gyro output ON
            <sog1> : gyro output OFF
        */
        ser.write("<sog1>");
        ROS_INFO_STREAM("imu setup: angular velocity ON");
        ser.readline();
        /*
            set linear acceleration/velocity output
            <soa0> : no output
            <soa1> : set accel data ON
            <soa2> : set local accel data w/o gravity ON
            <soa3> : set global accel data w/o gravity ON
            <soa4> : set local velocity data ON
            <soa5> : set global velocity data ON
        */
        ser.write("<soa2>");
        ROS_INFO_STREAM("imu setup: linear velocity ON");
        ser.readline();
        /*
            magnetometer enable/disable
            <sem0> : magnetometer OFF
            <sem1> : magnetometer ON    
        */
        ser.write("<sem1>");
        ROS_INFO_STREAM("imu setup: magnetometer ON");
        ser.readline();
        /*
            set distance output
            <sod0> : distance OFF
            <sod1> : local distance ON
            <sod2> : global distance OFF
        */
        ser.write("<sod0>");
        ROS_INFO_STREAM("imu setup: distance OFF");
        ser.readline();
        /*
            set output rate (ms)
            <sor1~1000> : set output rate
        */
        ser.write("<sor10>");
        ROS_INFO_STREAM("imu setup: 100hz output rate");
        ser.readline();
        ROS_INFO_STREAM("Start EBIMU node");
    }
    else {
        return -1;
    }

    while(ros::ok()){
        ser.flush();

        string str = ser.readline();
        vector<string> str_vec = split(str, ',');
        string str_head = split(str_vec[0],'*').back();
        str_vec.erase(str_vec.begin());
        str_vec.insert(str_vec.begin(), str_head);

        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";

        // note: ebimu gives quaternion data in order of (z, y, x, w)

        tf::Quaternion q;
        q.setZ(stod(str_vec[0]));
        q.setY(stod(str_vec[1]));
        q.setX(stod(str_vec[2]));
        q.setW(stod(str_vec[3]));
        double yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        mat.getRPY(roll, pitch, yaw);
        // invert sign due to EBIMU sign convention
        pitch = -pitch;
        yaw = -yaw;

        rpy_data.header.stamp = ros::Time::now();
        rpy_data.header.frame_id = "base_link";
        rpy_data.vector.x = roll;
        rpy_data.vector.y = pitch;
        rpy_data.vector.z = yaw;
        rpy_pub.publish(rpy_data);

        tf::Quaternion q_rh = tf::createQuaternionFromRPY(roll, pitch, yaw);

        imu_data.orientation.x = q_rh.x();
        imu_data.orientation.y = q_rh.y();
        imu_data.orientation.z = q_rh.z();
        imu_data.orientation.w = q_rh.w();
        imu_data.orientation_covariance = { 0.0007, 0, 0, 0, 0.0007, 0, 0, 0, 0.0007 };
        imu_data.linear_acceleration.x = stod(str_vec[7]);
        imu_data.linear_acceleration.y = stod(str_vec[8]);
        imu_data.linear_acceleration.z = stod(str_vec[9]);
        imu_data.linear_acceleration_covariance = { 0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005 };
        imu_data.angular_velocity.x = stod(str_vec[4]) * M_PI / 180.0;
        imu_data.angular_velocity.y = stod(str_vec[5]) * M_PI / 180.0;
        imu_data.angular_velocity.z = stod(str_vec[6]) * M_PI / 180.0;
        imu_data.angular_velocity_covariance = { 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001 };
        imu_pub.publish(imu_data);

        tf::Quaternion q_reverse_2d = tf::createQuaternionFromRPY(-roll, -pitch, 0);
        tf::Transform base_link_2_base_footprint_tf;
        base_link_2_base_footprint_tf.setOrigin(tf::Vector3(0, 0, -0.1));
        base_link_2_base_footprint_tf.setRotation(q_reverse_2d);
        base_link_2_base_footprint_br.sendTransform(
            tf::StampedTransform(
                base_link_2_base_footprint_tf,
                ros::Time::now(),
                "base_link",
                "base_footprint"
            )
        );

        rate.sleep();

    }
}