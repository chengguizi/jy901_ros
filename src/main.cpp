/**
 * @file main.cpp
 * @author Huimin Cheng (NUS)
 * @brief ROS Wrapper for JY901, without ability to configure
 * @version 0.1
 * @date 2018-10-16
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <yaml-cpp/yaml.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <cmath>

#include "jy901.h"

struct Range{
    int accel;
    int gyro;
    const double g = 9.8;
    const int k = 32768;
}imu_range;

struct Calib{
        double scale[3];
        double bias[3];
};

ros::Publisher _imu_pub;
ros::Publisher _mag_pub;

ros::Time _ros_time_first_frame;
ros::Time _imu_time_first_frame;

ros::Time getTime(const STime &time)
{
    uint64_t sec = (unsigned int)time.ucHour*3600 + (unsigned int)time.ucMinute*60 + (unsigned int)time.ucSecond;
    ros::Time rosTime;
    rosTime.fromNSec(sec*1e9 + (uint64_t)time.usMiliSecond*1e6);
    return rosTime;
}


bool apply_calibration;
Calib imu_calib;
void calibrate_imu(sensor_msgs::Imu &imu_msg){
    imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.x / imu_calib.scale[0];
    imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.y / imu_calib.scale[1];
    imu_msg.linear_acceleration.z = imu_msg.linear_acceleration.z / imu_calib.scale[2];
}

sensor_msgs::Imu getImu(const SAcc &acc, const SGyro &gyro, const SQuater &quat)
{
    sensor_msgs::Imu rosImu;
    rosImu.linear_acceleration.x = (double)acc.a[0]/32768*imu_range.accel*imu_range.g;
    rosImu.linear_acceleration.y = (double)acc.a[1]/32768*imu_range.accel*imu_range.g;
    rosImu.linear_acceleration.z = (double)acc.a[2]/32768*imu_range.accel*imu_range.g;

    if (apply_calibration){
        calibrate_imu(rosImu);
    }

    const double deg2rad = M_PI/180.0;
    rosImu.angular_velocity.x = (double)gyro.w[0]/32768*imu_range.gyro*deg2rad;
    rosImu.angular_velocity.y = (double)gyro.w[1]/32768*imu_range.gyro*deg2rad;
    rosImu.angular_velocity.z = (double)gyro.w[2]/32768*imu_range.gyro*deg2rad;

    // Oritentation used east-north-up
    rosImu.orientation.w = (double)quat.q0/32768;
    rosImu.orientation.x = (double)quat.q1/32768;
    rosImu.orientation.y = (double)quat.q2/32768;
    rosImu.orientation.z = (double)quat.q3/32768;

    return rosImu;
}

sensor_msgs::MagneticField getMag(const SMag &mag)
{
    sensor_msgs::MagneticField rosMag;

    rosMag.magnetic_field.x = mag.h[0];
    rosMag.magnetic_field.y = mag.h[1];
    rosMag.magnetic_field.z = mag.h[2];

    return rosMag;
}

void rosPublish(CJY901::Data &data)
{
    ros::Time stamp = ros::Time::now(); // ros::WallTime::now();
    // initialise first-frame time
    if (_imu_time_first_frame.isZero())
    {
        _ros_time_first_frame.fromSec(ros::WallTime::now().toSec());
        _imu_time_first_frame = getTime(data.stcTime);
    }

    auto imu_time = getTime(data.stcTime);
    auto imu_msg = getImu(data.stcAcc, data.stcGyro, data.stcQuater);
    auto mag_msg = getMag(data.stcMag);

    assert(imu_time >= _imu_time_first_frame);

    // ros::Time stamp = _ros_time_first_frame + ( imu_time - _imu_time_first_frame);

    imu_msg.header.stamp = mag_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "imu_frame";
    
    _imu_pub.publish(imu_msg);
    _mag_pub.publish(mag_msg);

    ROS_INFO_STREAM_THROTTLE (2, "Published ROS seq = " << data.seq << ", time=" << stamp );
}

int main (int argc, char** argv){

    //// ROS
    ros::init(argc, argv, "jy901");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    _imu_pub = nh.advertise<sensor_msgs::Imu>("imu0",5);
	_mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag0",5);

    _ros_time_first_frame = ros::Time(0);
	_imu_time_first_frame = ros::Time(0);

    //// Get IMU range, must be consistent with IMU register settings!
    ROS_ASSERT(local_nh.getParam("accel_range", imu_range.accel));
    ROS_ASSERT(local_nh.getParam("gyro_range", imu_range.gyro));

    //// Check if calibration is needed
    
    ROS_ASSERT(local_nh.getParam("apply_calibration", apply_calibration));
    if (apply_calibration){
        std::string imu_calib_file;
        ROS_ASSERT(local_nh.getParam("imu_calib_file", imu_calib_file));

        std::cout << "\n//////////////////Applying Calibration from " << imu_calib_file << std::endl;
        YAML::Node node = YAML::LoadFile(imu_calib_file);

        assert(node["scale"].IsSequence() && node["scale"].size() == 3);
        assert(node["bias"].IsSequence() && node["bias"].size() == 3);

        for (int i=0;i<3;i++)
        {
            imu_calib.scale[i] = node["scale"][i].as<double>();
            std::cout << "scale[" << i << "]=" << imu_calib.scale[i] << std::endl;
        }
            
        for (int i=0;i<3;i++)
        {
            imu_calib.bias[i] = node["bias"][i].as<double>();
            std::cout << "bias[" << i << "]=" << imu_calib.bias[i] << std::endl;
        }  
            
    }else
        std::cout << "\n//////////////////Not using calibration file.//////////////////\n" << std::endl;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    //// Open Serial Port
    std::string port;
    ROS_ASSERT(local_nh.getParam("port", port));
    int baudrate;  // 115200 is not enough, almost 4ms to communicate, too long
    ROS_ASSERT(local_nh.getParam("baudrate", baudrate));
    auto timeout = serial::Timeout(1,1); // both inter_byte_timeout and read_timeout_constant
    serial::Serial serial(port, baudrate, timeout);

    if(serial.isOpen())
        std::cout << "Serial Port is Open:" << port << std::endl;
    else
    {
        std::cout << "Serial Port fails to open: " << port << std::endl;
        return -1;
    }
    
    // serial.setTimeout(timeout); // inter_byte_timeout, read_timeout_constant, read_timeout_multiplier, write_timeout_constant, write_timeout_multiplier
    
    

    //// Instance of JY901
    CJY901 jy901;
    ros::Rate loop_rate(400);
    serial.flush();
    unsigned char buffer[160];
    serial.read(buffer,160); // just to make things stable, in latency


    while(ros::ok()){
        static int last_seq = 0;
        try {
            if(serial.available()){
                unsigned int len;
                
                len = serial.read(buffer,160);
                
                jy901.CopeSerialData(buffer,len);
            }
        }
        catch (const std::exception& e){
            std::cout << "Catch Exception: " << e.what() << std::endl;
            exit(-1);
        }

        if (last_seq != jy901.data.seq){
            last_seq = jy901.data.seq;
            rosPublish(jy901.data);
        }
        loop_rate.sleep();
    }

    return 0;
}