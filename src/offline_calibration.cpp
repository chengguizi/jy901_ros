/**
 * @file offline_calibrate.cpp
 * @author Huimin Cheng (NUS)
 * @brief Automatic routine to determine the max/min for each axis, along with it bias
 * @version 0.1
 * @date 2018-10-18
 * 
 * @copyright Copyright (c) 2018
 * 
 */


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <cmath>
#include <fstream>
#include <ctime>

#include <unistd.h>
#include <linux/limits.h>
#include <bitset>

const int BUFFER_SIZE = 512;



struct Limits{
    double xMin = 0;
    double xMax = 0;
    double yMin = 0;
    double yMax = 0;
    double zMin = 0;
    double zMax = 0;

    std::vector<double> xMin_msg;
    std::vector<double> xMax_msg;
    std::vector<double> yMin_msg;
    std::vector<double> yMax_msg;
    std::vector<double> zMin_msg;
    std::vector<double> zMax_msg;

    std::bitset<6> valid = 0;
};

Limits accelLimits;

bool isCalibrating = false;

void imuCallback(const sensor_msgs::Imu::ConstPtr &imu){
    // std::cout << "Received: " << imu->header.seq << std::endl;

    static std::vector<sensor_msgs::Imu> imu_buffer(BUFFER_SIZE);
    static int next_idx = 0;
    static std::vector<double> accel(3,0.0);
    static std::vector<double> gyro(3,0.0);

    if (!isCalibrating)
        return;

    bool do_reset = false;

    //// STEP 0: Abort if spinning is too great
    double spin_norm = imu->angular_velocity.x*imu->angular_velocity.x +
                        imu->angular_velocity.y*imu->angular_velocity.y +
                        imu->angular_velocity.z*imu->angular_velocity.z;

    const double spin_threshold = 5.0/180*M_PI;
    const double spin_threshold_reset = 50.0/180*M_PI;

    
    if (spin_norm > spin_threshold_reset*spin_threshold_reset){
        do_reset = true;
        ROS_INFO_STREAM_THROTTLE(1, "Spin Detected, Reset");
    }else if (spin_norm > spin_threshold*spin_threshold){
        // ROS_INFO_STREAM_THROTTLE(0.1, "Spin Detected, Skip this Recording(rad/s): " << spin_norm);
        return;
    }

    //// STEP 1: update buffer with the newest reading
    imu_buffer[next_idx] = *imu;
    next_idx++;


    if (next_idx != BUFFER_SIZE) 
    {
        accel[0] += imu->linear_acceleration.x;
        accel[1] += imu->linear_acceleration.y;
        accel[2] += imu->linear_acceleration.z;

        gyro[0] += imu->angular_velocity.x;
        gyro[1] += imu->angular_velocity.y;
        gyro[2] += imu->angular_velocity.z;

    }else{ // Buffer full, one cycle complete
        do_reset = true; 
        
        accel[0] /= BUFFER_SIZE;
        accel[1] /= BUFFER_SIZE;
        accel[2] /= BUFFER_SIZE;

        gyro[0] /= BUFFER_SIZE;
        gyro[1] /= BUFFER_SIZE;
        gyro[2] /= BUFFER_SIZE;

        if (accelLimits.xMax < accel[0]){
            accelLimits.xMax = accel[0];
            accelLimits.xMax_msg = accel;
            if (accelLimits.xMax > 8.8)
                accelLimits.valid.set(0);
        }
        if (accelLimits.xMin > accel[0]){
            accelLimits.xMin = accel[0];
            accelLimits.xMin_msg = accel;
            if (accelLimits.xMin < -8.8)
                accelLimits.valid.set(1);
        }
        if (accelLimits.yMax < accel[1]){
            accelLimits.yMax = accel[1];
            accelLimits.yMax_msg = accel;
            if (accelLimits.yMax > 8.8)
                accelLimits.valid.set(2);
        }
        if (accelLimits.yMin > accel[1]){
            accelLimits.yMin = accel[1];
            accelLimits.yMin_msg = accel;
            if (accelLimits.yMin < -8.8)
                accelLimits.valid.set(3);
        }
        if (accelLimits.zMax < accel[2]){
            accelLimits.zMax = accel[2];
            accelLimits.zMax_msg = accel;
            if (accelLimits.zMax > 8.8)
                accelLimits.valid.set(4);
        }
        if (accelLimits.zMin > accel[2]){
            accelLimits.zMin = accel[2];
            accelLimits.zMin_msg = accel;
            if (accelLimits.zMin < -8.8)
                accelLimits.valid.set(5);
        }

        printf("Current Accel avg: %lf, %lf, %lf\n",accel[0], accel[1], accel[2]);
        // printf("Gyro avg: %lf, %lf, %lf\n",gyro[0], gyro[1], gyro[2]);
        printf("Limit Accel avg: [%lf %lf], [%lf %lf], [%lf %lf]\n", 
            accelLimits.xMin,accelLimits.xMax,
            accelLimits.yMin,accelLimits.yMax,
            accelLimits.zMin,accelLimits.zMax);
        std::cout << "Valid Status: " << accelLimits.valid << std::endl;
    }

    if(do_reset){
        next_idx = 0;
        accel = std::vector<double>(3,0);
        gyro = std::vector<double>(3,0);
    // std::fill(accel,accel+3,0.0);
    // std::fill(gyro,gyro+3,0.0);
    }
    
}

std::string get_working_path()
{
   char temp[PATH_MAX];
   return ( getcwd(temp, PATH_MAX) ? std::string( temp ) : std::string("") );
}

void get_date_time_dash(std::string &date_time){
    std::time_t result = std::time(nullptr);
    struct tm * timeinfo = localtime(&result);

    char buffer_date[80];
    strftime(buffer_date,80,"imu_calib-%F-%H-%M-%S.yaml",timeinfo);
    date_time = std::string(buffer_date);
}

int main (int argc, char** argv){
    std::cout << "\nAccelerometer Scale and Bias Calibration Utility" << std::endl;
    auto cwd = get_working_path();
    std::cout << "Current Working Directory: " << cwd << std::endl;

    //// ROS
    ros::init(argc, argv, "offline_calibration");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber _imu_sub;
    


    // YAML::Node config = YAML::LoadFile("imu_calib.yaml");

    
    
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    std::cout << "\nAny key to start calibration..." << std::endl;
    std::cin.get();
    std::cout << "===============Started================" << std::endl;
    isCalibrating = true;
    _imu_sub = nh.subscribe("/imu0", 5, &imuCallback);

    std::cout << "Listening to topic " << "/imu0" << std::endl;

    std::cout << "\nAny key to terminate calibration..." << std::endl;
    std::cin.get();
    isCalibrating = false;
    spinner.stop();
    ros::Duration(0.5).sleep();

    if (accelLimits.valid.all()){
        printf("\n\n\nFinal Accel Limit X: [%lf %lf]\n", accelLimits.xMin,accelLimits.xMax);
        printf(" @ Attitude min: %lf %lf %lf\n", accelLimits.xMin_msg[0], accelLimits.xMin_msg[1], accelLimits.xMin_msg[2]);
        printf(" @ Attitude max: %lf %lf %lf\n", accelLimits.xMax_msg[0], accelLimits.xMax_msg[1], accelLimits.xMax_msg[2]);

        printf("Final Accel Limit Y: [%lf %lf]\n", accelLimits.yMin,accelLimits.yMax);
        printf(" @ Attitude min: %lf %lf %lf\n", accelLimits.yMin_msg[0], accelLimits.yMin_msg[1], accelLimits.yMin_msg[2]);
        printf(" @ Attitude max: %lf %lf %lf\n", accelLimits.yMax_msg[0], accelLimits.yMax_msg[1], accelLimits.yMax_msg[2]);

        printf("Final Accel Limit Z: [%lf %lf]\n", accelLimits.zMin,accelLimits.zMax);
        printf(" @ Attitude min: %lf %lf %lf\n", accelLimits.zMin_msg[0], accelLimits.zMin_msg[1], accelLimits.zMin_msg[2]);
        printf(" @ Attitude max: %lf %lf %lf\n", accelLimits.zMax_msg[0], accelLimits.zMax_msg[1], accelLimits.zMax_msg[2]);


        // The bias is the difference between the measured gravity and the reference
        double bias_x = (accelLimits.xMax + accelLimits.xMin)/2;
        double bias_y = (accelLimits.yMax + accelLimits.yMin)/2;
        double bias_z = (accelLimits.zMax + accelLimits.zMin)/2;


        // The scale factor is the ratio between measured gravity and the reference 9.8 m/s^2
        double scale_x = (accelLimits.xMax - accelLimits.xMin)/(9.8*2);
        double scale_y = (accelLimits.yMax - accelLimits.yMin)/(9.8*2);
        double scale_z = (accelLimits.zMax - accelLimits.zMin)/(9.8*2);

        assert ( scale_x>0 && scale_y>0 && scale_z>0);

        


        // YAML Output
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "scale" << YAML::Comment("The scale factor is the ratio between measured gravity and the reference 9.8 m/s^2");
        out << YAML::Value << YAML::BeginSeq << scale_x << YAML::Comment("x") << scale_y << YAML::Comment("y") << scale_z << YAML::Comment("z") << YAML::EndSeq;
        out << YAML::Key << "bias" << YAML::Comment("The bias is the difference between the measured gravity and the reference");
        out << YAML::Value << YAML::BeginSeq << bias_x << YAML::Comment("x") << bias_y << YAML::Comment("y") << bias_z << YAML::Comment("z") << YAML::EndSeq;
        out << YAML::EndMap;

        std::string temp;
        temp = "x-limits: [" + std::to_string(accelLimits.xMin) + ", " + std::to_string(accelLimits.xMax) + "]";
        out << YAML::Newline << YAML::Comment(temp);
        temp = "y-limits: [" + std::to_string(accelLimits.yMin) + ", " + std::to_string(accelLimits.yMax) + "]";
        out << YAML::Newline << YAML::Comment(temp);
        temp = "z-limits: [" + std::to_string(accelLimits.zMin) + ", " + std::to_string(accelLimits.zMax) + "]";
        out << YAML::Newline << YAML::Comment(temp);

        

        std::cout << out.c_str() << std::endl;
        
        std::ofstream fout;
        std::string filename;
        get_date_time_dash(filename);
        fout.open(filename);
        fout << out.c_str();
        fout.close();
        
    }else{
        std::cout << "Calibration Incomplete!\n" << std::endl;
    }

    ros::shutdown();
    return 0;
}