#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H
/**
 * @file dead_reckoning.h
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-21
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
// #include <Eigen/Geometry> 

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <cmath>

#include <ros/ros.h>

class DeadReckoning{
public:
    struct Parameters{
        Eigen::Affine3d g_project; // transformation from world-scale to imu-scale, in imu-coordinates
        Eigen::Vector3d g = {0, 0, 9.8}; // East-North-Up
        double hp_rc_time = 5;
    };

    enum IntegrationMode{
        GENERAL_MOTION,
        STATIONARY,
        ACCEL_SMALL,
        DOMINANT_DIRECTION
    };

    struct State{
        IntegrationMode integration_mode;

        Eigen::Vector3d velocity = {0,0,0};
        Eigen::Vector3d position = {0,0,0};
        Eigen::Vector3d accel = {0,0,0};
        ros::Time stamp = ros::Time(0);

        // Eigen::Vector3d accel_bias = {0,0,0};

        // bool valid = false;
    };
    
    DeadReckoning() = default;

    void setParams(const Parameters &params){
        this->params = params;
        std::cout << "DeadReckoning: Params Loaded. " << std::endl << params.g_project.matrix() << std::endl;
    }

    void setIntegrationMode(IntegrationMode mode){state.integration_mode = mode;}
    
    void process(const Eigen::Vector3d &accel_world, const ros::Time &stamp, double delta_t, const Eigen::Vector3d &dominant_velocity);
    nav_msgs::Odometry getOdometryMsg();
    sensor_msgs::Imu getAccelWorldMsg();
    // bool valid(){return state.valid;}
    void reset();

private:
    State state;
    Parameters params;

    void integrate(const Eigen::Vector3d &accel_world, double delta_t);
};


void DeadReckoning::process(const Eigen::Vector3d &accel_world,const ros::Time &stamp, double delta_t, const Eigen::Vector3d &dominant_velocity){
    
    state.stamp = stamp;

        // Estimate Global Accelerometer Drift
    static Eigen::Vector3d accel_low_pass_bias = {0,0,0};
    if (state.integration_mode == STATIONARY){     
        const double alpha = 0.999;
        accel_low_pass_bias = alpha*accel_low_pass_bias + (1.0-alpha)*accel_world; 
    }
    // std::cout << "accel_low_pass_bias = " << accel_low_pass_bias.transpose() << std::endl;

    state.accel = accel_world; // - accel_low_pass_bias;

    // Dominant Velocity and acceleration corrections
    if (state.integration_mode == STATIONARY)
    {
        // Trim velocity towards origin
        // state.velocity = (1.0/(1.0 + 10*delta_t))*state.velocity;

        // if (state.velocity.norm() < 0.1)
        //     state.velocity = {0,0,0};
        state.velocity = 1.0/(1.0 + delta_t)*state.velocity;
        // Set acceleration zero
        state.accel = {0,0,0};

    }else if (state.integration_mode == ACCEL_SMALL){
        // state.velocity = 1.0/(1.0 + delta_t)*state.velocity;
    }else if(state.integration_mode == DOMINANT_DIRECTION){
        // // // Trim velocity towards the dominant direction
        Eigen::Vector3d projected_velocity = dominant_velocity.normalized().dot(state.velocity)*dominant_velocity.normalized();
        double strength = 100.0/( 100.0 + dominant_velocity.norm());
        state.velocity = projected_velocity*(1-strength) + strength*state.velocity;

        // trim acceleration to only contain components in dominant_direction
        state.accel = dominant_velocity.normalized().dot(state.accel)*dominant_velocity.normalized();
        std::cout << "strength= " << strength << std::endl;
    }else if(state.integration_mode == GENERAL_MOTION);
        // empty
    else{
        std::cout  << "Uncaught dead reckoning integration mode" << std::endl;
    }

    state.velocity = state.velocity +  state.accel * delta_t;
    state.position += state.velocity * delta_t +   0.5 * state.accel * delta_t * delta_t;
}

void DeadReckoning::integrate(const Eigen::Vector3d &accel_world, double delta_t){

    // static Eigen::Vector3d pre_accel_world = {0,0,0};
    

    // double alpha = params.hp_rc_time / (params.hp_rc_time + delta_t);



    // STEP 1: Update bias estimation, only when accel is small;
    
    // hm: Ideally, this should happen when a period of zero velocity is detected
    // if (accel_world.norm() < 0.3 && state.velocity.norm() < 0.5 ){ 
    //     const double bias_gain = 0.1;
    //     double k = 1.0 / (1.0 +  bias_gain * delta_t);
    //     state.accel_bias = state.accel_bias*k + (1.0 - k)*accel_world;
    //     // std::cout << std::setprecision(3) << std::setw(8) << std::fixed << state.accel_bias.transpose() << " k=" << k << std::endl;
    // }

    state.accel = accel_world; //- state.accel_bias;

    // STEP 2: If acceleration is small, set it too zero (should already be taken cared of by step 1, but just for the transient sake)

    // if ( state.accel.norm() <  0.1)
    //     state.accel = Eigen::Vector3d::Zero();

    
    

    // if (state.accel.norm() < 0.1)
    //     state.accel = {0,0,0};

    // YOU CANT ADD *0.99 COEFFICIENT HERE
    // IF YOU ADD, THERE WILL BE VELOCITY OVERSHOOT, THAT COS POSITION TO KICK-BACK
    state.velocity = state.velocity +  state.accel * delta_t; 

    

    // ROS_INFO_STREAM_THROTTLE(0.1, std::setprecision(2) << std::setw(6) <<  std::fixed << (state.accel * delta_t * 200).transpose() );
    
    // auto velocity_new = state.velocity +  state.accel * delta_t;
    
    // state.velocity = alpha*state.velocity + alpha*(velocity_new - state.velocity);


    // auto position_new = state.position + state.velocity * delta_t +   0.5 * state.accel * delta_t * delta_t;
    // state.position = alpha*state.position +  alpha * (position_new - state.position);

    state.position += state.velocity * delta_t +   0.5 * state.accel * delta_t * delta_t;

    // pre_accel_world = accel_world;
}


nav_msgs::Odometry DeadReckoning::getOdometryMsg(){
    nav_msgs::Odometry odometry;

    odometry.header.stamp = state.stamp;
    odometry.header.frame_id = "world_frame";

    odometry.pose.pose.position.x =  state.position(0);
    odometry.pose.pose.position.y =  state.position(1);
    odometry.pose.pose.position.z =  state.position(2);

    odometry.twist.twist.linear.x = state.velocity(0);
    odometry.twist.twist.linear.y = state.velocity(1); 
    odometry.twist.twist.linear.z = state.velocity(2); 

    return odometry;
}

sensor_msgs::Imu DeadReckoning::getAccelWorldMsg(){
    sensor_msgs::Imu imu;

    imu.header.stamp = state.stamp;
    imu.header.frame_id = "world_frame";

    imu.linear_acceleration.x = state.accel(0);
    imu.linear_acceleration.y = state.accel(1);
    imu.linear_acceleration.z = state.accel(2);

    imu.orientation.w = 1;
    imu.orientation.x = imu.orientation.y = imu.orientation.z = 0;

    return imu;
}

void DeadReckoning::reset(){
    state.position = {0,0,0};
    state.velocity = {0,0,0};
    state.accel = {0,0,0};
    // state.accel_bias = {0,0,0};
}

#endif /* DEAD_RECKONING_H */
