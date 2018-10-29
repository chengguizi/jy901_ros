#ifndef VELOCITY_OBSERVER_H
#define VELOCITY_OBSERVER_H

/**
 * @file velocity_estimator.h
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-23
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <Eigen/Eigen>

#include <sensor_msgs/Imu.h>

#include <cassert>

#include <iostream>
#include <vector>

class VelocityObserver{

public:
    const int imu_buffer_size = 21; // this must be a odd number
    // latency = imu_current_idx + 1
    const int imu_current_idx = (imu_buffer_size) / 2; // zero-indexed

    enum EstimatedState{
        GENERAL_MOTION, 
        STATIONARY, // There is almost no acceleration (delta velocity change is zero)
        ACCEL_SMALL, // There is little acceleration, is this a drift?
        DOMINANT_DIRECTION // There appears to be a consistent dominant velocity direction   
    };

    struct State{
        EstimatedState state;
        Eigen::Vector3d dominant_velocity; // direction and magnitude
    };

    struct ImuBuffer{
        sensor_msgs::Imu::ConstPtr msg_ptr;
        Eigen::Vector3d delta_velocity;
        double delta_t;
        // double likelihood_term;
    };

    struct Parameters{
        double maximum_delta_t = 0.1;
        double high_pass_rc_time_v = 0.5; // https://en.wikipedia.org/wiki/High-pass_filter#Discrete-time_realization

        double stationary_threshold_ms = 0.12;
        double small_accel_threshold_ms = 0.3;
        double dominant_velocity_threshold_ms = 0.8;
    };

    VelocityObserver() : velocity_high_pass(Eigen::Vector3d::Zero()){
        imu_buffer.resize(imu_buffer_size, {nullptr,Eigen::Vector3d::Zero()});
    }

    void setParams(const Parameters& params) {
        this->params = params;
        std::cout << "VelocityEstimator: Parameters Loaded" << std::endl;
    }
    
    void push_back(const sensor_msgs::Imu::ConstPtr &imu_ptr); // accel readings should be after g removal

    void pop_current(sensor_msgs::Imu::ConstPtr &curr_imu_ptr, double &delta_t, State &state);

    // State getState(){ return state;}

private:

    Parameters params;

    std::vector<ImuBuffer> imu_buffer;

    // State state;

    Eigen::Vector3d velocity_high_pass;

};


void VelocityObserver::push_back(const sensor_msgs::Imu::ConstPtr &imu_ptr){

    //// STEP 0: Data Entry and Sanity Check

    // For each new imu readings, check the delta_t with the previous entry
    double delta_t;
    if (imu_buffer[imu_buffer_size-1].msg_ptr != nullptr)
        delta_t = imu_ptr->header.stamp.toSec() - imu_buffer[imu_buffer_size-1].msg_ptr->header.stamp.toSec();
    else
        delta_t = 0;

    assert(delta_t >= 0);

    // If the difference is too big, flush the buffer
    if ( delta_t > params.maximum_delta_t){
        // sensor_msgs::Imu::Ptr zero_imu_ptr(new sensor_msgs::Imu());
        // zero_imu_ptr->header = imu_ptr->header;
        // zero_imu_ptr->linear_acceleration = {0,0,0};
        // imu_buffer.push_back(zero_imu_ptr);

        imu_buffer.clear();
        imu_buffer.resize(imu_buffer_size, {nullptr,Eigen::Vector3d::Zero(), 0});
        // reset high pass filter too
        velocity_high_pass = {0,0,0};

        std::cout << "Detect big delta_t, use zero acceleration: dt = " << delta_t << std::endl;
    }

    // Pop the first entry to discard
    imu_buffer.erase(imu_buffer.begin());

    // Calculate delta velocity
    const Eigen::Vector3d accel = {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z};
    Eigen::Vector3d delta_velocity = accel * delta_t;

    // Calculate likelihood term for stationary condition estimation
    // const Eigen::Vector3d gyro = { imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z };
    // double likelihood_term = accel.squaredNorm()/params.accel_variance + gyro.squaredNorm()/params.gyro_variance;
    // Add the new reading into the last entry
    imu_buffer.push_back({imu_ptr, delta_velocity, delta_t});

    //// STEP 1: High Pass Calculation;
    const double alpha = params.high_pass_rc_time_v / (params.high_pass_rc_time_v +  delta_t);

    // Eigen::Vector3d accel = {imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z};

    velocity_high_pass = alpha*velocity_high_pass + alpha*(delta_velocity);
}

void VelocityObserver::pop_current(sensor_msgs::Imu::ConstPtr &curr_imu_ptr, double &delta_t, State &state){

    // If buffer not full, return nullptr
    if (imu_buffer[0].msg_ptr == nullptr){
        curr_imu_ptr = nullptr;
        return;
    }
        
    // auto prev_imu_ptr = imu_buffer[imu_current_idx-1].msg_ptr;
    curr_imu_ptr = imu_buffer[imu_current_idx].msg_ptr;
    
    delta_t = imu_buffer[imu_current_idx].delta_t; // curr_imu_ptr->header.stamp.toSec() - prev_imu_ptr->header.stamp.toSec();
    assert (delta_t > 0);
    

    // Calculate average velocity

    Eigen::Vector3d avg_delta_velocity = {0,0,0};
    for (auto imu : imu_buffer){
        avg_delta_velocity += imu.delta_velocity;
    }
    double span_t = imu_buffer[imu_buffer_size - 1].msg_ptr->header.stamp.toSec() - imu_buffer[0].msg_ptr->header.stamp.toSec();
    // HERE, avg_delta_velocity contains accelerometer bias, could be minus off if we could estimate the bias
    avg_delta_velocity /= span_t; // velocity change per second
    std::cout << std::setprecision(3) << std::setw(8) << std::fixed;
    std::cout << "velocity_high_pass = " << velocity_high_pass.norm();
    std::cout << " avg_delta_velocity = " << avg_delta_velocity.norm() << std::endl;

    // Calculate variance
    double variance = 0;
    for (auto imu : imu_buffer){
        variance += (imu.delta_velocity / delta_t - avg_delta_velocity).squaredNorm();
    }

    std::cout << "variance = " << variance << std::endl;

    // Determine the state
    state.state = GENERAL_MOTION;

    if (avg_delta_velocity.norm() <= params.stationary_threshold_ms){
        state.state = STATIONARY;
        
    }else if(avg_delta_velocity.norm() <= params.small_accel_threshold_ms){
        state.state = ACCEL_SMALL;
    }else if (avg_delta_velocity.norm() > params.dominant_velocity_threshold_ms){
        state.state = DOMINANT_DIRECTION;
    }

    state.dominant_velocity = avg_delta_velocity;
}

#endif /* VELOCITY_OBSERVER_H */
