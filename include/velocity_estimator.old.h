#ifndef ZERO_VELOCITY_ESTIMATOR_H
#define ZERO_VELOCITY_ESTIMATOR_H
/**
 * @file zero_velocity_estimator.h
 * @author Huimin Cheng (NUS)
 * @brief 
 * @version 0.1
 * @date 2018-10-23
 * 
 * @copyright Copyright (c) 2018
 * 
 */

#include <Eigen/Eigen>

#include <cassert>

#include <iostream>
#include <vector>

class VelocityEstimator{

public:
    const int imu_buffer_size = 11; // this must be a odd number
    const int imu_current_idx = (imu_buffer_size + 1) / 2;
    enum EstimatedState{
        GENERAL_MOTION, 
        STATIONERY, // There is almost no acceleration (delta velocity change is zero)
        ACCEL_FREE, // There is little acceleration, is this a drift?
        DOMINANT_DIRECTION // There appears to be a consistent dominant velocity direction   
    };

    struct Parameters{
        int N = 10; // window size
        int H = 16; // hysteresis memory length

        double threshold = 2; // from experiment

        double settling_ratio = 0.3;

        double accel_variance = 0.005;
        double gyro_variance = 0.01;

        double hysteresis_decay = 0.9;
        double hysteresis_threshold = 0.5*0.5; // (m/s)^2
    };

    VelocityEstimator(){
        buffer.resize(params.N);
        velocity_buff.resize(params.N);
        hysteresis_buff.resize(params.H);
    }

    void setParams(const Parameters& params) {
        this->params = params;
        std::cout << "ZeroVelocityEstimator: Parameters Loaded" << std::endl;

        buffer.resize(params.N);
        velocity_buff.resize(params.N, {0,0,0});
        hysteresis_buff.resize(params.H, 0);
    }

    void push_back(const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, Eigen::Vector3d &velocity); // accel readings should be after g removal

    EstimatedState getState();

private:

    Parameters params;

    std::vector<double> buffer;

    std::vector<double> hysteresis_buff;

    std::vector<Eigen::Vector3d> velocity_buff;

    EstimatedState state;

};


void ZeroVelocityEstimator::push_back(const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro, Eigen::Vector3d &velocity){

    static int data_idx = 0;
    static double data = 0;

    static int zero_count = 0;

    // putting data term into the sum;
    data += accel.squaredNorm()/params.accel_variance + gyro.squaredNorm()/params.gyro_variance;

    velocity_buff[data_idx] = velocity;

    const double alpha = 0.9;
    static Eigen::Vector3d velocity_hp = {0,0,0};

    static Eigen::Vector3d prev_velocity;

    velocity_hp = velocity_hp*alpha + alpha*(velocity - prev_velocity);
    prev_velocity = velocity;

    // std::cout << accel.squaredNorm() << " " << gyro.squaredNorm() << std::endl;

    // increment the data_index for the next round
    data_idx++;

    // condition for likelihood calculation
    if (data_idx == params.N){

        // averaging
        data /= params.N;

        // debug

        std::cout << "data = " << data << std::endl;

        // update hysteresis
        double vel_squared = velocity.squaredNorm();
        
        double hysteresis = std::max(hysteresis_buff[0], vel_squared);

        for (size_t i = 0; i < hysteresis_buff.size() - 1; i++){

            hysteresis_buff[i] = (hysteresis_buff[i+1] < vel_squared) ? vel_squared : hysteresis_buff[i+1];
        }

        hysteresis_buff[params.H - 1] = vel_squared;



        std::cout << "vel = " << vel_squared << ", hysteresis = " << hysteresis << std::endl;

        EstimatedState current = MOVING;

        // double dynamic_thres = params.threshold * (1 + 2*(state == MOVING));
        // if (data <  dynamic_thres ){

        //     std::cout << "ZERO VELOCITY BY SMALL ACCEL&GYRO, thres = " << dynamic_thres << std::endl;

        //     current = (EstimatedState) (current | ZERO_VELOCITY_BY_ACCEL_GYRO) ;
        // }

        // Detect a settling trend in velocity

        // std::cout << "velocity_hp.norm() = " << velocity_hp.norm() << std::endl;

        // if ( hysteresis > 0.1 && velocity_hp.norm() < 0.01 && vel_squared / hysteresis < params.settling_ratio ){


        //     double ratio = vel_squared / hysteresis;
        //     // still, we need to ensure the acceleration is small enough

        //         std::cout << "ZERO VELOCITY BY SETTLING VELOCITY" << std::endl;
        //         current = (EstimatedState) (current |ZERO_VELOCITY_BY_VEL );
        // }


        // A simple filter
        // if (current != MOVING)
        //     zero_count++;
        // else
        //     zero_count = 0;

        // if (zero_count > 1)
        //     state = current;
        // else
        //     state = MOVING;

        state = current;

        if (state != MOVING){
            hysteresis_buff.clear();
            hysteresis_buff.resize(params.H, 0);
        }



        // reset index
        data_idx = 0;
        data = 0;
        velocity_buff.clear();
        velocity_buff.resize(params.N, {0,0,0});
    }
}

ZeroVelocityEstimator::EstimatedState ZeroVelocityEstimator::getState(){
    return state;
}


#endif /* ZERO_VELOCITY_ESTIMATOR_H */
