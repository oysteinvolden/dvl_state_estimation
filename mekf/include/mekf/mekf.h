#ifndef MEKF_H_
#define MEKF_H_

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <iostream>
#include <math.h>
#include <numeric>
#include "buffer.hpp"
#include "common.h"


namespace mekf{

    class MEKF{

        public:

            static constexpr int k_num_states_ = 15; // Not 16 since we parametrize unit quaternion with three parameters (error state)

            MEKF();

            void updateDVL(const vec3& dvl_vel, const bool& velocity_valid, uint64_t time_usec);

            void updateCamPose(const vec3& cam_pos, const quat& cam_quat, uint64_t time_usec);
	        void updateSbgNav(const vec3& sbg_pos, const vec3& sbg_vel, uint64_t time_usec);
            void updateSbgQuat(const quat& sbg_quat, uint64_t time_usec);

            void run_mekf(const vec3& ang_vel, const vec3& lin_acc, uint64_t time_us, double dt);
	 
            quat getQuat();
            vec3 getPosition();
            vec3 getVelocity();
            uint64_t getImuTime(); 

            // smallest signed angle
            double ssa(double angle);

            // customized modulo function
            double mod(double x, double y);

            // sampling constants
            double h; // sampling time [s]
            double dvl_dt; // sampling dvl time

        private:
            
            // state
            state state_;
            
            // FIFO buffers
            Buffer<cameraPoseSample> camPoseBuffer_;
            Buffer<sbgNavSample> sbgNavBuffer_;
            Buffer<sbgQuatSample> sbgQuatBuffer_;

            Buffer<imuSample> imuBuffer_;
            Buffer<dvlSample> dvlBuffer_;
            
            // FIFO buffer lengths 
            const int cam_buffer_length_ {9};  // TODO: check lengths
            const int dvl_buffer_length_ {4};  // TODO: check lengths
            const int sbg_nav_buffer_length_ {6};  // TODO: check lengths
            const int sbg_quat_buffer_length_ {4};  // TODO: check lengths

            const int imu_buffer_length_ {15}; // TODO: check lengths

            // new samples
            imuSample imu_sample_new_ {};  // imu sample capturing the newest imu data
            dvlSample dvl_sample_new_ {}; // dvl sample capturing the newest dvl data
            cameraPoseSample cam_pose_sample_new_ {}; // cam sample capturing the newest vision data
            sbgNavSample sbg_nav_sample_new_ {}; // SBG nav data capturing the newest INS data
            sbgQuatSample sbg_quat_sample_new_ {}; // SBG quat data capturing the newest INS data

            // delayed samples
            imuSample imu_sample_delayed_ {};	// captures the imu sample on the delayed time horizon
            dvlSample dvl_sample_delayed_ {};   // captures the dvl sample on the delayed time horizon
            cameraPoseSample cam_pose_delayed_ {}; // captures the cam pose sample on the delayed time horizon
            sbgNavSample sbg_nav_delayed_ {};
            
            // flags on received updates
            bool dvl_ready_ = false;
            bool sbg_ready_ = false;
            bool cam_pose_ready_ = false;

            // timing
            uint64_t time_last_dvl_ {0};
            uint64_t time_last_cam_pose_ {0};
            uint64_t time_last_sbg_pos_ {0};
            uint64_t time_last_sbg_quat_ {0};

            uint64_t time_last_imu_ {0};
            uint64_t current_cam_pose_time = {0}; 

            // filter initalization
            bool initializeFilter();
            bool filter_initialised_; // true when initialized 

            // gravity constants
            const double mu = 63.4396 * (M_PI/180); // lattitude (Trondheim brattøra)
            double g;
            vec3 g_n;
            double gravity(double lattitude);
            
            // Bias time constants (user specified)
            const double T_acc = 1000; 
            const double T_gyro = 1000; 

            // Covariance and predictor
            Eigen::Matrix<double, k_num_states_, k_num_states_> P_prd, P_hat; 

            // process noise weights: v, acc_bias, w, ars_bias
            Eigen::Matrix<double, 12, 12> Qd, Q; 

            // measurement noise - position aiding + compass 
            Eigen::Matrix<double, 10, 10> Rd; // CAM + DVL
      
            // constant matrices
            Eigen::Matrix<double, 3, 3> O3, I3;
            Eigen::Matrix<double, 15, 15> I15;
            Eigen::Matrix<double, 1, 3> O_13;
            Eigen::Matrix<double, 1, 9> O_19;

            // reference vector
            vec3 v01;

            // Rotation matrix
            Eigen::Matrix<double,3,3> R;

            // Discrete-time KF matrices
            Eigen::Matrix<double, 15, 15> A, Ad;
            Eigen::Matrix<double, 10, 15> Cd; // DVL + CAM
            Eigen::Matrix<double, 15, 12> E, Ed;

            // kalman gain
            Eigen::Matrix<double, 15, 10> K; // DVL + CAM
            Eigen::Matrix<double, 15, 15> IKC;

            // epsilon matrix
            Eigen::Matrix<double, 10, 1> eps; // DVL + SBG

            // estimated error state
            Eigen::Matrix<double, 15, 1> delta_x_hat;
             
            // SBG INS measurements
            vec3 sbg_pos;
            vec3 sbg_vel;
            quat sbg_quat;

            // cam measurement / eucliden distance
            double d_cam;
            vec3 cam_pos;
            double roll_cam, pitch_cam, yaw_cam; 

            // DVL measurements
            vec3 dvl_vel, dvl_vel_NED;

            // IMU counter 
            int counter = 0;

            // count the number of accepted camera measurements
            cam_accept_count = 0;

    };

}


#endif /* defined(MEKF_H_) */
