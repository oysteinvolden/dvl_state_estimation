#include <mekf/mekf.h>

// libraries for file streams
#include <iostream>
#include <fstream>
#include <sstream>

// log IMU bias 
const char *path_log_imu_bias="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/imu_bias/imu_bias.txt";
std::ofstream log_imu_bias(path_log_imu_bias);

// log velocity data
const char *path_log_dvl_velocity="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/velocity/velocity_dvl.txt";
std::ofstream log_dvl_velocity(path_log_dvl_velocity);
const char *path_log_sbg_velocity="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/velocity/velocity_sbg.txt";
std::ofstream log_sbg_velocity(path_log_sbg_velocity);
const char *path_log_ins_velocity="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/velocity/velocity_ins.txt";
std::ofstream log_ins_velocity(path_log_ins_velocity);

// log position / heading
const char *path_log_sbg_pos_heading="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/pos_heading/pos_heading_sbg.txt";
std::ofstream log_sbg_pos_headig(path_log_sbg_pos_heading);
const char *path_log_dvl_pos_heading="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/pos_heading/pos_heading_dvl.txt";
std::ofstream log_dvl_pos_heading(path_log_dvl_pos_heading);
const char *path_log_dvl_pos="/home/oysteinvolden/mekf_ws_dvl/logging/2022-05-19-13-43-48/pos_heading/pos_dvl.txt";
std::ofstream log_dvl_pos(path_log_dvl_pos);


namespace mekf{

    MEKF::MEKF(){
      
        // initialize INS        
        
	    state_.quat_nominal = quat(1,0,0,0);
        state_.vel = vec3(0,0,0);
        state_.pos = vec3(0,0,0);
        state_.gyro_bias = vec3(0,0,0);
        state_.accel_bias = vec3(0,0,0);
        
	    
         
        // allocate imu buffer
        imuBuffer_.allocate(imu_buffer_length_);
        for (int i = 0; i < imu_buffer_length_; i++){
            imuSample imu_sample_init = {};
            imuBuffer_.push(imu_sample_init);
        }

        // allocate dvl buffer
        dvlBuffer_.allocate(dvl_buffer_length_);
        for (int i = 0; i < dvl_buffer_length_; i++){
            dvlSample dvl_sample_init = {};
            dvlBuffer_.push(dvl_sample_init);
        }
        
        // allocate camera pose buffer
        camPoseBuffer_.allocate(cam_buffer_length_);
        for (int i = 0; i < cam_buffer_length_; i++) {
            cameraPoseSample camera_pose_sample_init = {};
            camPoseBuffer_.push(camera_pose_sample_init);
        }


       // allocate INS pos buffer
        sbgNavBuffer_.allocate(sbg_nav_buffer_length_);
        for (int i = 0; i < sbg_nav_buffer_length_; i++) {
            sbgNavSample sbg_nav_sample_init = {};
            sbgNavBuffer_.push(sbg_nav_sample_init);
        }

        // allocate INS quat buffer
        sbgQuatBuffer_.allocate(sbg_quat_buffer_length_);
        for (int i = 0; i < sbg_quat_buffer_length_; i++) {
            sbgQuatSample sbg_quat_sample_init = {};
            sbgQuatBuffer_.push(sbg_quat_sample_init);
        }
        
        // set to false, so we can initialize
        filter_initialised_ = false;

        // set to false until we have a valid MEKF estimate to publish
        publish_MEKF_ = false;

    }


    bool MEKF::initializeFilter(){

        // *** initializa covariance matrices ***

        P_prd.setIdentity(k_num_states_,k_num_states_);

        // put the squares of the typical values of the the state components + margin

        /*
        P_prd.diagonal() << 0.000016, 0.000016, 0.000016, // delta pos in NED [m] 
                            0.000016, 0.000016, 0.000016, // delta velocity in NED [m/s]
                            0.000003,   0.000003,   0.000003, // delta acceleration bias [m/s^2]  
                            0.00000005,  0.00000005,  0.00000005, // delta a 
                            0.00000025, 0.00000025, 0.00000025; // delta gyro bias [rad]
        */
        
        
        P_prd.diagonal() << //10*4*10e-6, 10*4*10e-6, 10*4*10e-6, // delta pos in NED [m] 
                            4*10e-6, 4*10e-6, 4*10e-6, // delta pos in NED [m] // TEST
                            //10*4*10e-6, 10*4*10e-6, 10*4*10e-6,
                            4*10e-6, 4*10e-6, 4*10e-6, // delta velocity in NED [m/s] // TEST
                            //0.000003,   0.000003,   0.000003, // delta acceleration bias [m/s^2]  
                            //0,0,0,
                            pow(0.00015,2), pow(0.0002,2), pow(0.0008,2), // delta acceleration bias [m/s^2]  // TEST
                            //0.00009,   0.00001,   0.00001, // delta acceleration bias [m/s^2] // TEST 
                            0.00000005,  0.00000005,  0.00000005, // delta a 
                            0.00000025, 0.00000025, 0.00000025; // delta gyro bias [rad]
        

        //P_prd = 0.001 * P_prd; 
        
        
        P_prd = 0.01 * P_prd; 
        


        //P_prd = 5 * P_prd; 
        

        /*
        P_prd.diagonal() << 0.06, 0.06, 0.06, // delta pos [m]
                            0.06, 0.06, 0.06, // delta velocity [m/s]
                            0.0025, 0.0001, 0.0001, // delta acceleration bias [m/s^2]  
                            0.0001, 0.0001, 0.0001, // delta a 
                            0.0001, 0.0001, 0.0001; // delta gyro bias [rad]
        */


        // ADIS imu data
    
        double sigma_w_acc = 0.0001333; // velocity random walk
        double sigma_b_acc = 8.3212184588*10e-7; // velocity in run instability bias

        //double sigma_w_acc = 10e-15; // velocity random walk
        //double sigma_b_acc = 10e-15; // velocity in run instability bias

        double sigma_w_ars = 0.000026179938; // angular random walk
        double sigma_b_ars = 6.54728501*10e-8; // angular in run instability bias 
        

        // initialize process weights - v, acc_bias, w, gyro_bias (v, acc_bias, w, ars_bias)        
        Qd.diagonal() << sigma_w_acc, sigma_w_acc, sigma_w_acc,
                         sigma_b_acc, sigma_b_acc, sigma_b_acc,
                         //0.1*sigma_w_acc, 0.1*sigma_w_acc, 0.1*sigma_w_acc,
                         //0.1*sigma_b_acc, 0.1*sigma_b_acc, 0.1*sigma_b_acc,
                         sigma_w_ars, sigma_w_ars, sigma_w_ars,
                         sigma_b_ars, sigma_b_ars, sigma_b_ars;
                         //0.1*sigma_w_ars, 0.1*sigma_w_ars, 0.1*sigma_w_ars,
                         //0.1*sigma_b_ars, 0.1*sigma_b_ars, 0.1*sigma_b_ars;
        
        //Qd = 3 * h * Qd; // SBG
        Qd = 1000 * h * Qd; // DVL

        /*
        Qd.diagonal() << 0.01, 0.01, 0.01,
         0.01, 0.01, 0.01,
         0.1, 0.1, 0.1,
         0.001, 0.001, 0.001;
        */
        
 
        // this one is good
        /*
        Rd.diagonal() << 0.001, 0.001, 0.001, // p
                         0.001, 0.001, 0.001, // acc
                         0.0001;       // psi                 
        */

        // DVL + SBG
        
        Rd.diagonal() << 0.001, 0.001, 0.001, // p
                         //0,0,0, // pos - assume perfect position measurements
                         //0,0,0, // vel - assume perfect velocity measurements
                         0.001, 0.001, 0.001, // vel
                         0.001, 0.001, 0.001, // acc
                         0.0001;       // psi   
                         //0.00001;       // psi   
        

        //Rd.diagonal() << 0.001, 0.001, 0.001; // vel
        
        //Rd.diagonal() << 0.001, 0.002, 0.001; // vel (used this for DVL)

        //Rd.diagonal() << 0.001, 0.001, 0.001, // pos
        //                 0.001, 0.001, 0.001; // vel
  

        //Rd = 0.001*Rd; // adis imu

        //Rd = 0.0005*Rd; // adis imu
        
        
        //Rd = 0.0001*Rd; // adis imu
        
        
        //Rd = 0.00005*Rd; // adis imu
        
        Rd = 0.00001*Rd; // adis imu
        
        //Rd = 0.000001*Rd; // adis imu
        

        // %%%%% initialize state %%%%

	
        // extract SBG nav sample
        sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
        sbg_pos = sbg_nav_newest.posNED;
	    sbg_vel = sbg_nav_newest.vel; 

        // extract velocity measurment (DVL)
        dvlSample dvl_data_newest = dvlBuffer_.get_newest();
        y_vel = dvl_data_newest.vel;

        vel_integrated = dvl_data_newest.vel;
        //vel_integrated = sbg_nav_newest.vel;

        dvl_pos_integrated = sbg_nav_newest.posNED;
    
        // check is SBG sample has arrived (and DVL?)
        if( (sbg_pos.x() == 0 && sbg_pos.y() == 0 && sbg_pos.z() == 0) || (y_vel.x() == 0 && y_vel.y() == 0) ){
            return false;
        }
        else
        {
            // extract sbg quat
            sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
            sbg_quat = sbg_quat_newest.quatNED;

            //std::cout << "sbg quat: " << std::endl;
            //std::cout << sbg_quat.w() << " " << sbg_quat.x() << " " << sbg_quat.y() << " " << sbg_quat.z() << " " << std::endl;

            
            double quat_norm_sbg = sqrt ( pow(sbg_quat.w(),2) + pow(sbg_quat.x(),2) + pow(sbg_quat.y(),2) + pow(sbg_quat.z(),2) ) ;

            if( !(quat_norm_sbg >= 0.9999 && quat_norm_sbg <= 1.0001) ){
            //if( !(quat_norm_sbg <= 1) ){
                std::cout << "SBG quat not properly normalized INIT" << std::endl;
                return false;
            }           

            //sbg_quat.normalize();

            // convert DVL from body to NED
            // Rotation matrix 
            R = sbg_quat.toRotationMatrix(); // from body to NED?

            //y_vel_NED = R * y_vel;

            // in NED
            dvl_vel_integrated = R * y_vel;


            state_.quat_nominal = sbg_quat;
            //state_.quat_nominal = quat(1,0,0,0);
            //state_.vel = y_vel_NED; 
            state_.vel = sbg_vel;
            //state_.vel = vec3(0,0,0); 
            state_.pos = sbg_pos;

            //state_.gyro_bias = vec3(0,0,0); 
            //state_.accel_bias = vec3(0,0,0);

            // after estimated bias -> initialize to avoid drift
            state_.gyro_bias = vec3(0,0,0); 

            //state_.accel_bias = vec3(-0.0533,0.0666,-0.016);
            //state_.accel_bias = vec3(-0.1034,0.1171,-0.016);

            //state_.accel_bias = vec3(-0.1991,0.50,-0.016);

            //state_.accel_bias = vec3(-0.1991,0.3335,-0.016);

            //state_.accel_bias = vec3(-0.057,0.03,-0.01);
            //state_.accel_bias = vec3(-0.065,-0.032,-0.016);

            //state_.accel_bias = vec3(-0.03258,0.6512,-0.043);
            //state_.accel_bias = vec3(-0.3306,-0.0265,-0.0265);

            state_.accel_bias = vec3(-0.06,-0.03,0);
            //state_.accel_bias = vec3(-0.06,0,0);

            return true;
        }
	
        //return true;
        


    }

    void MEKF::updateDVL(const vec3& dvl_vel, const bool& velocity_valid, uint64_t time_usec){

       
        dvl_sample_new_.vel = dvl_vel;
        dvl_sample_new_.velocity_valid = velocity_valid;
        dvl_sample_new_.time_us = time_usec;

        // update last time
        time_last_dvl_ = time_usec;

        // push dvl data to buffer
        dvlBuffer_.push(dvl_sample_new_);


    }

    
    void MEKF::updateCamPose(const vec3& cam_pos, const quat& cam_quat, uint64_t time_usec){

        // copy required data
        cam_pose_sample_new_.posNED = cam_pos;
        cam_pose_sample_new_.quatNED = cam_quat;
        cam_pose_sample_new_.posErr = 0.05; // TODO: check later
        cam_pose_sample_new_.angErr = 0.05; // TODO: check later
        cam_pose_sample_new_.time_us = time_usec; // TODO: do we need to subtract camera delay here?

        // update last time
        time_last_cam_pose_ = time_usec;

        // push pose to buffer
        camPoseBuffer_.push(cam_pose_sample_new_);

    }
    


    void MEKF::updateSbgNav(const vec3& sbg_pos, const vec3& sbg_vel, uint64_t time_usec){

        // copy required data
        sbg_nav_sample_new_.posNED = sbg_pos;
	    sbg_nav_sample_new_.vel = sbg_vel;
        sbg_nav_sample_new_.time_us = time_usec; // TODO: do we need to subtract delay here?

        // update last time
        time_last_sbg_pos_ = time_usec;

        // push pose to buffer
        sbgNavBuffer_.push(sbg_nav_sample_new_); 

    }

    void MEKF::updateSbgQuat(const quat& sbg_quat, uint64_t time_usec){

        // copy required data
        sbg_quat_sample_new_.quatNED = sbg_quat;
        sbg_quat_sample_new_.time_us = time_usec; // TODO: do we need to subtract delay here?

        // update last time
        time_last_sbg_quat_ = time_usec;

        // push pose to buffer
        sbgQuatBuffer_.push(sbg_quat_sample_new_); 

    }




    void MEKF::run_mekf(const vec3& ang_vel, const vec3& lin_acc, uint64_t time_us, double dt){

        
        // check if SBG is valid (i.e., normalized quaternion)

        sbgQuatSample sbg_quat_sample = sbgQuatBuffer_.get_newest();
        sbg_quat = sbg_quat_sample.quatNED; 

        double quat_norm_sbg = sqrt ( pow(sbg_quat.w(),2) + pow(sbg_quat.x(),2) + pow(sbg_quat.y(),2) + pow(sbg_quat.z(),2) ) ;

        
        //quat q_ins_init = state_.quat_nominal;

        //double quat_norm_sbg = sqrt ( pow(q_ins_init.w(),2) + pow(q_ins_init.x(),2) + pow(q_ins_init.y(),2) + pow(q_ins_init.z(),2) ) ;


        // TODO: check INS instead?
        
        if( !(quat_norm_sbg >= 0.9999 && quat_norm_sbg <= 1.0001) ){ 
            std::cout << "SBG quat not properly normalized" << std::endl;
            return;
        } 
        
          


        // copy imu data
        imu_sample_new_.delta_ang = vec3(ang_vel.x(), ang_vel.y(), ang_vel.z()); // current yaw rate [rad/s] // TODO: change name
        imu_sample_new_.delta_vel = vec3(lin_acc.x(), lin_acc.y(), lin_acc.z()); // current linear acceleration [m/s^2]

        imu_sample_new_.delta_ang_dt = dt;
        imu_sample_new_.delta_vel_dt = dt;
        imu_sample_new_.time_us = time_us;
        time_last_imu_ = time_us; // update last time

        // push to buffer
        imuBuffer_.push(imu_sample_new_); 

        // get the oldest data from the buffer
        imu_sample_delayed_ = imuBuffer_.get_oldest(); // TODO: neccessary or can we use imu_sample_new directly?
        //imu_sample_delayed_ = imuBuffer_.get_newest();


        // *** TEST validity of IMU measurements ***

        
	    if( (abs(imu_sample_delayed_.delta_ang.x()) < 0.00001) || (abs(imu_sample_delayed_.delta_ang.y()) < 0.00001) || (abs(imu_sample_delayed_.delta_ang.z()) < 0.00001) ){
            std::cout << "gyro invalid" << std::endl;
	        return;
        }


	    if( (abs(imu_sample_delayed_.delta_vel.x()) < 0.00001) || (abs(imu_sample_delayed_.delta_vel.y()) < 0.00001) || (abs(imu_sample_delayed_.delta_vel.z()) < 0.00001) ){
		    std::cout << "acc invalid" << std::endl;
 	        return;
        }
        
        
        
        
        



        // check if filter is initialized
        if (!filter_initialised_) {
            filter_initialised_ = initializeFilter();
            if (!filter_initialised_) {
                return;
            }
        }


        if(counter_imu > 0){

            //double lin_acc_x = lin_acc.x() - 0.05; //- 0.1; // - 0.065; // bias compansated
            double lin_acc_x = lin_acc.x() - 0.06; //- 0.1; // - 0.065; // bias compansated
            double lin_acc_y = lin_acc.y() - 0.03; // + 0.03; //- 0.1; // - 0.032; // bias compansated

            vec3 lin_acc_copy = vec3(lin_acc_x, lin_acc_y, lin_acc.z());

            vel_integrated = vel_integrated + h*lin_acc_copy;

        }
        else
        {
            double lin_acc_x = lin_acc.x(); // - 0.065; // bias compansated
            double lin_acc_y = lin_acc.y(); // + 0.05; // - 0.032; // bias compansated

            vec3 lin_acc_copy = vec3(lin_acc_x, lin_acc_y, lin_acc.z());

            vel_integrated = vel_integrated + h*lin_acc_copy;
        }
        


        counter_imu++;

       



        // logging IMU velocity data (in BODY)
        //trace_id_ins = 0;

        /*
        if(trace_id_ins > 0){
            vel_integrated.x() = vel_integrated.x() - 0.0005;
        }
        else
        {
            std::cout << "******************* TEST ****************" << std::endl;
        }
        */

        trace_id_ins++;
        

        log_ins_velocity << std::fixed;
        log_ins_velocity << std::setprecision(16);
        log_ins_velocity << 1 << " " << time_us*10e-7 << " " << vel_integrated.x() << " " << vel_integrated.y() << " " << vel_integrated.z() << std::endl;

        if(trace_id_ins > 30000){
            log_ins_velocity.close();
        } 

                   




        // %%% KF states and matrices %%%

        // % INS states
        vec3 p_ins = state_.pos;
        vec3 v_ins = state_.vel;
        vec3 acc_bias_ins = state_.accel_bias;
        quat q_ins = state_.quat_nominal;
        vec3 gyro_bias_ins = state_.gyro_bias;

        // % WGS-84 gravity model
        g = gravity(mu);
        g_n << 0, 0, g;
       
        // Constants
        O3.setZero(3,3);
        O_13.setZero(1,3);
        O_19.setZero(1,9);
        I3.setIdentity(3,3);
        I15.setIdentity(15,15);

        // Reference vector
        v01 << 0, 0, 1; 

        // Rotation matrix 
        R = q_ins.toRotationMatrix(); // from body to NED?

        // Bias compensated IMU measurements
        vec3 f_ins = imu_sample_delayed_.delta_vel - acc_bias_ins; 
        vec3 w_ins = imu_sample_delayed_.delta_ang - gyro_bias_ins;


        // * Discrete-time KF matrices *

        // TODO: move the static parts of the matrices to initialization?

        A.block(0,0,3,3) = O3;
        A.block(0,3,3,3) = I3;
        A.block(0,6,3,3) = O3;
        A.block(0,9,3,3) = O3;
        A.block(0,12,3,3) = O3;

        A.block(3,0,3,3) = O3;
        A.block(3,3,3,3) = O3;
        A.block(3,6,3,3) = -R;
        A.block(3,9,3,3) = -R*Smtrx(f_ins);
        A.block(3,12,3,3) = O3;

        // alternative - constant velocity
        //A.block(3,0,3,3) = O3;
        //A.block(3,3,3,3) = O3;
        //A.block(3,6,3,3) = O3;
        //A.block(3,9,3,3) = O3;
        //A.block(3,12,3,3) = O3;

        A.block(6,0,3,3) = O3;
        A.block(6,3,3,3) = O3;
        A.block(6,6,3,3) = -(1/T_acc)*I3;
        A.block(6,9,3,3) = O3;
        A.block(6,12,3,3) = O3;

        A.block(9,0,3,3) = O3;
        A.block(9,3,3,3) = O3;
        A.block(9,6,3,3) = O3;
        A.block(9,9,3,3) = -Smtrx(w_ins);
        A.block(9,12,3,3) = -I3; 

        A.block(12,0,3,3) = O3;
        A.block(12,3,3,3) = O3;
        A.block(12,6,3,3) = O3;
        A.block(12,9,3,3) = O3; 
        A.block(12,12,3,3) = -(1/T_gyro)*I3;

   

        Ad =  I15 + h*A + 0.5*(h*A)*(h*A); 

        // alternative - constant velocity
        Ad.block(3,0,3,3) = O3;
        Ad.block(3,3,3,3) = O3;
        Ad.block(3,6,3,3) = O3;
        Ad.block(3,9,3,3) = O3;
        Ad.block(3,12,3,3) = O3;
 
      
        // linearization of heading measurement
        vec3 a = (2/q_ins.w()) * vec3(q_ins.x(),q_ins.y(),q_ins.z()); // 2 x Gibbs vector


        double u = 2 * ( a.x()*a.y() + 2*a.z() ) / ( 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2) );


        double du = 1 / (1 + pow(u,2));
        vec3 c_psi = du * ( 1 / pow( ( 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2) ), 2) ) * 
                vec3( -2*(( pow(a.x(),2) + pow(a.z(),2) - 4 )*a.y() + pow(a.y(),3) + 4*a.x()*a.z()),
                       2*(( pow(a.y(),2) - pow(a.z(),2) + 4 )*a.x() + pow(a.x(),3) + 4*a.y()*a.z()),
                       4*( pow(a.z(),2) + a.x()*a.y()*a.z() + pow(a.x(),2) - pow(a.y(),2) + 4 ));
      

        // position
        Cd.block(0,0,3,3) = I3;
        Cd.block(0,3,3,3) = O3;
        Cd.block(0,6,3,3) = O3;
        Cd.block(0,9,3,3) = O3;
        Cd.block(0,12,3,3) = O3;
        
        // velocity
        Cd.block(3,0,3,3) = O3;
        Cd.block(3,3,3,3) = I3;
        Cd.block(3,6,3,3) = O3;
        Cd.block(3,9,3,3) = O3;
        Cd.block(3,12,3,3) = O3;
        

        // Gravity
        Cd.block(6,0,3,3) = O3;
        Cd.block(6,3,3,3) = O3;
        Cd.block(6,6,3,3) = O3;
        Cd.block(6,9,3,3) = Smtrx(R.transpose()*v01);
        Cd.block(6,12,3,3) = O3;
        
        // heading
        Cd.block(9,0,1,9) = O_19; 
        Cd.block(9,9,1,3) = c_psi.transpose();
        Cd.block(9,12,1,3) = O_13;
        

        E.block(0,0,3,3) = O3;
        E.block(0,3,3,3) = O3;
        E.block(0,6,3,3) = O3;
        E.block(0,9,3,3) = O3;

        E.block(3,0,3,3) = -R;
        E.block(3,3,3,3) = O3;
        E.block(3,6,3,3) = O3;
        E.block(3,9,3,3) = O3;

        E.block(6,0,3,3) = O3;
        E.block(6,3,3,3) = I3;
        E.block(6,6,3,3) = O3;
        E.block(6,9,3,3) = O3;

        E.block(9,0,3,3) = O3;
        E.block(9,3,3,3) = O3;
        E.block(9,6,3,3) = -I3;
        E.block(9,9,3,3) = O3;

        E.block(12,0,3,3) = O3;
        E.block(12,3,3,3) = O3;
        E.block(12,6,3,3) = O3;
        E.block(12,9,3,3) = I3;

        Ed = h * E;


        
        // %%% kalman filter algorithm %%%


        // check if fresh corrections exists
        
        // * if available:
        // -> pop first measurement with timestamp older than imu timestamp
        // -> remove old data in buffer (set tail to the item that comes after the one we removed)
        // -> return true
        
        //cam_pose_ready_ = camPoseBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &cam_pose_delayed_); 

        sbg_ready_ = sbgNavBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &sbg_nav_delayed_); 

        dvl_ready_ = dvlBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &dvl_sample_delayed_);

        //dvl_ready_ = false; // dead reckoning

        //sbg_ready_ = false; // dead reckoning

        
        
        counter++;

        if(counter <= 15000){
            sbg_ready_ = false; // dead reckoning
        }

       
        
        if(counter > 15000 && sbg_ready_ == true){

            Qd = Qd/333;

            P_prd.diagonal() << 4*10e-6, 4*10e-6, 4*10e-6, // delta pos in NED [m] // TEST
                                4*10e-6, 4*10e-6, 4*10e-6, // delta velocity in NED [m/s] // TEST
                                pow(0.00015,2), pow(0.0002,2), pow(0.0008,2), // delta acceleration bias [m/s^2]  // TEST
                                0.00000005,  0.00000005,  0.00000005, // delta a 
                                0.00000025, 0.00000025, 0.00000025; // delta gyro bias [rad]
        
        
            P_prd = 0.01 * P_prd; 


        }
        


        
        
        
        
        
        
        
        /*
        if(dvl_ready_){
            std::cout << "DVL READY" << std::endl;
        } 

        if(sbg_ready_){
            std::cout << "SBG READY" << std::endl;
        } 
        */
        
            


        // extract pos/yaw measurements
        sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
        y_pos = sbg_nav_newest.posNED;

        sbg_vel = sbg_nav_newest.vel;

        sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
        sbg_quat = sbg_quat_newest.quatNED;

        // quaternion to euler angles       
        geometry_msgs::Quaternion quat_sbg2;
        quat_sbg2.x = sbg_quat.x();
	    quat_sbg2.y = sbg_quat.y();
	    quat_sbg2.z = sbg_quat.z();
	    quat_sbg2.w = sbg_quat.w();

	    // quat -> tf
	    tf::Quaternion quat_sbg3;
	    tf::quaternionMsgToTF(quat_sbg2, quat_sbg3);

        double roll_sbg, pitch_sbg, yaw_sbg;
        tf::Matrix3x3(quat_sbg3).getRPY(roll_sbg, pitch_sbg, yaw_sbg);
        y_psi = yaw_sbg;

        // extract velocity measurment (DVL)
        dvlSample dvl_data_newest = dvlBuffer_.get_newest();
        y_vel = dvl_data_newest.vel;

        // rotate from body to NED
        y_vel_NED = R * y_vel;

        
        /*
        // reject invalid DVL measurements
        if(!(dvl_data_newest.velocity_valid)){
            dvl_ready_ = false;

            // interpolate later

        }
        */

        // logging DVL velocity (in NED)
        int trace_id_dvl3 = 0;
        log_dvl_velocity << std::fixed;
        log_dvl_velocity << std::setprecision(16);
        log_dvl_velocity << ++trace_id_dvl3 << " " << time_us*10e-7 << " " << y_vel_NED.x() << " " << y_vel_NED.y() << " " << y_vel_NED.z() <<  std::endl;

        if(trace_id_dvl3 > 30000){
            log_dvl_velocity.close();
        }

        dvl_pos_integrated = dvl_pos_integrated + h*y_vel_NED; // + 0.5*pow(h,2)*acc;
   

        // logging

        int trace_id_dvl = 0;
	
        log_dvl_pos << std::fixed;
        log_dvl_pos << std::setprecision(16);
        log_dvl_pos << ++trace_id_dvl << " " << time_us*10e-7 << " " << dvl_pos_integrated.x() << " " << dvl_pos_integrated.y() << " " << dvl_pos_integrated.z() <<  std::endl;

        if(trace_id_dvl > 200){
            log_dvl_pos.close();
        }
        



        // SBG and DVL not available (no aiding)     
        if(!sbg_ready_ && !dvl_ready_){

            P_hat = P_prd;

        }

        // SBG + DVL available or SBG available
        
        else if(sbg_ready_ && dvl_ready_ || sbg_ready_ && !dvl_ready_){

            //std::cout << "SBG USED" << std::endl;

            // Logging
            /*
            int trace_id_sbg = 0;
            log_sbg_velocity << std::fixed;
            log_sbg_velocity << std::setprecision(16);
            log_sbg_velocity << ++trace_id_sbg << " " << time_us*10e-7 << " " << sbg_vel.x() << " " << sbg_vel.y() << " " << sbg_vel.z() << std::endl;

            if(trace_id_sbg > 30000){
                log_sbg_velocity.close();
            }
            */

        /*
            if(dvl_ready_){
                int trace_id_dvl = 0;
                log_dvl_velocity << std::fixed;
                log_dvl_velocity << std::setprecision(16);
                log_dvl_velocity << ++trace_id_dvl << " " << time_us*10e-7 << " " << y_vel_NED.x() << " " << y_vel_NED.y() << " " << y_vel_NED.z() <<  std::endl;

                if(trace_id_dvl > 30000){
                    log_dvl_velocity.close();
                }
            }
        */
            
            K.setZero(15,10); // TODO: initialize before?

            // KF gain
            K = P_prd * Cd.transpose() * (Cd * P_prd * Cd.transpose() + Rd).inverse(); 

            IKC.setZero(15,15); // TODO: initialize before?
            IKC = I15 - K * Cd;


            // estimation error
            vec3 v1 = -f_ins/g; // gravity vector
            v1 = v1 / sqrt( v1.transpose() * v1 );

            vec3 eps_pos = y_pos - p_ins;
            //vec3 eps_vel = sbg_vel - v_ins; 
            
            
            vec3 eps_g = v1 - R.transpose() * v01; 


            // inputs to atan2
            double x = ( 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2) );
            double y = 2 * ( a.x()*a.y() + 2*a.z() );

            // smallest signed angle 
            double eps_psi = ssa(y_psi - atan2(y,x)); // (in radians)
            //double eps_psi = ssa(y_psi - atan(u)); // (in radians)


            // error measurements 
            eps.setZero(10,1);
            eps.block(0,0,3,1) = eps_pos;
            //eps.block(3,0,3,1) = eps_vel;
            eps.block(6,0,3,1) = eps_g;
            eps(9,0) = eps_psi;

 
            // corrector
            delta_x_hat = K * eps;
            P_hat = IKC * P_prd * IKC.transpose() + K * Rd * K.transpose();


            // error quaternion (2 x Gibbs vector)
            vec3 delta_a = delta_x_hat.block(9,0,3,1);
            vec4 delta_q_hat_vec = 1/(sqrt(4 + delta_a.transpose() * delta_a)) * vec4(2,delta_a.x(),delta_a.y(),delta_a.z());
            quat delta_q_hat = quat(delta_q_hat_vec(0),delta_q_hat_vec(1),delta_q_hat_vec(2),delta_q_hat_vec(3)); // vec4 to quat

            // INS reset
            p_ins = p_ins + delta_x_hat.block(0,0,3,1);	                 // position
	        //v_ins = v_ins + delta_x_hat.block(3,0,3,1);			         // velocity
	        acc_bias_ins = acc_bias_ins + delta_x_hat.block(6,0,3,1);    // acc bias
	        gyro_bias_ins = gyro_bias_ins + delta_x_hat.block(12,0,3,1); // gyro bias         
            q_ins = quatprod(q_ins, delta_q_hat);                        // schur product   
            q_ins.normalize();                                           // normalization

        }
        
        
        // DVL available
        
        else if(!sbg_ready_ && dvl_ready_){

            //std::cout << "***** DVL USED *****" << std::endl;


            //dvl_pos_integrated = dvl_pos_integrated + dvl_dt*y_vel_NED;

            //dvl_pos_integrated = dvl_pos_integrated + (1/5.5)*y_vel_NED;


            // logging

            /*
            int trace_id = 0;
	
            log_dvl_pos << std::fixed;
            log_dvl_pos << std::setprecision(16);
            log_dvl_pos << ++trace_id << " " << time_us*10e-7 << " " << dvl_pos_integrated.x() << " " << dvl_pos_integrated.y() << " " << dvl_pos_integrated.z() <<  std::endl;

            if(trace_id > 1000){
                log_dvl_pos.close();
            }
            */

            /*
            int trace_id = 0;
	
            log_dvl_velocity << std::fixed;
            log_dvl_velocity << std::setprecision(16);
            log_dvl_velocity << ++trace_id << " " << time_us*10e-7 << " " << y_vel_NED.x() << " " << y_vel_NED.y() << " " << y_vel_NED.z() <<  std::endl;

            if(trace_id > 3000){
                log_dvl_velocity.close();
            }
            */

            
            K.setZero(15,10); // DVL + SBG
    
            // KF gain
            K = P_prd * Cd.transpose() * (Cd * P_prd * Cd.transpose() + Rd).inverse(); 


            IKC.setZero(15,15); // TODO: initialize before?
            IKC = I15 - K * Cd;


            // estimation error
            vec3 eps_vel = y_vel_NED - v_ins;
     

            // velocity measurements 
            //eps.setZero(10,1);
            eps.block(3,0,3,1) = eps_vel;

 
            // corrector
            delta_x_hat = K * eps;
            P_hat = IKC * P_prd * IKC.transpose() + K * Rd * K.transpose();


            // INS reset
	        v_ins = v_ins + delta_x_hat.block(3,0,3,1); // velocity
        }
        


        // predictor
        P_prd = Ad * P_hat * Ad.transpose() + Ed * Qd * Ed.transpose();


        // INS propagation: x_ins[k+1]
        if(counter > 15000){
            vec3 a_ins = R * f_ins + g_n;                                                          // linear acceleration
            p_ins = p_ins + h * v_ins + pow(h,2)/2 * a_ins;                                        // exact discretization
            //p_ins = p_ins + h * v_ins;                                                              // exact discretization
            v_ins = v_ins + h * a_ins;                                                             // exact discretization   
        }

        // DVL - constant velocity - avoid f_ins
        if(counter <= 15000){
            vec3 p_old = p_ins;
            p_ins = p_old + h*y_vel_NED;
            v_ins = (p_ins-p_old)/h;
        }

        
        //vec4 q_ins_vec = (h*Tquat_vec3(w_ins)).exp() * vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()); // exact discretization
        vec4 q_ins_vec = vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()) + h*Tquat_quat(q_ins) * w_ins; // Euler's method (alternative)

        q_ins = quat(q_ins_vec(0),q_ins_vec(1),q_ins_vec(2),q_ins_vec(3));                     // vec4 to quat 
        q_ins.normalize();                                                                     // normalization

        // update INS states
        state_.pos = p_ins;
        state_.vel = v_ins;
        state_.accel_bias = acc_bias_ins;
        state_.quat_nominal = q_ins;
        state_.gyro_bias = gyro_bias_ins;

        // copy a_ins
        //acc = a_ins;

        //std::cout << "acc bias: " << std::endl; 
        //std::cout << acc_bias_ins << std::endl;

        // logging SBG velocity data
        int trace_id_sbg = 0;
        log_sbg_velocity << std::fixed;
        log_sbg_velocity << std::setprecision(16);
        log_sbg_velocity << ++trace_id_sbg << " " << time_us*10e-7 << " " << sbg_vel.x() << " " << sbg_vel.y() << " " << sbg_vel.z() << std::endl;

        if(trace_id_sbg > 30000){
            log_sbg_velocity.close();
        }

        // logging IMU velocity data
        /*
        int trace_id_ins = 0;
        log_ins_velocity << std::fixed;
        log_ins_velocity << std::setprecision(16);
        log_ins_velocity << ++trace_id_ins << " " << time_us*10e-7 << " " << v_ins.x() << " " << v_ins.y() << " " << v_ins.z() << std::endl;

        if(trace_id_ins > 30000){
            log_ins_velocity.close();
        } 
        */   





        // %%% LOG BIAS %%%


        int trace_id = 0;
	
        log_imu_bias << std::fixed;
        log_imu_bias << std::setprecision(16);
        log_imu_bias << ++trace_id << " " << time_us*10e-7 << " " << acc_bias_ins.x() << " " << acc_bias_ins.y() << " " << acc_bias_ins.z() << " " << gyro_bias_ins.x() << " " << gyro_bias_ins.y() << " " << gyro_bias_ins.z() <<  std::endl;

        if(trace_id > 30000){
            log_imu_bias.close();
        }
	    
        

        // %%%%%%% extract ground truth %%%%%%%%%%%
        
        /*
        sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
        sbg_pos = sbg_nav_newest.posNED;
	    sbg_vel = sbg_nav_newest.vel;

        sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
        sbg_quat = sbg_quat_newest.quatNED;

        
        // quaternion to euler angles       
        geometry_msgs::Quaternion quat_sbg2;
        quat_sbg2.x = sbg_quat.x();
	    quat_sbg2.y = sbg_quat.y();
	    quat_sbg2.z = sbg_quat.z();
	    quat_sbg2.w = sbg_quat.w();

	    // quat -> tf
	    tf::Quaternion quat_sbg3;
	    tf::quaternionMsgToTF(quat_sbg2, quat_sbg3);

        double roll_sbg, pitch_sbg, yaw_sbg;
        tf::Matrix3x3(quat_sbg3).getRPY(roll_sbg, pitch_sbg, yaw_sbg); 
        */
    
        
        std::cout << "SBG pos: " << std::endl;
        std::cout << y_pos << std::endl;
        //std::cout << "SBG vel: " << std::endl;
        //std::cout << sbg_vel << std::endl;
        std::cout << "SBG yaw: " << y_psi*(180/M_PI) << std::endl;
        

        int trace_id_sbg2 = 0;
	
        log_sbg_pos_headig << std::fixed;
        log_sbg_pos_headig << std::setprecision(16);
        log_sbg_pos_headig << ++trace_id_sbg2 << " " << time_us*10e-7 << " " << y_pos.x() << " " << y_pos.y() << " " << y_pos.z() << " " << y_psi*(180/M_PI) << std::endl;

        if(trace_id_sbg2 > 30000){
            log_sbg_pos_headig.close();
        }
        

        // %%%%% extract estimated %%%%%%%%%%

        
        // quaternion to euler angles       
        geometry_msgs::Quaternion quat_msg2;
        quat_msg2.x = q_ins.x();
	    quat_msg2.y = q_ins.y();
	    quat_msg2.z = q_ins.z();
	    quat_msg2.w = q_ins.w();

	    // quat -> tf
	    tf::Quaternion quat3;
	    tf::quaternionMsgToTF(quat_msg2, quat3);

        double roll_est, pitch_est, yaw_est;
        tf::Matrix3x3(quat3).getRPY(roll_est, pitch_est, yaw_est); 
        
        
        std::cout << "estimated pos: " << std::endl;
        std::cout << state_.pos << std::endl;

        //std::cout << "y pos: " << std::endl;
        //std::cout << y_pos << std::endl;

        //std::cout << "estimated vel: " << std::endl;
        //std::cout << state_.vel << std::endl;
        //std::cout << "dvl vel: " << std::endl;
        //std::cout << y_vel << std::endl;
        
        std::cout << "estimated yaw: " << yaw_est*(180/M_PI) << std::endl;
        

        int trace_id_dvl2 = 0;
	
        log_dvl_pos_heading << std::fixed;
        log_dvl_pos_heading << std::setprecision(16);
        log_dvl_pos_heading << ++trace_id_dvl2 << " " << time_us*10e-7 << " " << state_.pos.x() << " " << state_.pos.y() << " " << state_.pos.z() << " " << yaw_est*(180/M_PI) << std::endl;

        if(trace_id_dvl2 > 30000){
            log_dvl_pos_heading.close();
        }
        

        // reset flag after each INS update 
        //cam_pose_ready_ = false;  
        dvl_ready_ = false; 
        sbg_ready_ = false;

        // assign publish_MEKF_ to true if mekf estimates are accurate enough (so we can publish mekf instead of sbg)
        //publish_MEKF(time_us); 

      
    }




    // gravity model (MSS toolbox)
    double MEKF::gravity(double mu){
        return 9.7803253359 * ( 1 + 0.001931850400 * pow(sin(mu),2) ) / sqrt( 1 - 0.006694384442 * pow(sin(mu),2) );
    }

    // Smallest signed angle (MSS toolbox)
    //double MEKF::ssa(double angle){
    //    return ((angle + M_PI) - floor((angle + M_PI)/(2*M_PI)) * 2*M_PI) - M_PI;
    //}

    double MEKF::ssa(double angle){
        return mod(angle + M_PI, 2*M_PI) - M_PI;
    }

    double MEKF::mod(double x, double y){
        return x - floor(x/y) * y;
    }

    
    //angle = mod( angle + pi, 2 * pi ) - pi;

    //mod(x,y) = x - floor(x/y) * y




    quat MEKF::getQuat(){
        return state_.quat_nominal;
    }

    vec3 MEKF::getPosition(){
        return state_.pos;
    }

    vec3 MEKF::getVelocity(){
        return state_.vel;
    }

    uint64_t MEKF::getImuTime(){
        return time_last_imu_; 
    }




}


