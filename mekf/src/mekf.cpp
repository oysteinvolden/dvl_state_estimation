#include <mekf/mekf.h>


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

        // allocate SBG pos buffer
        sbgNavBuffer_.allocate(sbg_nav_buffer_length_);
        for (int i = 0; i < sbg_nav_buffer_length_; i++) {
            sbgNavSample sbg_nav_sample_init = {};
            sbgNavBuffer_.push(sbg_nav_sample_init);
        }

        // allocate SBG quat buffer
        sbgQuatBuffer_.allocate(sbg_quat_buffer_length_);
        for (int i = 0; i < sbg_quat_buffer_length_; i++) {
            sbgQuatSample sbg_quat_sample_init = {};
            sbgQuatBuffer_.push(sbg_quat_sample_init);
        }
        
        // set to false, so we can initialize
        filter_initialised_ = false;

    }


    bool MEKF::initializeFilter(){

        // *** initialize covariance matrices ***

        // put the squares of the typical values of the the state components + margin        
        P_prd.diagonal() << 0.000016, 0.000016, 0.000016, // delta pos in NED [m] 
                            0.000016, 0.000016, 0.000016, // delta velocity in NED [m/s]
                            0.000003,   0.000003,   0.000003, // delta acceleration bias [m/s^2]  
                            0.00000005,  0.00000005,  0.00000005, // delta a 
                            0.00000025, 0.00000025, 0.00000025; // delta gyro bias [rad]
        
        // ADIS imu data
        double sigma_w_acc = 0.0001333; // velocity random walk
        double sigma_b_acc = 8.3212184588*10e-7; // velocity in run instability bias
        double sigma_w_ars = 0.000026179938; // angular random walk
        double sigma_b_ars = 6.54728501*10e-8; // angular in run instability bias 
        
        // initialize process weights - v, acc_bias, w, gyro_bias (v, acc_bias, w, ars_bias)        
        Qd.diagonal() << sigma_w_acc, sigma_w_acc, sigma_w_acc,
                         sigma_b_acc, sigma_b_acc, sigma_b_acc,
                         sigma_w_ars, sigma_w_ars, sigma_w_ars,
                         sigma_b_ars, sigma_b_ars, sigma_b_ars;
        
        Qd = 3 * h * Qd; // SBG / CAM
        //Qd = 1000 * h * Qd; // DVL
        
        // DVL + SBG / CAM
        Rd.diagonal() << 0.001, 0.001, 0.001, // p
                         0.001, 0.001, 0.001, // vel
                         0.001, 0.001, 0.001, // acc (gravity vector)
                         0.0001;       // psi   

        Rd = 0.001*Rd; // adis imu
    

        // %%%%% initialize state %%%%
	
        // extract SBG nav sample
        sbgNavSample sbg_nav_newest = sbgNavBuffer_.get_newest();
        sbg_pos = sbg_nav_newest.posNED;
	    sbg_vel = sbg_nav_newest.vel; 

        // check if SBG sample has arrived 
        if(sbg_pos.x() == 0 && sbg_pos.y() == 0 && sbg_pos.z() == 0){
            return false;
        }
        else
        {
            // extract sbg quat sample
            sbgQuatSample sbg_quat_newest = sbgQuatBuffer_.get_newest();
            sbg_quat = sbg_quat_newest.quatNED;
            
            // initialize with ground truth SGB INS
            state_.quat_nominal = sbg_quat;
            state_.vel = sbg_vel;
            state_.pos = sbg_pos;
            state_.gyro_bias = vec3(0,0,0); 
            state_.accel_bias = vec3(0,0,0);

            // pre-fixed bias 
            //state_.accel_bias = vec3(-0.065,-0.035,0);
            
            return true;
        }

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
        cam_pose_sample_new_.posErr = 0.05; 
        cam_pose_sample_new_.angErr = 0.05; 
        cam_pose_sample_new_.time_us = time_usec; 

        // update last time
        time_last_cam_pose_ = time_usec;

        // push pose to buffer
        camPoseBuffer_.push(cam_pose_sample_new_);
    }
    

    void MEKF::updateSbgNav(const vec3& sbg_pos, const vec3& sbg_vel, uint64_t time_usec){

        // copy required data
        sbg_nav_sample_new_.posNED = sbg_pos;
	    sbg_nav_sample_new_.vel = sbg_vel;
        sbg_nav_sample_new_.time_us = time_usec; 

        // update last time
        time_last_sbg_pos_ = time_usec;

        // push pose to buffer
        sbgNavBuffer_.push(sbg_nav_sample_new_); 
    }

    void MEKF::updateSbgQuat(const quat& sbg_quat, uint64_t time_usec){

        // copy required data
        sbg_quat_sample_new_.quatNED = sbg_quat;
        sbg_quat_sample_new_.time_us = time_usec; 

        // update last time
        time_last_sbg_quat_ = time_usec;

        // push pose to buffer
        sbgQuatBuffer_.push(sbg_quat_sample_new_); 
    }




    void MEKF::run_mekf(const vec3& ang_vel, const vec3& lin_acc, uint64_t time_us, double dt){

        
        // check if unit constraint is satisfied
        sbgQuatSample sbg_quat_sample = sbgQuatBuffer_.get_newest();
        sbg_quat = sbg_quat_sample.quatNED; 

        double quat_norm_sbg = sqrt ( pow(sbg_quat.w(),2) + pow(sbg_quat.x(),2) + pow(sbg_quat.y(),2) + pow(sbg_quat.z(),2) ) ;

        if( !(quat_norm_sbg >= 0.9999 && quat_norm_sbg <= 1.0001) ){ 
            std::cout << "SBG quat not properly normalized" << std::endl;
            return;
        } 
           
        // copy imu data
        imu_sample_new_.delta_ang = vec3(ang_vel.x(), ang_vel.y(), ang_vel.z()); // current yaw rate [rad/s] 
        imu_sample_new_.delta_vel = vec3(lin_acc.x(), lin_acc.y(), lin_acc.z()); // current linear acceleration [m/s^2]
        imu_sample_new_.delta_ang_dt = dt;
        imu_sample_new_.delta_vel_dt = dt;
        imu_sample_new_.time_us = time_us;
        time_last_imu_ = time_us; 

        // push to buffer
        imuBuffer_.push(imu_sample_new_); 

        // get the oldest data from the buffer
        imu_sample_delayed_ = imuBuffer_.get_oldest();

        // check if filter is initialized
        if (!filter_initialised_) {
            filter_initialised_ = initializeFilter();
            if (!filter_initialised_) {
                return;
            }
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

      
        // linearization of heading measurement
        vec3 a = (2/q_ins.w()) * vec3(q_ins.x(),q_ins.y(),q_ins.z()); // 2 x Gibbs vector

        double u_y = 2 * ( a.x()*a.y() + 2*a.z() );
        double u_x = 4 + pow(a.x(),2) - pow(a.y(),2) - pow(a.z(),2);
        double u = u_y/u_x;
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
        
        cam_pose_ready_ = camPoseBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &cam_pose_delayed_); 

        sbg_ready_ = sbgNavBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &sbg_nav_delayed_); 

        dvl_ready_ = dvlBuffer_.pop_first_older_than(imu_sample_delayed_.time_us, &dvl_sample_delayed_);


        sbg_ready_ = false; // dead reckoning

        
        // count IMU iteration
        counter++;

        // simulate camera dropout
        //if(counter >= 4750){
        //    cam_pose_ready_ = false; // dead reckoning
        //}
       
 
        // %%% extract camera pos/yaw measurements %%%
        if(cam_pose_ready_){

            cameraPoseSample cam_newest = camPoseBuffer_.get_newest();
            cam_pos = cam_newest.posNED;
            quat cam_quat = cam_newest.quatNED;

            // quaternion to euler angles       
            geometry_msgs::Quaternion quat_cam;
            quat_cam.x = cam_quat.x();
	        quat_cam.y = cam_quat.y();
	        quat_cam.z = cam_quat.z();
	        quat_cam.w = cam_quat.w();

	        // quat -> tf
	        tf::Quaternion quat_cam2;
	        tf::quaternionMsgToTF(quat_cam, quat_cam2);
            tf::Matrix3x3(quat_cam2).getRPY(roll_cam, pitch_cam, yaw_cam);

            // measured euclidean distance (camera)
            d_cam = sqrt( pow(cam_pos.x(),2) + pow(cam_pos.y(),2) ); 
        }

        

        // %%% extract velocity measurment (DVL) %%%
        dvlSample dvl_data_newest = dvlBuffer_.get_newest();
        dvl_vel = dvl_data_newest.vel;

        // rotate from body to NED
        dvl_vel_NED = R * dvl_vel;


        // acceptance critera (the second critera (n>=8 point correspondences) is handled in the AprilTag node)
        if(d_cam > 20 && cam_pose_ready_){       
            cam_pose_ready_ = false;
        }

            
        // cam not available (no aiding - assume GNSS never available)     
        if(!cam_pose_ready_){

            P_hat = P_prd;

        }

        // cam available
        if(cam_pose_ready_ && d_cam <= 20){  
                    
            // KF gain
            K.setZero(15,10);
            K = P_prd * Cd.transpose() * (Cd * P_prd * Cd.transpose() + Rd).inverse(); 

            IKC.setZero(15,15); 
            IKC = I15 - K * Cd;

            // estimation error
            vec3 v1 = -f_ins/g; // gravity vector
            v1 = v1 / sqrt( v1.transpose() * v1 );

            vec3 eps_pos = cam_pos - p_ins; // cam
            vec3 eps_vel = dvl_vel_NED - v_ins; // DVL - we use the last available measurement here
            vec3 eps_g = v1 - R.transpose() * v01; 

            // smallest signed angle 
            double eps_psi = ssa(yaw_cam - atan2(u_y,u_x)); // (in radians)
       
            // error measurements 
            eps.setZero(10,1);
            eps.block(0,0,3,1) = eps_pos;
            eps.block(3,0,3,1) = eps_vel;
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
	        v_ins = v_ins + delta_x_hat.block(3,0,3,1);			         // velocity
	        acc_bias_ins = acc_bias_ins + delta_x_hat.block(6,0,3,1);    // acc bias
	        gyro_bias_ins = gyro_bias_ins + delta_x_hat.block(12,0,3,1); // gyro bias         
            q_ins = quatprod(q_ins, delta_q_hat);                        // schur product   
            q_ins.normalize();                                           // normalization

        }
                
        // predictor
        P_prd = Ad * P_hat * Ad.transpose() + Ed * Qd * Ed.transpose();

        // INS propagation: x_ins[k+1]
        
        // if landmarks availalble
        if(d_cam <= 20 && cam_pose_ready_){  
        //if(counter < 4750){ // camera dropout      
            vec3 a_ins = R * f_ins + g_n;                                                          // linear acceleration
            p_ins = p_ins + h * v_ins + pow(h,2)/2 * a_ins;                                        // exact discretization
            //p_ins = p_ins + h * v_ins;                                                           // exact discretization
            v_ins = v_ins + h * a_ins;                                                             // exact discretization   
        }

        // measurement dropout
        else{
            vec3 p_old = p_ins;
            p_ins = p_old + h*dvl_vel_NED;
            v_ins = (p_ins-p_old)/h;
        }
        

        vec4 q_ins_vec = (h*Tquat_vec3(w_ins)).exp() * vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()); // exact discretization
        //vec4 q_ins_vec = vec4(q_ins.w(),q_ins.x(),q_ins.y(),q_ins.z()) + h*Tquat_quat(q_ins) * w_ins; // Euler's method (alternative)
        q_ins = quat(q_ins_vec(0),q_ins_vec(1),q_ins_vec(2),q_ins_vec(3));                     // vec4 to quat 
        q_ins.normalize();                                                                     // normalization
        

        // update INS states
        state_.pos = p_ins;
        state_.vel = v_ins;
        state_.accel_bias = acc_bias_ins;
        state_.quat_nominal = q_ins;
        state_.gyro_bias = gyro_bias_ins;


        // reset flag after each INS update 
        cam_pose_ready_ = false;  
        dvl_ready_ = false; 
        sbg_ready_ = false;
      
    }




    double MEKF::gravity(double mu){
        return 9.7803253359 * ( 1 + 0.001931850400 * pow(sin(mu),2) ) / sqrt( 1 - 0.006694384442 * pow(sin(mu),2) );
    }


    double MEKF::ssa(double angle){
        return mod(angle + M_PI, 2*M_PI) - M_PI;
    }

    double MEKF::mod(double x, double y){
        return x - floor(x/y) * y;
    }

    

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


