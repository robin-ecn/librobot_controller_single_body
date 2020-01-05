#include <librobot_controller_single_body/librobot_controller_single_body.hpp>



    FBL::FBL(const double Frequency, const double mass, const Eigen::Matrix3d Inertia):Frequency_(Frequency), m_(mass), Im_(Inertia)
    {
        //std::cout<<"point A"<<std::endl;
        // initial matrix
        /*Im_<<1.0f,0.0f,0.0f,
            0.0f,1.0f,0.0f,
            0.0f,0.0f,1.0f;
        */
        //std::cout<<"point B"<<std::endl;
        // graviry vector
        g<<0,0,9.8;    
        
        //std::cout<<"point C"<<std::endl;
        // intiliza the previous pose   
        qt_pre_ = Eigen::MatrixXd::Zero(3,1);
        qa_pre_ = Eigen::MatrixXd::Zero(3,1);

        //std::cout<<"point D"<<std::endl;
        kp_t_.setIdentity();
        kd_t_.setIdentity();

        kp_a_.setIdentity();
        kd_a_.setIdentity();                            

    }



    void FBL::getGains(const Eigen::Vector3d Kp_t, const Eigen::Vector3d Kd_t, const Eigen::Vector3d Kp_a, const Eigen::Vector3d Kd_a) 
    {

            // translational gains
            kp_t_ = Kp_t.asDiagonal();

            kd_t_ = Kd_t.asDiagonal();

            // oritational gains
            kp_a_ = Kp_a.asDiagonal();

            kd_a_ = Kp_a.asDiagonal();
    }



    void FBL::getTrajectoryAndPose(const FBL::Vector6d q, const FBL::Vector6d q_des, const FBL::Vector6d dq_des, const FBL::Vector6d ddq_des)
    {

        //1. get desired position and attitude
        qt_des_ = q_des.head<3>();      
        qa_des_ = q_des.tail<3>();      

        dqt_des_ = dq_des.head<3>();      
        dqa_des_ = dq_des.tail<3>();  

        ddqt_des_ = ddq_des.head<3>();
        ddqa_des_ = ddq_des.tail<3>();   

        // 2. get current position and attitude

        // current position vector
        qt_ = q.head<3>();
        // current euler angle vector
        qa_ = q.tail<3>();

        // get Euler angles
        phi_   = qa_(0);
        theta_ = qa_(1);
        psi_   = qa_(2);

        // velocity vector
        dqt_ = (qt_ - qt_pre_)*Frequency_;
        dqa_ = (qa_ - qa_pre_)*Frequency_;   

        dphi_   = dqa_(0);
        dtheta_ = dqa_(1);
        dpsi_   = dqa_(2);       

        // 3. get rotation matrix T_ and dT_ 
        getRotationMatrix();  

    }


    // output the control input: force (3X1) and torque (3X1)
    void FBL::getControlInputs(FBL::Vector6d &wrench_des)
    {
        // 1. get tracking error e_t_, e_a_ and de_t_, de_a_
        getTrackingError();

        // 2. get virtual input v_t and v_a_
        getVirtualInput();

        // 3. get requred force and torques: f_ and tau_
        getInversedDynamic();

        // 4. output the wrench
        wrench_des<<f_,tau_;

    }




    // get the tracking error in translation and oritation
    void FBL::getTrackingError()
    {
        // translational tracking error
        //e_t_ =  qt_ - qt_pre_; 
        e_t_ = qt_des_ - qt_;

        //de_t_ = e_t_*Frequency_;
        de_t_ = dqt_des_ - dqt_;

        // oritational tracking error
        //e_a_ =  qa_ - qa_pre_;
        e_a_ =  qa_des_ - qa_;   

        //de_a_ = e_a_*Frequency_;
        de_a_ = dqa_des_ - dqa_;
    }



    void FBL::getVirtualInput()
    {
        // virtual input for translation
        v_p_ = ddqt_des_ + kp_t_*e_t_+kd_t_*de_t_;
        
        // virtual input for attitude
        v_a_ = ddqa_des_ + kp_a_*e_a_+kd_a_*de_a_;

    }

    void FBL::getInversedDynamic()
    {
        // translational invertied dynamic
        f_ = m_*g + m_*v_p_;
        
        // oritational invertied dynamic

        // intermiate varaiables
        Eigen::Vector3d rs_Tdqa;

        rs_Tdqa = T_ * dqa_;

        Eigen::Vector3d rs_ITdqa;
        rs_ITdqa =  Im_*T_*dqa_; 

        tau_ = Im_ * (T_ * v_a_ + dT_ * dqa_) + rs_Tdqa.cross(rs_ITdqa);

    }




    void FBL::getRotationMatrix()
    {
        // get rotation matrix based on euler angles 3-2-1
        Eigen::AngleAxisd rollAngle(phi_, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(theta_, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(psi_, Eigen::Vector3d::UnitZ());

        /*Eigen::Quaterniond Q =  rollAngle *pitchAngle*yawAngle;
        Eigen::Matrix3d R = Q.toRotationMatrix();
        T_ = R;
        */
        T_ = yawAngle*pitchAngle*rollAngle;

        // get diritive of rotation matrix
        dT_<< 1,0,-cos(theta_)*dtheta_,
             0, -sin(phi_)*dphi_, cos(phi_)*cos(theta_)*dphi_-sin(phi_)*sin(theta_)*dtheta_,
             0, -cos(phi_)*dphi_, -sin(phi_)*cos(theta_)-cos(phi_)*sin(theta_)*dtheta_;

    }





    void FBL::update(const FBL::Vector6d q)
    {
        // save the last position and attitude
        qt_pre_ = qt_;
        qa_pre_ = qa_;

        // get current position and attitude
        qt_ = q.head<3>();
        qa_ = q.tail<3>();

    }

    


/*
    void FBL::initPara()
    {
        // current position vector
        qt_ = q.head<3>();

        // current euler angle vector
        qa_ = q.tail<3>();

        phi_   = qa_(0);
        theta_ = qa_(1);
        psi_   = qa_(2);

        // velocity vector
        dqt_ = (qt_ - qt_pre_)*Frequency_;
        dqa_ = (qa_ - qa_pre_)*Frequency_;   

        dphi_   = dqa_(0);
        dtheta_ = dqa_(1);
        dpsi_   = dqa_(2);         

        // get euler angular speed
        dqa_ = (qa_-qa_pre_)* Frequency_;      


    }
*/