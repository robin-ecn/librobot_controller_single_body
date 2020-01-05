#ifndef FBL_H
#define FBL_H
#include <cmath>
#include <iostream>

//namespace Eigen {
//   typedef Matrix<double,6,1> Vector6d;
//}

#include <Eigen/Dense>

class FBL{

    public:

    // declare the 6d vector for trajectory
    typedef Eigen::Matrix<double,6,1> Vector6d;
    
    // initial controller with the frenquency
    FBL(const double Frequency, const double mass, const Eigen::Matrix3d Inertia);

    // get trajectory and current pose
    void getTrajectoryAndPose(const FBL::Vector6d q, const FBL::Vector6d q_des, const FBL::Vector6d dq_des, const FBL::Vector6d ddq_des);


    // set gains for controller
    void getGains(const Eigen::Vector3d Kp_t, const Eigen::Vector3d Kd_t, const Eigen::Vector3d Kp_a, const Eigen::Vector3d Kd_a); 

    // output the required wrench (6X1)
    void getControlInputs(FBL::Vector6d &wrench_des);

    void getTrackingError();

    void getVirtualInput();

    void getInversedDynamic();

    void getRotationMatrix();

    // update the drone previous position and attitude
    void update(const FBL::Vector6d q);


    /************** mass and inertia of robot******************/
    double m_ = 1;

    Eigen::Matrix3d Im_;

    // frenquency of controller
    double Frequency_;
            
    // gravity vector
    // Eigen::Vector3d g(0, 0, 9.8);
    Eigen::Vector3d g;


    /************** Desired trajectory******************/  

    // desired posiiton and attitude vectors
    Eigen::Vector3d qt_des_;
    Eigen::Vector3d dqt_des_;
    Eigen::Vector3d ddqt_des_;

    Eigen::Vector3d qa_des_;
    Eigen::Vector3d dqa_des_;
    Eigen::Vector3d ddqa_des_;


   /************** Current Pose******************/
    // current pose
    // position vector and velocity vector
    Eigen::Vector3d qt_;
    Eigen::Vector3d dqt_;    

    // attitdue vector and angular velocity vector
    Eigen::Vector3d qa_;
    Eigen::Vector3d dqa_;   

    // euler angles of robot
    double phi_;
    double theta_;
    double psi_;   

    // euler angular speed of robot
    double dphi_;
    double dtheta_;
    double dpsi_;     
    

    // privious position and attitude
    Eigen::Vector3d qt_pre_;
    Eigen::Vector3d qa_pre_;


/************** Controller******************************/
    // Gains 
    Eigen::DiagonalMatrix<double, 3> kp_t_;

    Eigen::DiagonalMatrix<double, 3> kd_t_;    

    Eigen::DiagonalMatrix<double, 3> kp_a_;        
      
    Eigen::DiagonalMatrix<double, 3> kd_a_;   

     // translational tracking error
    Eigen::Vector3d e_t_;

    Eigen::Vector3d de_t_;

    // oritantional tracking error
    Eigen::Vector3d e_a_;

    Eigen::Vector3d de_a_;      

    // rotation matrix 
    Eigen::Matrix3d T_;

    // dirative of rotation matrix 
    Eigen::Matrix3d dT_;    


    // virtual input for translation
    Eigen::Vector3d v_p_;
    Eigen::Vector3d v_a_;

    // output force and torque
    Eigen::Vector3d f_;    
    Eigen::Vector3d tau_;



};

#endif
