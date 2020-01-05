#include <librobot_controller_single_body/librobot_controller_single_body.hpp>

#include <iostream>
#include <cmath>

int main()
{

      // 0. get desired trajectory and current pose
      FBL::Vector6d q;
      q<<1,2,3,    0.8147,
        0.9058,
        0.1270;

      FBL::Vector6d q_des;
      q_des<<7,8,9,10,11,12;     

      FBL::Vector6d dq_des;
      dq_des<<13,14,15,16,17,18; 

      FBL::Vector6d ddq_des;
      ddq_des<<19,20,21,22,23,24; 

      double  Frenquency_ = 200;        

      FBL::Vector6d wrench_des;
      // 1.define FBL object
      //std::cout<<"p0"<<std::endl;
      FBL controllerFBL(Frenquency_);
      //std::cout<<"p1"<<std::endl;
      controllerFBL.getTrajectoryAndPose(q, q_des, dq_des, ddq_des);
      //std::cout<<"p2"<<std::endl;
      controllerFBL.getControlInputs(wrench_des);
      //std::cout<<"p3"<<std::endl;
      controllerFBL.update(q);
      //std::cout<<"p4"<<std::endl;

}



