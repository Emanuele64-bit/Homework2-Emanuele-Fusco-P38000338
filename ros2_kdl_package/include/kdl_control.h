#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd velocity_control_null(KDL::Frame &_desPos,
                        //    KDL::Twist &_desVel,
                        //    KDL::Twist &_desAcc,
                           double _Kpp);

    Eigen::VectorXd compute_q0_dot(KDL::Frame &_desPos,
                                                    //  KDL::Twist &_desVel,
                                                    //  KDL::Twist &_desAcc,
                                                     double Kpp);
                                                    
    Eigen::VectorXd compute_J_cam(KDL::Chain chain_);

    Eigen::VectorXd vision_control (KDL::Frame &_desPos,
                                                Eigen::Vector3d &_P_c_o,
                                                double qx,
                                                double qy,
                                                double qz,
                                                double qw,
                                                double Kpp,
                                                KDL::Chain chain_,
                                                Eigen::MatrixXd K);

private:

    KDLRobot* robot_;

};

#endif
