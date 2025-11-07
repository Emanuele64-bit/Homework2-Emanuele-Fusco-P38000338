#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::velocity_control_null(KDL::Frame &_desPos,
                                                    //  KDL::Twist &_desVel,
                                                    //  KDL::Twist &_desAcc,
                                                     double Kpp)
{
    // Stato
    Eigen::VectorXd q = robot_->getJntValues();      // n×1
    const int n = q.size();
    // unsigned int m = robot_->getNrJnts();

    // Errore di posizione (solo traslazione)
    KDL::Frame cartpos = robot_->getEEFrame();
    KDL::Vector d = _desPos.p - cartpos.p;
    Eigen::Vector3d e_p(d.x(), d.y(), d.z());        // 3×1


    // Jacobiano (6×n) e parte lineare (3×n)
    KDL::Jacobian J_kdl = robot_->getEEJacobian();
    Eigen::MatrixXd J = J_kdl.data;    //Eigen::MatrixXd J_dag = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd J_dag = J.transpose()*(J.inverse()*J.transpose()).inverse();


    // Limiti
    Eigen::MatrixXd limits = robot_->getJntLimits();
    Eigen::VectorXd qmin = limits.col(0);
    Eigen::VectorXd qmax = limits.col(1);


    const double lambda = 1e-3;

    Eigen::VectorXd gradient;
    gradient.resize(n);
    
    for(unsigned int i = 0; i < n; i++) {
        if (q(i,0) < qmin(i) || q(i,0) > qmax(i)) {
            std::cout << "joint " << i << " limits violated. Value = "<< q(i,0)*180.0/M_PI << "\n"; 
        }
            gradient(i,0) = 1.0/lambda * std::pow((qmax(i) - qmin(i)),2)* (2*q(i,0) - qmax(i) - qmin(i))/(std::pow((qmax(i)-q(i,0)),2)*std::pow((q(i,0)-qmin(i)),2));
        } 
    Eigen::VectorXd q0_dot; 
    q0_dot = gradient; 
    Eigen::VectorXd q_dot = J*Kpp*e_p +(I-J_dag*J)*q0_dot;

    return q_dot;
}




// const double eps = 1e-9;
// Eigen::VectorXd qdot0(n);
// for (int i = 0; i < n; ++i) {
//     const double qi  = q(i);
//     const double qmi = qmin(i);
//     const double qMa = qmax(i);

//     if (qi < qmi || qi > qMa) {
//         std::cout << "joint " << i << " limits violated. Value = "
//                   << qi * 180.0 / M_PI << "\n";
//     }

//     const double d1 = std::max(qMa - qi, eps);   // (q+ - q)
//     const double d2 = std::max(qi - qmi, eps);   // (q - q-)
//     const double range = (qMa - qmi);
//     const double num   = (2.0*qi - qMa - qmi);
//     const double grad_i = (range*range * num) / (d1*d1 * d2*d2) / gamma_lim;

//     qdot0(i) = -grad_i;
// }

// // Proiezione
// Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
// Eigen::MatrixXd N = I - Jp_dag * Jp;

// // Legge: q̇ = Jp^† (Kp e_p) + N q̇0
// Eigen::VectorXd qdot = Jp_dag * (_Kpp * e_p) + N * qdot0; // n×1
// return qdot;


// Eigen::VectorXd calculate_q_dot_0(double gamma, Eigen::VectorXd q){

//     unsigned int n = robot_->getNrJnts();
//     Eigen::VectorXd gradient;
//     gradient.resize(n);

//     for(unsigned int i = 0; i < n; i++)
//     {
//         if (q(i,0) < robot_->jntLimits(i,0) || q(i,0) > robot_->jntLimits(i,1))
//         {
//             std::cout << "joint " << i << " limits violated. Value = "<< q(i,0)*180.0/M_PI << "\n";
//         }
//         gradient(i,0) = 1.0/gamma * std::pow((robot_->jntLimits(i,1) - robot_->jntLimits(i,0)),2)* (2*q(i,0) - robot_->jntLimits(i,1) - robot_->jntLimits(i,0))/(std::pow((robot_->jntLimits(i,1)-q(i,0)),2)*std::pow((q(i,0)-robot_->jntLimits(i,0)),2));
//     }
//     return gradient;
// }


    



