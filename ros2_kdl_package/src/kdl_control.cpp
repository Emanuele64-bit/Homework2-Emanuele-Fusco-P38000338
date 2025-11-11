#include "kdl_control.h"
#include "utils.h"


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

Eigen::VectorXd KDLController::velocity_control_null(KDL::Frame &_desPos, double Kpp)
{
    // Stato
    Eigen::VectorXd q = robot_->getJntValues();      // n×1
    const int n = q.size();

    //Errore di posizione (solo traslazione)
    KDL::Frame cartpos = robot_->getEEFrame();
    KDL::Vector d = _desPos.p - cartpos.p;
    Eigen::Vector3d e_p(d.x(), d.y(), d.z());        // 3×1

    // Jacobiana (6×n) e parte lineare (3×n)
    KDL::Jacobian J_kdl = robot_->getEEJacobian();
    Eigen::MatrixXd J6 = J_kdl.data;    //Eigen::MatrixXd J_dag = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd J = J6.block(0, 0, 3, n);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd J_dag = J.transpose()*(J*J.transpose()).inverse();



    // Limiti

    Eigen::MatrixXd limits = robot_->getJntLimits();
    Eigen::VectorXd qmin = limits.col(0);
    Eigen::VectorXd qmax = limits.col(1);

    const double lambda = 100;

    //unsigned int m = robot_->getNrJnts();
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
    Eigen::VectorXd q_dot = J_dag*Kpp*e_p +(I-J_dag*J)*q0_dot;



    return q_dot;
}


Eigen::VectorXd KDLController::compute_q0_dot(KDL::Frame &_desPos)
{
    // Stato
    Eigen::VectorXd q = robot_->getJntValues();      // n×1
    const int n = q.size();

    //Errore di posizione (solo traslazione)
    KDL::Frame cartpos = robot_->getEEFrame();
    KDL::Vector d = _desPos.p - cartpos.p;
    Eigen::Vector3d e_p(d.x(), d.y(), d.z());        // 3×1

    // Jacobiana (6×n) e parte lineare (3×n)
    KDL::Jacobian J_kdl = robot_->getEEJacobian();
    Eigen::MatrixXd J6 = J_kdl.data;    //Eigen::MatrixXd J_dag = J.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd J = J6.block(0, 0, 3, n);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd J_dag = J.transpose()*(J*J.transpose()).inverse();



    // Limiti

    Eigen::MatrixXd limits = robot_->getJntLimits();
    Eigen::VectorXd qmin = limits.col(0);
    Eigen::VectorXd qmax = limits.col(1);

    const double lambda = 100;

    //unsigned int m = robot_->getNrJnts();
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

    return q0_dot;
} // end q0_dot function


Eigen::MatrixXd KDLController::compute_J_cam(KDL::Chain chain_)
{
    KDL::Jacobian J_ee = robot_->getEEJacobian();

    // Position of the ArUco marker with respect to the camera
    KDL::Frame T_ee_cam = KDL::Frame(
        KDL::Rotation::RPY(3.14, -1.57, 0.0),
        KDL::Vector(0.0, 0.0, 0.17)
    );

    KDL::Jacobian J_cam(chain_.getNrOfJoints());
    J_cam.data = J_ee.data;

    KDL::Frame T_cam_ee = T_ee_cam.Inverse();

    J_cam.changeBase(T_cam_ee.M);
    //KDL::changeBase(J_cam, T_ee_cam.M);

    Eigen::MatrixXd J_c = J_cam.data;

    return J_c;
}


Eigen::VectorXd KDLController::vision_control(KDL::Frame &_desPos,
                                                geometry_msgs::msg::PoseStamped::ConstSharedPtr aruco_msg,
                                                KDL::Chain chain_,
                                                Eigen::MatrixXd K)
{
    // Numero di giunti
    Eigen::VectorXd q = robot_->getJntValues();      // n×1
    const int n = q.size();

    // s_d
    Eigen::Vector3d s_d {0, 0, 1};

    // q0_dot
    Eigen::VectorXd q0_dot = compute_q0_dot(_desPos);

    // P_c_o
    KDL::Frame aruco_frame = toKDL(aruco_msg->pose);
    Eigen::Vector3d _P_c_o(aruco_frame.p[0], aruco_frame.p[1], aruco_frame.p[2]);
    // R_c
    KDL::Rotation R_c= aruco_frame.M;

    // ||P||
    double p_norm = (double)_P_c_o.norm();
    // std::cout << "p_norm = " << p_norm << std::endl;

    // s
    Eigen::Vector3d s(0,0,1);
    if (p_norm > 1e-6) {
        s = _P_c_o / p_norm;
    }

    // J_cam
    Eigen::MatrixXd J_cam = compute_J_cam(chain_);

    // S(s)
    Eigen::Matrix3d S = skew(s);
        // {{0, -s(2), s(1)},
        //  {s(2), 0, -s(0)},
        //  {-s(1), s(0), 0}};
    // std::cout << "S = " << S << std::endl;

    // R
    // Supponiamo R sia una Matrix3d
    Eigen::Matrix3d R_c_eigen;  // 3x3 rotation matrix

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_c_eigen(i, j) = R_c(i, j);
        }
    }

    // Costruisci matrice 6x6: [R^T, 0; 0, R^T]
    Eigen::Matrix<double, 6, 6> R;
    R.setZero();  // Inizializza a zero

    R.block<3, 3>(0, 0) = R_c_eigen.transpose();  // Blocco alto-sinistra
    R.block<3, 3>(3, 3) = R_c_eigen.transpose();  // Blocco basso-destra

    // std::cout << "R = " << R << std::endl;

    // L
    Eigen::Matrix<double, 3, 6> L;
    L.setZero();

    L.block<3, 3>(0, 0) = -(1/p_norm)*(Eigen::Matrix3d::Identity() - s*s.transpose());
    L.block<3, 3>(0, 3) = S;
    L = L*R;
    // std::cout << "L = " << L << std::endl;

    double lambda = 0.01;
    Eigen::MatrixXd pseudoinverse = (L*J_cam).transpose()*((L*J_cam)*(L*J_cam).transpose() + lambda*Eigen::MatrixXd::Identity(3,3)).inverse();

    // N
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) -  pseudoinverse*L*J_cam;
    // std::cout << "N = " << N << std::endl;


    Eigen::VectorXd q_dot;
    q_dot = K*pseudoinverse*s_d + N*q0_dot;
    // std::cout << "q_dot = " << q_dot << std::endl;
    return q_dot;
}
