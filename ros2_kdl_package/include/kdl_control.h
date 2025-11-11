#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd velocity_control_null(KDL::Frame &_desPos, double _Kpp);

    Eigen::VectorXd vision_control (KDL::Frame &_desPos,
                                                geometry_msgs::msg::PoseStamped::ConstSharedPtr aruco_msg,
                                                KDL::Chain chain_,
                                                Eigen::MatrixXd K);

private:
    Eigen::VectorXd compute_q0_dot(KDL::Frame &_desPos);
                                                    
    Eigen::MatrixXd compute_J_cam(KDL::Chain chain_);

    KDLRobot* robot_;

};

#endif
