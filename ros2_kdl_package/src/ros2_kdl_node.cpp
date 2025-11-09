// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
/* additional include for point 1c */
#include <functional>
#include <thread>
#include "ros2_kdl_action_interface/action/ros2kdl.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"
// #include "action_tutorials_cpp/visibility_control.h" // for Windows but lo possiamo levare

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using namespace std::placeholders;

using ExecuteLinearTrajectory = ros2_kdl_action_interface::action::Ros2kdl;
using GoalHandleTraj = rclcpp_action::ServerGoalHandle<ExecuteLinearTrajectory>;


class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        //ACTION_TUTORIALS_CPP_BUILDING_DLL
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"),
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter
            declare_parameter("cmd_interface", "velocity_ctrl_null"); 

            // declare_parameter("traj_duration", 5.0);
            // declare_parameter("acc_duration", 1.0);
            // declare_parameter("total_time", 2.0);
            // declare_parameter("trajectory_len", 500.0);
            // declare_parameter("Kp", 1.5);
            // declare_parameter("end_position", std::vector<double>{0.25, 0.0, 0.35});

            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity_ctrl" || cmd_interface_ == "effort" || cmd_interface_ == "velocity_ctrl_null"))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity_ctrl', 'effort' or 'velocity_ctrl_null' instead..."); return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            iteration_ = 0; 
            t_ = 0;
            joint_state_available_ = false; 

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); 
            joint_efforts_cmd_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 
                10, 
                std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;


            // Create an action server
            this->action_server_ = rclcpp_action::create_server<ExecuteLinearTrajectory>(
                this,
                "ros2kdl",
                std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
                std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
                std::bind(&Iiwa_pub_sub::handle_accepted, this, _1));            

            // Creating the publisher
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                //                             std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity_ctrl" || cmd_interface_ == "velocity_ctrl_null"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                //                             std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                //                             std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }  

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        }

    private:
        rclcpp_action::Server<ExecuteLinearTrajectory>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                                std::shared_ptr<const ExecuteLinearTrajectory::Goal> goal)
        {
            if (goal->end_position.size() < 3) {
                RCLCPP_ERROR(get_logger(), "Parametro 'end_position' ha dimensione < 3");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (goal->traj_duration <= 0.0 || goal->acc_duration < 0.0 || goal->acc_duration > goal->traj_duration/2.0) {
                RCLCPP_ERROR(get_logger(), "Parametri tempo non validi (T=%.3f, Ta=%.3f)", goal->traj_duration, goal->acc_duration);
                return rclcpp_action::GoalResponse::REJECT;
            }

            RCLCPP_INFO(this->get_logger(), "Received goal request with:\n"
                                            "\ttraj_duration = %f;\n"
                                            "\tacc_duration = %f;\n"
                                            "\ttotal_time = %f;\n"
                                            "\ttrajectory_len = %f;\n"
                                            "\tKp = %f;\n"
                                            "\tend_position = [%f, %f, %f]", 
                                            goal->traj_duration,
                                            goal->acc_duration,
                                            goal->total_time,
                                            goal->trajectory_len,
                                            goal->kp,
                                            goal->end_position[0],
                                            goal->end_position[1],
                                            goal->end_position[2]);


            (void)uuid; 
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTraj> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle; 
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTraj> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&Iiwa_pub_sub::cmd_publisher, this, _1), goal_handle}.detach();
        }

        void cmd_publisher(const std::shared_ptr<GoalHandleTraj> goal_handle){
            iteration_ = iteration_ + 1;

            KDLController controller_(*robot_);

            RCLCPP_INFO(this->get_logger(), "Executing goal");

            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<ExecuteLinearTrajectory::Feedback>();
            auto & p_err_norm = feedback->error_norm;
            auto result = std::make_shared<ExecuteLinearTrajectory::Result>();

            // Getting goals
            this->total_time = goal->total_time;
            this->trajectory_len = goal->trajectory_len; 
            this->traj_duration = goal->traj_duration;
            this->acc_duration = goal->acc_duration;
            this->Kp = goal->kp;
            this->end_position.resize(3);
            this->end_position[0] = goal->end_position[0];
            this->end_position[1] = goal->end_position[1];
            this->end_position[2] = goal->end_position[2];

            // define trajectory
            int freq = trajectory_len / total_time;
            double dt = 1.0 / freq;

            rclcpp::Rate loop_rate(freq);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y). Taken by yaml file
            Eigen::Vector3d end_position_; 
            end_position_ << goal->end_position[0], -goal->end_position[1], goal->end_position[2];

            // Plan trajectory
            double traj_radius = 0.15;
            // Retrieve the first trajectory point
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner(goal->traj_duration, goal->acc_duration, init_position, end_position_); // currently using trapezoidal velocity profile
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(goal->traj_duration, init_position, traj_radius, goal->acc_duration);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }

            while(rclcpp::ok() && t_ < total_time){
                // time update
                t_+=dt;

                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    result->final_error_norm = p_err_norm;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Trajectory goal canceled.");
                    return;
                }
                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; 
                desFrame.M = cartpos.M; 
                desFrame.p = toKDL(p_.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                p_err_norm = (double)error.norm();
                std::cout << "||e_p("<< t_ <<")|| = " << p_err_norm << std::endl;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; 
                    nextFrame.M = cartpos.M; 
                    nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity_ctrl"){
                    // Compute differential IK
                    Vector6d cartvel; 
                    cartvel << p_.vel + Kp*error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }
                else if(cmd_interface_ == "velocity_ctrl_null"){ 
                    std::cout << "Running velocity control null." << std::endl;
                    joint_velocities_cmd_.data = controller_.velocity_control_null(desFrame, Kp);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity_ctrl" || cmd_interface_ == "velocity_ctrl_null"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback: error_norm = %f", p_err_norm);

                loop_rate.sleep();
            }

            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                    
            // Stop it
            // Send joint velocity commands
            if(cmd_interface_ == "position"){
                // Set joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if(cmd_interface_ == "velocity_ctrl" || cmd_interface_ == "velocity_ctrl_null"){
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
            }
            else if(cmd_interface_ == "effort"){
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }
            
            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            // Check if goal is done
            if (rclcpp::ok()) {
                result->final_error_norm = p_err_norm;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal! Trajectory executed successfully!");
            }


        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        
        //Adding parameters
        double traj_duration;
        double acc_duration;
        double total_time;
        double trajectory_len;
        double Kp;
        std::vector<double> end_position;

        KDL::Frame init_cart_pose_;
}; // end class Iiwa


 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}