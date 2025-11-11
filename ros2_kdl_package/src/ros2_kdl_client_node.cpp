#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ros2_kdl_action_interface/action/ros2kdl.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

using ExecuteLinearTrajectory = ros2_kdl_action_interface::action::Ros2kdl;
using GoalHandleTraj = rclcpp_action::ClientGoalHandle<ExecuteLinearTrajectory>;


class Iiwa_action_client : public rclcpp::Node
{
    public:
        Iiwa_action_client()
        : Node("iiwa_action_client")
        {
            this->client_ptr_ = rclcpp_action::create_client<ExecuteLinearTrajectory>(this, "ros2kdl");

            // Read the goals from the parameters
            declare_parameter("traj_duration", 5.0);
            declare_parameter("acc_duration", 1.0);
            declare_parameter("total_time", 10.0);
            declare_parameter("trajectory_len", 500.0);
            declare_parameter("Kp", 1.5);
            declare_parameter("end_position", std::vector<double>{0.25, 0.1, 0.35});

            // declare traj_type parameter
            get_parameter("traj_duration", traj_duration);
            RCLCPP_INFO(get_logger(),"Current trajectory duration is: '%f'", traj_duration);
            // if (!(traj_type_ == "linear" || traj_trajectorytype_ == "circular"))
            // {
            //     RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            // }

            // declare  parameter 
            get_parameter("acc_duration", acc_duration);
            RCLCPP_INFO(get_logger(),"Current acceleration duration is: '%f'", acc_duration);
            // if (!(traj_type_ == "linear" || traj_trajectorytype_ == "circular"))
            // {
            //     RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            // }

            // declare traj_type parameter 
            get_parameter("total_time", total_time);
            RCLCPP_INFO(get_logger(),"Current total time is: '%f'", total_time);
            // if (!(traj_type_ == "linear" || traj_trajectorytype_ == "circular"))
            // {
            //     RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            // }

            // declare traj_type parameter
            get_parameter("trajectory_len", trajectory_len);
            RCLCPP_INFO(get_logger(),"Current trajectory length is: '%f'", trajectory_len);
            // if (!(traj_type_ == "linear" || traj_trajectorytype_ == "circular"))
            // {
            //     RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            // }

            // declare traj_type parameter
            get_parameter("Kp", Kp);
            RCLCPP_INFO(get_logger(),"Current Kp is: '%f'", Kp);
            // if (!(traj_type_ == "linear" || traj_trajectorytype_ == "circular"))
            // {
            //     RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            // }

            // declare traj_type parameter
            get_parameter("end_position", end_position);
            RCLCPP_INFO(get_logger(),"Current end_position is:");
            for(int i=0; i<3; i++){
                RCLCPP_INFO(get_logger(),"\tend_position[%d]: %f", i, end_position[i]);
            }
            // if (!(traj_type_ == "linear" || traj_trajectorytype_ == "circular"))
            // {
            //     RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            // }

            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&Iiwa_action_client::send_goal, this));
        }

        void send_goal()
        {
            using namespace std::placeholders;

            this->timer_->cancel(); // stop the timer

            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = ExecuteLinearTrajectory::Goal();
            // Specify the goal:
            goal_msg.traj_duration = traj_duration;
            goal_msg.acc_duration = acc_duration;
            goal_msg.total_time = total_time;
            goal_msg.trajectory_len = trajectory_len;
            goal_msg.kp = Kp;
            goal_msg.end_position[0] = end_position[0];
            goal_msg.end_position[1] = end_position[1];
            goal_msg.end_position[2] = end_position[2]; 

            RCLCPP_INFO(this->get_logger(), "Sending goals:\n"
                                            "\ttraj_duration = %f;\n"
                                            "\tacc_duration = %f;\n"
                                            "\ttotal_time = %f;\n"
                                            "\ttrajectory_len = %f;\n"
                                            "\tKp = %f;\n"
                                            "\tend_position = [%f, %f, %f]", 
                                            this->traj_duration,
                                            this->acc_duration,
                                            this->total_time,
                                            this->trajectory_len,
                                            this->Kp,
                                            this->end_position[0],
                                            this->end_position[1],
                                            this->end_position[2]);

            auto send_goal_options = rclcpp_action::Client<ExecuteLinearTrajectory>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&Iiwa_action_client::goal_response_callback, this, _1);
            send_goal_options.feedback_callback = std::bind(&Iiwa_action_client::feedback_callback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&Iiwa_action_client::result_callback, this, _1);
            // send goals
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<ExecuteLinearTrajectory>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        //Adding parameters
        double traj_duration;
        double acc_duration;
        double total_time;
        double trajectory_len;
        double Kp;
        std::vector<double> end_position;

        void goal_response_callback(const GoalHandleTraj::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(GoalHandleTraj::SharedPtr,
                                const std::shared_ptr<const ExecuteLinearTrajectory::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback: ||e_p|| = %f.", feedback->error_norm);
        }

        void result_callback(const GoalHandleTraj::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }
            RCLCPP_INFO(this->get_logger(), "Goal! \n\tResult: ||final_e_p|| = %f", result.result->final_error_norm);
            rclcpp::shutdown();
        }
};  // class Iiwa_action_client



int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_action_client>());
    rclcpp::shutdown();
    return 0;
}