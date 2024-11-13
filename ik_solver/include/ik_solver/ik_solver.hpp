#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>

#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"

class IKSolver : public rclcpp::Node
{
    public:
        IKSolver();
        ~IKSolver();

        void ee_pose_target_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void jnt_state_current_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void solve_ik();
    
    private:
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ee_pose_target_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jnt_state_current_sub_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jnt_state_result_pub_;

        KDL::Chain chain_;
        KDL::JntArray lb_, ub_;
        KDL::JntArray jnt_state_result_;
        KDL::JntArray jnt_state_current_;
        KDL::Frame ee_pose_target_;

        std::vector<KDL::JntArray> all_solutions;

        std::thread solve_ik_thread_;

        std::string urdf_xml_;
        std::string chain_start_;
        std::string chain_end_;

        double timeout_;
        double epsilon_;

        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> start_time;
        std::chrono::duration<double> diff;

        std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;

        std::mutex solve_ik_mutex_;

        bool jnt_state_current_ok_;
        bool ee_pose_target_ok_;

};