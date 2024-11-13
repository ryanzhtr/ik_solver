#include <iostream>
#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <memory>

#include <ik_solver/ik_solver.hpp>

IKSolver::IKSolver() : Node("iksolver")
{
    this->declare_parameter<double>("timeout", 0.005);
    this->declare_parameter<double>("epsilon", 1e-5);
    this->declare_parameters<std::string>(
        std::string(),
        std::map<std::string, std::string>{
            {"chain_start", std::string()},
            {"chain_end", std::string()},
            {"robot_description", std::string()},
        });

    this->get_parameter("timeout", this->timeout_);
    this->get_parameter("epsilon", this->epsilon_);
    this->get_parameter("chain_start", this->chain_start_);
    this->get_parameter("chain_end", this->chain_end_);
    this->get_parameter("robot_description", this->urdf_xml_);

    jnt_state_current_ok_ = false;
    ee_pose_target_ok_ = false;

    if (this->chain_start_.empty() || this->chain_end_.empty())
    {
        RCLCPP_FATAL(this->get_logger(), "Missing chain info in launch file");
        exit(-1);
    }

    tracik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(chain_start_, chain_end_, urdf_xml_, timeout_, epsilon_);

    bool valid = tracik_solver_->getKDLChain(chain_);

    if (!valid)
    {
        RCLCPP_ERROR(this->get_logger(), "There was no valid KDL chain found.");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Valid KDL chain found.");
    }

    valid = tracik_solver_->getKDLLimits(lb_, ub_);

    if (!valid)
    {
        RCLCPP_ERROR(this->get_logger(), "There was no valid KDL joint limits found.");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Valid KDL joint limits found.");
    }

    assert(chain_.getNrOfJoints() == lb_.data.size());
    assert(chain_.getNrOfJoints() == ub_.data.size());

    RCLCPP_INFO(this->get_logger(), "Using %d joints", chain_.getNrOfJoints());

    jnt_state_result_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/jnt_state_result", 100);

    ee_pose_target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/ee_pose_target", 10, std::bind(&IKSolver::ee_pose_target_callback, this, std::placeholders::_1));
    jnt_state_current_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&IKSolver::jnt_state_current_callback, this, std::placeholders::_1));

    solve_ik_thread_ = std::thread(&IKSolver::solve_ik, this);
}

IKSolver::~IKSolver()
{
    if (solve_ik_thread_.joinable())
    {
        solve_ik_thread_.join();
    }
}

void IKSolver::ee_pose_target_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // update target_ee_pose_
    std::lock_guard<std::mutex> guard(solve_ik_mutex_);
    // msg -> KDL::Frame
    KDL::Vector position(msg->position.x / 1000.0, msg->position.y / 1000.0, msg->position.z / 1000.0);

    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;

    KDL::Rotation rotation = KDL::Rotation::Quaternion(x, y, z, w);

    ee_pose_target_ = KDL::Frame(rotation, position);
}

void IKSolver::jnt_state_current_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // update jnt_state_current_
    std::lock_guard<std::mutex> guard(solve_ik_mutex_);
    // msg -> KDL::JntArray
    if (msg->position.size() != 7)
    {
        std::cout << jnt_state_current_.rows() << msg->position.size() << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("IKSolver"), "Received joint states do not match expected size");
        return;
    }

    jnt_state_current_.resize(msg->position.size());

    size_t i = 0;
    for (const auto &pos : msg->position)
    {
        jnt_state_current_(i++) = pos;
    }

    RCLCPP_INFO(rclcpp::get_logger("IKSolver"), "Updated current joint state");
}

void IKSolver::solve_ik()
{
    while (rclcpp::ok())
    {
        // std::cout << "Ready to solve inverse kinematics" << std::endl;
        start_time = std::chrono::system_clock::now();
        {
            std::lock_guard<std::mutex> guard(solve_ik_mutex_);

            if (jnt_state_current_.rows() != chain_.getNrOfJoints())
            {
                // RCLCPP_ERROR(this->get_logger(), "Invalid current joint state. Expected %d joints but got %d.",
                //              chain_.getNrOfJoints(), jnt_state_current_.rows());
                continue;
            }

            KDL::Vector position = ee_pose_target_.p;
            KDL::Rotation rotation = ee_pose_target_.M;

            if (position.Norm() == 0)
            {
                // RCLCPP_ERROR(this->get_logger(), "Invalid end-effector position for zero vector.");
                continue;
            }

            double x, y, z, w;
            rotation.GetQuaternion(x, y, z, w);
            double quaternion_norm = sqrt(x * x + y * y + z * z + w * w);

            if (std::abs(quaternion_norm - 1.0) > 1e-3)
            {
                // RCLCPP_ERROR(this->get_logger(), "Invalid end-effector orientation for non-normalized quaternion.");
                continue;
            }

            int rc = tracik_solver_->CartToJnt(jnt_state_current_, ee_pose_target_, jnt_state_result_);
            if (rc >= 0)
            {
                // Publish the result
                sensor_msgs::msg::JointState joint_state_msg;
                joint_state_msg.header.stamp = this->now();
                joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};

                for (unsigned int i = 0; i < jnt_state_result_.data.size(); ++i)
                {
                    joint_state_msg.position.push_back(jnt_state_result_(i));
                }

                jnt_state_result_pub_->publish(joint_state_msg);
            }

            else
            {
                RCLCPP_ERROR(this->get_logger(), "IK solution failed");
            }
        }

        diff = std::chrono::system_clock::now() - start_time;
        RCLCPP_INFO(this->get_logger(), "IK computation took %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<IKSolver>());
    rclcpp::shutdown();

    return 0;
}