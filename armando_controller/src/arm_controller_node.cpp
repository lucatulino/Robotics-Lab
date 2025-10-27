#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;

class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm_controller_node"), cmd_index_(0)
    {
        this->declare_parameter<std::string>("controller_type", "position");
        controller_type_ = this->get_parameter("controller_type").as_string();

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmController::jointCallback, this, _1));

        if (controller_type_ == "position")
        {
            cmd_pub_position_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/position_controller/commands", 10);
        }
        else if (controller_type_ == "trajectory")
        {
            cmd_pub_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/joint_trajectory_controller/joint_trajectory", 10);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown controller_type: %s", controller_type_.c_str());
            throw std::runtime_error("Unknown controller_type");
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ArmController::publishCommands, this));

        this->declare_parameter<std::vector<double>>("cmd1", {0.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("cmd2", {0.5, 0.5, 0.5, 0.5});
        this->declare_parameter<std::vector<double>>("cmd3", {-0.5, -0.5, -0.5, -0.5});
        this->declare_parameter<std::vector<double>>("cmd4", {1.0, -1.0, 0.5, -0.5});
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Current joint positions:");
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), " %s: %.3f", msg->name[i].c_str(), msg->position[i]);
        }
    }

    void publishCommands()
    {
        std::vector<std::vector<double>> commands = {
            this->get_parameter("cmd1").as_double_array(),
            this->get_parameter("cmd2").as_double_array(),
            this->get_parameter("cmd3").as_double_array(),
            this->get_parameter("cmd4").as_double_array()
        };

        if (controller_type_ == "position")
        {
            std_msgs::msg::Float64MultiArray msg;
            msg.data = commands[cmd_index_];
            cmd_pub_position_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published POSITION command #%zu", cmd_index_ + 1);
        }
        else if (controller_type_ == "trajectory")
        {
            trajectory_msgs::msg::JointTrajectory traj_msg;
            traj_msg.joint_names = {"j0", "j1", "j2", "j3"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = commands[cmd_index_];
            point.time_from_start = rclcpp::Duration(2, 0); // 2 secondi

            traj_msg.points.push_back(point);
            cmd_pub_trajectory_->publish(traj_msg);
            RCLCPP_INFO(this->get_logger(), "Published TRAJECTORY command #%zu", cmd_index_ + 1);
        }

        cmd_index_ = (cmd_index_ + 1) % commands.size();
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_position_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_trajectory_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t cmd_index_;
    std::string controller_type_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

