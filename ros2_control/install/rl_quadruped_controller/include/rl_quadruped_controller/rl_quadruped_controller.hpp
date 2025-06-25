#pragma once

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <torch/script.h>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

struct CtrlInterfaces
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_torque_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_velocity_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kp_command_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kd_command_interface_;


    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_effort_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    imu_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    foot_force_state_interface_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    odom_state_interface_;

    CtrlInterfaces() = default;

    void clear()
    {
        joint_torque_command_interface_.clear();
        joint_position_command_interface_.clear();
        joint_velocity_command_interface_.clear();
        joint_kd_command_interface_.clear();
        joint_kp_command_interface_.clear();

        joint_effort_state_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        imu_state_interface_.clear();
        foot_force_state_interface_.clear();
    }
};

namespace rl_quadruped_controller
{

class RLQuadrupedController : public controller_interface::ControllerInterface
{
public:
  RLQuadrupedController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

  std::vector<float> get_current_pos();
  void sit(int step, std::vector<float> current_pos);
  void stand(int step, std::vector<float> current_pos);
  void move();

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  std::string mode_ = "sit";
  std::string prev_mode_;
  bool is_mode_change_ = false;

  std::string policy_path_;
  std::string config_path_;

  CtrlInterfaces ctrl_interfaces_;
  std::vector<std::string> joint_names_;
  std::string base_name_ = "base";
  std::vector<std::string> feet_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::string command_prefix_;

  // IMU Sensor
  std::string imu_name_;
  std::vector<std::string> imu_interface_types_;
  // Foot Force Sensor
  std::string foot_force_name_;
  std::vector<std::string> foot_force_interface_types_;

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
  command_interface_map_ = {
    {"position", &ctrl_interfaces_.joint_position_command_interface_},
    {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
    {"effort", &ctrl_interfaces_.joint_torque_command_interface_},
    {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
    {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
  };

  std::unordered_map<
    std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
  state_interface_map_ = {
    {"position", &ctrl_interfaces_.joint_position_state_interface_},
    {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_},
    {"effort", &ctrl_interfaces_.joint_effort_state_interface_}
  };

  std::vector<float> cmd_{0.0, 0.0, 0.0};
  std::vector<float> latest_cmd_{0.0, 0.0, 0.0};
  std::vector<float> initial_angles_, sit_angles_, default_angles_, cmd_scale_;
  float action_scale_{1.0}, ang_vel_scale_{1.0}, dof_pos_scale_{1.0}, dof_vel_scale_{1.0};

  torch::jit::script::Module policy_;
  torch::Tensor obs_buffer_;
  torch::Tensor prev_action_ = torch::zeros({12});

  int one_step_obs_size_{45}, obs_buf_size_{6};

                         
  std::vector<float> current_pos_;
  float steps_ = 150.0; 
  int step_ = 0;

  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace rl_quadruped_controller
