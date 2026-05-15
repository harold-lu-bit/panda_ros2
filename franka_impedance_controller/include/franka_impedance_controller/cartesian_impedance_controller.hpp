#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/float32.hpp>

#include <franka/robot_state.h>
#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>

#include "visibility_control.h"

namespace franka_impedance_controller {
class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Config methods

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  // ROS2 lifecycle related methods
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Main real-time method
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  bool k_elbow_activated{true};

  franka_msgs::msg::FrankaRobotState robot_state_;
  franka_msgs::msg::FrankaRobotState init_robot_state_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  std::string arm_id_;
  int num_joints{7};

  // param configuration
  void configure_parameters();

  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_j_d);
  Eigen::Matrix<double, 7, 1> saturateTorque(const Eigen::Matrix<double, 7, 1>& tau_d_calculated);

  // Nullspace exploration
  void updateNullspaceExploration(const Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian);

  // Classic cartesian controller
  double filter_params_{0.005};
  const double delta_tau_max_{1.0};
  const double tau_max_{6.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  double nullspace_stiffness_;
  double nullspace_stiffness_target_;
  double translational_clip_;
  double rotational_clip_;
  double q_limit_;
  Eigen::MatrixXd nullspace_base_;
  Eigen::JacobiSVD<Eigen::MatrixXd> nullspace_jacobi_svd_;
  bool enable_nullspace_joints_{false};
  bool enable_nullspace_joints_explore_{false};
  float nullspace_explore_dir_{0.0};
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_explore_error_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_nullspace_dir_;

  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void nullspaceDirCallback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;
  rcl_interfaces::msg::SetParametersResult param_callback(
      const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace franka_impedance_controller
