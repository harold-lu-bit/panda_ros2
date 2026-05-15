// Copyright (c) 2023 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"

namespace {

geometry_msgs::msg::PoseStamped create_current_pose_stamped(
    const franka_msgs::msg::FrankaRobotState& franka_state_msg,
    const std::string& base_frame_name) {
  const Eigen::Map<const Eigen::Matrix4d> transformation_matrix(franka_state_msg.o_t_ee.data());
  const Eigen::Quaterniond quaternion(transformation_matrix.topLeftCorner<3, 3>());
  const Eigen::Translation3d translation(transformation_matrix.block<3, 1>(0, 3));

  geometry_msgs::msg::PoseStamped current_pose_stamped;
  current_pose_stamped.header = franka_state_msg.header;
  current_pose_stamped.header.frame_id = base_frame_name;
  current_pose_stamped.pose.position.x = translation.x();
  current_pose_stamped.pose.position.y = translation.y();
  current_pose_stamped.pose.position.z = translation.z();
  current_pose_stamped.pose.orientation.x = quaternion.x();
  current_pose_stamped.pose.orientation.y = quaternion.y();
  current_pose_stamped.pose.orientation.z = quaternion.z();
  current_pose_stamped.pose.orientation.w = quaternion.w();
  return current_pose_stamped;
}

geometry_msgs::msg::WrenchStamped create_external_wrench_in_stiffness_frame(
    const franka_msgs::msg::FrankaRobotState& franka_state_msg,
    const std::string& stiffness_frame_name) {
  geometry_msgs::msg::WrenchStamped wrench_in_stiffness_frame;
  wrench_in_stiffness_frame.header = franka_state_msg.header;
  wrench_in_stiffness_frame.header.frame_id = stiffness_frame_name;
  wrench_in_stiffness_frame.wrench.force.x = franka_state_msg.k_f_ext_hat_k.at(0);
  wrench_in_stiffness_frame.wrench.force.y = franka_state_msg.k_f_ext_hat_k.at(1);
  wrench_in_stiffness_frame.wrench.force.z = franka_state_msg.k_f_ext_hat_k.at(2);
  wrench_in_stiffness_frame.wrench.torque.x = franka_state_msg.k_f_ext_hat_k.at(3);
  wrench_in_stiffness_frame.wrench.torque.y = franka_state_msg.k_f_ext_hat_k.at(4);
  wrench_in_stiffness_frame.wrench.torque.z = franka_state_msg.k_f_ext_hat_k.at(5);
  return wrench_in_stiffness_frame;
}

}  // namespace

namespace franka_robot_state_broadcaster {

FrankaRobotStateBroadcaster::FrankaRobotStateBroadcaster(
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state)
    : franka_robot_state_(std::move(franka_robot_state)) {}

FrankaRobotStateBroadcaster::~FrankaRobotStateBroadcaster() {
  stopPublishThread();
}

void FrankaRobotStateBroadcaster::startPublishThread() {
  if (!is_publish_thread_running_) {
    bool has_stale_data = false;
    state_buffer_.get_active_buffer(has_stale_data);

    data_ready_.store(false, std::memory_order_relaxed);
    is_publish_thread_running_.store(true, std::memory_order_release);
    publish_thread_ = std::thread(&FrankaRobotStateBroadcaster::publishRunner, this);
  }
}

void FrankaRobotStateBroadcaster::stopPublishThread() {
  is_publish_thread_running_.store(false, std::memory_order_release);
  data_ready_.store(true, std::memory_order_release);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  data_ready_.store(false, std::memory_order_relaxed);
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
    auto_declare<std::string>("robot_description", "");
    param_listener = std::make_shared<ParamListener>(get_node());
    params = param_listener->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = franka_robot_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params = param_listener->get_params();
  std::string robot_description;
  if (!get_node()->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }

  if (!franka_robot_state_) {
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        params.arm_id + "/" + state_interface_name, robot_description);
  }

  current_pose_stamped_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      kCurrentPoseTopic, rclcpp::SystemDefaultsQoS());
  external_wrench_in_stiffness_frame_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
          kExternalWrenchInStiffnessFrame, rclcpp::SystemDefaultsQoS());
  try {
    franka_state_publisher = get_node()->create_publisher<franka_msgs::msg::FrankaRobotState>(
        "~/" + state_interface_name, rclcpp::SystemDefaultsQoS());
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  startPublishThread();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  stopPublishThread();
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  auto& franka_state_msg = state_buffer_.get_free_buffer();
  franka_state_msg.header.stamp = time;

  if (!franka_robot_state_->get_values_as_message(franka_state_msg)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get franka state via franka state interface.");
    return controller_interface::return_type::ERROR;
  }

  state_buffer_.commit_free_buffer();
  data_ready_.store(true, std::memory_order_release);

  return controller_interface::return_type::OK;
}

void FrankaRobotStateBroadcaster::publishRunner() {
  while (is_publish_thread_running_.load(std::memory_order_acquire)) {
    if (!data_ready_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::microseconds(kPublishThreadSleepUs));
      continue;
    }
    data_ready_.store(false, std::memory_order_relaxed);

    bool has_new_data = false;
    auto& franka_state_msg = state_buffer_.get_active_buffer(has_new_data);
    if (!has_new_data) {
      continue;
    }

    franka_state_publisher->publish(franka_state_msg);

    current_pose_stamped_publisher_->publish(
        create_current_pose_stamped(franka_state_msg, franka_robot_state_->get_base_frame_name()));
    external_wrench_in_stiffness_frame_publisher_->publish(
        create_external_wrench_in_stiffness_frame(franka_state_msg,
                                                  franka_robot_state_->get_stiffness_frame_name()));
  }
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
