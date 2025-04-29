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

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace franka_robot_state_broadcaster {

// Override trylock to customize the locking mechanism
// You are excused for wondering why this is necessary.
// RealtimePublisher::lock() isn't suitable for a 1kHz messaging system - it sleeps for 200
// microseconds. RealtimePublisher::trylock() failure is highly likely due to the RCU >1kHz
// publish rate. Here we basically force the scheduler to yield our thread, simultaneously
// telling it reschedule us ASAP - hence not sleep_for(0) which doesn't necessarily yield.
// After 10 attempts, we give up. Hopefully, the next call to update() will be successful.
bool FrankaRobotStateBroadcaster::FrankaRobotStateRealtimePublisher::trylock() {
  int count{0};
  while (++count <= try_count_ &&
         !realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState>::trylock()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  return count <= try_count_;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
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

  franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
      franka_semantic_components::FrankaRobotState(params.arm_id + "/" + state_interface_name));

  current_pose_stamped_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      kCurrentPoseTopic, rclcpp::SystemDefaultsQoS());
  external_wrench_in_stiffness_frame_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
          kExternalWrenchInStiffnessFrame, rclcpp::SystemDefaultsQoS());
  try {
    franka_state_publisher = get_node()->create_publisher<franka_msgs::msg::FrankaRobotState>(
        "~/" + state_interface_name, rclcpp::SystemDefaultsQoS());
    realtime_franka_state_publisher =
        std::make_shared<FrankaRobotStateBroadcaster::FrankaRobotStateRealtimePublisher>(
            franka_state_publisher);
    ;
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
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  if (!realtime_franka_state_publisher || !realtime_franka_state_publisher->trylock()) {
    RCLCPP_WARN(get_node()->get_logger(),
                 "Failed to lock the realtime publisher after %d attempts",
                 realtime_franka_state_publisher->try_count());
    return controller_interface::return_type::ERROR;
  }
  realtime_franka_state_publisher->msg_.header.stamp = time;

  if (!franka_robot_state_->get_values_as_message(realtime_franka_state_publisher->msg_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                  "Failed to get franka state via franka state interface.");
    realtime_franka_state_publisher->unlock();
    return controller_interface::return_type::ERROR;
  }

  realtime_franka_state_publisher->unlockAndPublish();
  const auto& franka_state_msg = realtime_franka_state_publisher->msg_;

  const Eigen::Map<const Eigen::Matrix4d> transformation_matrix(franka_state_msg.o_t_ee_c.data());
  const Eigen::Quaterniond quaternion(transformation_matrix.topLeftCorner<3, 3>());
  const Eigen::Translation3d translation(transformation_matrix.block<3, 1>(0, 3));
  geometry_msgs::msg::PoseStamped current_pose_stamped;
  current_pose_stamped.header.stamp = time;
  current_pose_stamped.header.frame_id = k_end_effector_frame_;
  current_pose_stamped.pose.position = geometry_msgs::build<geometry_msgs::msg::Point>()
    .x(translation.x())
    .y(translation.y())
    .z(translation.z());
  current_pose_stamped.pose.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>()
    .x(quaternion.x())
    .y(quaternion.y())
    .z(quaternion.z())
    .w(quaternion.w());
  current_pose_stamped_publisher_->publish(current_pose_stamped);

  geometry_msgs::msg::WrenchStamped wrench_in_stiffness_frame;
  wrench_in_stiffness_frame.header.stamp = time;
  wrench_in_stiffness_frame.header.frame_id = k_stiffness_frame_;
  wrench_in_stiffness_frame.wrench.force = geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(franka_state_msg.k_f_ext_hat_k.at(0))
    .y(franka_state_msg.k_f_ext_hat_k.at(1))
    .z(franka_state_msg.k_f_ext_hat_k.at(2));
  wrench_in_stiffness_frame.wrench.torque = geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(franka_state_msg.k_f_ext_hat_k.at(3))
    .y(franka_state_msg.k_f_ext_hat_k.at(4))
    .z(franka_state_msg.k_f_ext_hat_k.at(5));
  external_wrench_in_stiffness_frame_publisher_->publish(wrench_in_stiffness_frame);

  return controller_interface::return_type::OK;
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
