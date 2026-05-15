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

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_move_to_start_example_controller.hpp"

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

const double k_EPS = 1e-5;

void exp_val_near(const CommandInterface& cmdintf) {
  const double val = cmdintf.get_optional().value();
  EXPECT_NEAR(val, 0.0, k_EPS);
}

void MoveToStartExampleControllerTest::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void MoveToStartExampleControllerTest::TearDownTestSuite() {
  rclcpp::shutdown();
}

void MoveToStartExampleControllerTest::SetUp() {
  controller_ = std::make_unique<franka_example_controllers::MoveToStartExampleController>();
}

void MoveToStartExampleControllerTest::TearDown() {
  controller_.reset(nullptr);
  command_interface_storage_.clear();
  state_interface_storage_.clear();
}

void MoveToStartExampleControllerTest::SetUpController() {
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "test_move_to_start_example";
  params.robot_description = ros2_control_test_assets::minimal_robot_urdf;
  params.controller_manager_update_rate = 0;
  params.node_namespace = {};
  params.node_options = rclcpp::NodeOptions().enable_logger_service(true);
  const auto result = controller_->init(params);
  ASSERT_EQ(result, controller_interface::return_type::OK);
  std::vector<LoanedCommandInterface> command_ifs;
  std::vector<LoanedStateInterface> state_ifs;

  command_interface_storage_.clear();
  command_interface_storage_.reserve(joint_commands_.size());
  for (auto i = 0U; i < joint_commands_.size(); ++i) {
    command_interface_storage_.emplace_back(
        std::make_shared<CommandInterface>(joint_names_[i], HW_IF_EFFORT, &joint_commands_[i]));
  }
  for (const auto& command_interface : command_interface_storage_) {
    command_ifs.emplace_back(command_interface);
  }

  state_interface_storage_.clear();
  state_interface_storage_.reserve(joint_q_state_.size() + joint_dq_state_.size());
  for (auto i = 0U; i < joint_q_state_.size(); ++i) {
    state_interface_storage_.emplace_back(
        std::make_shared<StateInterface>(joint_names_[i], HW_IF_POSITION, &joint_q_state_[i]));
    state_interface_storage_.emplace_back(
        std::make_shared<StateInterface>(joint_names_[i], HW_IF_VELOCITY, &joint_dq_state_[i]));
  }
  for (const auto& state_interface : state_interface_storage_) {
    state_ifs.emplace_back(state_interface);
  }

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(MoveToStartExampleControllerTest, controller_gains_not_set_failure) {
  SetUpController();

  // Failure due to not set K, D gains
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, contoller_gain_empty) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", std::vector<double>()});

  // Failure due empty k gain parameter
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, contoller_damping_gain_empty) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", std::vector<double>()});

  // Failure due to empty d gain parameter
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, correct_controller_gains_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MoveToStartExampleControllerTest, correct_setup_on_activate_expect_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MoveToStartExampleControllerTest, correct_setup_on_update_expect_ok) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(controller_->update(time, duration), controller_interface::return_type::OK);

  exp_val_near(joint_1_pos_cmd_);
  exp_val_near(joint_2_pos_cmd_);
  exp_val_near(joint_3_pos_cmd_);
  exp_val_near(joint_4_pos_cmd_);
  exp_val_near(joint_5_pos_cmd_);
  exp_val_near(joint_6_pos_cmd_);
  exp_val_near(joint_7_pos_cmd_);
}
