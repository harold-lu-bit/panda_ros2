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

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_params.hpp"
#include "franka/robot_state.h"
#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

using namespace franka_robot_state_broadcaster;

namespace {

std::string test_robot_description() {
  return R"(
<?xml version="1.0"?>
<robot name="panda">
  <link name="panda_link0"/>
  <link name="panda_link1"/>
  <link name="panda_link2"/>
  <link name="panda_link3"/>
  <link name="panda_link4"/>
  <link name="panda_link5"/>
  <link name="panda_link6"/>
  <link name="panda_link7"/>
  <link name="panda_link8"/>
  <joint name="panda_joint1" type="revolute">
    <parent link="panda_link0"/>
    <child link="panda_link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint2" type="revolute">
    <parent link="panda_link1"/>
    <child link="panda_link2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint3" type="revolute">
    <parent link="panda_link2"/>
    <child link="panda_link3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint4" type="revolute">
    <parent link="panda_link3"/>
    <child link="panda_link4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint5" type="revolute">
    <parent link="panda_link4"/>
    <child link="panda_link5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint6" type="revolute">
    <parent link="panda_link5"/>
    <child link="panda_link6"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint7" type="revolute">
    <parent link="panda_link6"/>
    <child link="panda_link7"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="panda_joint8" type="revolute">
    <parent link="panda_link7"/>
    <child link="panda_link8"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
)";
}

}  // namespace

class TestFrankaRobotStateBroadcaster : public ::testing::Test {
 protected:
  void SetUp() override {
    broadcaster_ = std::make_unique<FrankaRobotStateBroadcaster>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_broadcaster";
    params.robot_description = ros2_control_test_assets::minimal_robot_urdf;
    params.controller_manager_update_rate = 0;
    params.node_namespace = {};
    params.node_options = rclcpp::NodeOptions().enable_logger_service(true);
    broadcaster_->init(params);
    broadcaster_->get_node()->set_parameter(
        rclcpp::Parameter("robot_description", test_robot_description()));
  }

  void TearDown() override {
    if (interfaces_assigned_) {
      broadcaster_->release_interfaces();
    }
    state_interface_storage_.clear();
    broadcaster_.reset(nullptr);
  }

  void AssignFrankaStateInterface() {
    robot_state_.O_T_EE = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0, 0.1, 0.2, 0.3, 1.0};
    robot_state_.K_F_ext_hat_K = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    robot_state_.robot_mode = franka::RobotMode::kUserStopped;

    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interface_storage_.clear();
    state_interface_storage_.emplace_back(std::make_shared<hardware_interface::StateInterface>(
        "panda", "robot_state", reinterpret_cast<double*>(&robot_state_address_)));
    state_interfaces.emplace_back(state_interface_storage_.front());

    broadcaster_->assign_interfaces({}, std::move(state_interfaces));
    interfaces_assigned_ = true;
  }

  std::unique_ptr<FrankaRobotStateBroadcaster> broadcaster_;
  franka::RobotState robot_state_;
  franka::RobotState* robot_state_address_{&robot_state_};
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interface_storage_;
  bool interfaces_assigned_{false};
};

TEST_F(TestFrankaRobotStateBroadcaster, test_init_return_success) {
  EXPECT_EQ(broadcaster_->on_init(), controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_configure_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_activate_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_deactivate_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_repeated_activate_deactivate_return_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
  }
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_without_franka_state_interface_returns_error) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::ERROR);
}

TEST_F(TestFrankaRobotStateBroadcaster, test_update_with_franka_state_returns_success) {
  EXPECT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  AssignFrankaStateInterface();
  EXPECT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  rclcpp::Time time(0.0, 0.0);
  rclcpp::Duration period(0, 0);

  EXPECT_EQ(broadcaster_->update(time, period), controller_interface::return_type::OK);
  EXPECT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
