// Copyright (c) 2026
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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace franka_gripper {

class GripperTopicNode : public rclcpp::Node {
 public:
  explicit GripperTopicNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("franka_gripper_topic_node", options) {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "");
    input_topic_ = declare_parameter<std::string>("input_topic", "gripper/openness_command");
    output_topic_ = declare_parameter<std::string>("output_topic", "gripper/openness");
    open_threshold_ = declare_parameter<double>("open_threshold", 0.8);
    close_threshold_ = declare_parameter<double>("close_threshold", 0.2);
    open_width_ = declare_parameter<double>("open_width", 0.08);
    open_speed_ = declare_parameter<double>("open_speed", 0.15);
    grasp_width_ = declare_parameter<double>("grasp_width", 0.0);
    grasp_speed_ = declare_parameter<double>("grasp_speed", 0.15);
    grasp_force_ = declare_parameter<double>("grasp_force", 20.0);
    epsilon_inner_ = declare_parameter<double>("epsilon_inner", 0.005);
    epsilon_outer_ = declare_parameter<double>("epsilon_outer", 0.005);
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);

    validateParameters();

    RCLCPP_INFO(get_logger(), "Trying to establish a connection with the gripper");
    try {
      gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
    } catch (const franka::Exception& exception) {
      RCLCPP_FATAL(get_logger(), "%s", exception.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "Connected to gripper");

    openness_publisher_ = create_publisher<std_msgs::msg::Float64>(output_topic_, 1);
    openness_subscription_ = create_subscription<std_msgs::msg::Float64>(
        input_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Float64::SharedPtr msg) { handleOpenness(msg->data); });
    publish_timer_ = create_wall_timer(rclcpp::Rate(publish_rate_).period(),
                                       [this]() { publishTargetOpenness(); });

    worker_thread_ = std::thread([this]() { workerLoop(); });
  }

  ~GripperTopicNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_requested_ = true;
      state_ = State::kStopping;
    }
    condition_.notify_all();

    if (gripper_) {
      try {
        gripper_->stop();
      } catch (const franka::Exception& exception) {
        RCLCPP_ERROR(get_logger(), "Failed to stop gripper during shutdown: %s", exception.what());
      }
    }

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    joinActionThread();
  }

 private:
  enum class Intent { kOpen, kClose };
  enum class State { kIdle, kOpening, kClosing, kStopping };

  void validateParameters() const {
    if (robot_ip_.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'robot_ip' not set");
      throw std::invalid_argument("Parameter 'robot_ip' not set");
    }
    if (open_threshold_ <= close_threshold_) {
      RCLCPP_FATAL(get_logger(), "open_threshold must be greater than close_threshold");
      throw std::invalid_argument("invalid gripper openness hysteresis thresholds");
    }
    if (publish_rate_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "publish_rate must be positive");
      throw std::invalid_argument("invalid publish_rate");
    }
  }

  void handleOpenness(double openness) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::optional<Intent> new_intent;
    if (openness > open_threshold_) {
      new_intent = Intent::kOpen;
    } else if (openness < close_threshold_) {
      new_intent = Intent::kClose;
    }

    if (!new_intent.has_value()) {
      return;
    }

    latest_intent_ = *new_intent;
    condition_.notify_all();
  }

  void publishTargetOpenness() {
    std_msgs::msg::Float64 msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_intent_.has_value()) {
        return;
      }
      msg.data = *latest_intent_ == Intent::kOpen ? 1.0 : 0.0;
    }
    openness_publisher_->publish(msg);
  }

  void workerLoop() {
    while (rclcpp::ok()) {
      Intent target_intent;
      std::optional<Intent> active_intent;
      bool should_stop_active_action = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {
          return shutdown_requested_ || action_finished_ ||
                 (latest_intent_.has_value() && latest_intent_ != active_intent_);
        });

        if (shutdown_requested_) {
          return;
        }

        if (action_finished_) {
          action_finished_ = false;
          action_running_ = false;
          if (action_failed_) {
            action_failed_ = false;
            active_intent_.reset();
          }
          if (state_ != State::kStopping) {
            state_ = State::kIdle;
          }
        }

        if (!latest_intent_.has_value() || latest_intent_ == active_intent_) {
          continue;
        }

        target_intent = *latest_intent_;
        active_intent = active_intent_;
        should_stop_active_action = active_intent.has_value() && action_running_;
        if (should_stop_active_action) {
          state_ = State::kStopping;
        }
      }

      if (should_stop_active_action) {
        stopActiveAction();
        joinActionThread();

        std::lock_guard<std::mutex> lock(mutex_);
        action_running_ = false;
        state_ = State::kIdle;
        if (!latest_intent_.has_value() || shutdown_requested_) {
          continue;
        }
        target_intent = *latest_intent_;
      }

      startAction(target_intent);
    }
  }

  void startAction(Intent intent) {
    joinActionThread();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_intent_ = intent;
      action_finished_ = false;
      action_running_ = true;
      state_ = intent == Intent::kOpen ? State::kOpening : State::kClosing;
    }

    action_thread_ = std::thread([this, intent]() {
      bool succeeded = false;
      if (intent == Intent::kOpen) {
        succeeded = executeOpen();
      } else {
        succeeded = executeClose();
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        action_failed_ = !succeeded;
        action_finished_ = true;
      }
      condition_.notify_all();
    });
  }

  bool executeOpen() {
    try {
      RCLCPP_INFO(get_logger(), "Opening gripper");
      if (!gripper_->move(open_width_, open_speed_)) {
        RCLCPP_ERROR(get_logger(), "Gripper move returned failure");
        return false;
      }
    } catch (const franka::Exception& exception) {
      RCLCPP_ERROR(get_logger(), "Gripper move failed: %s", exception.what());
      return false;
    }
    return true;
  }

  bool executeClose() {
    try {
      RCLCPP_INFO(get_logger(), "Closing gripper");
      if (!gripper_->grasp(grasp_width_, grasp_speed_, grasp_force_, epsilon_inner_,
                           epsilon_outer_)) {
        RCLCPP_ERROR(get_logger(), "Gripper grasp returned failure");
        return false;
      }
    } catch (const franka::Exception& exception) {
      RCLCPP_ERROR(get_logger(), "Gripper grasp failed: %s", exception.what());
      return false;
    }
    return true;
  }

  void stopActiveAction() {
    try {
      RCLCPP_INFO(get_logger(), "Stopping gripper before direction change");
      gripper_->stop();
    } catch (const franka::Exception& exception) {
      RCLCPP_ERROR(get_logger(), "Gripper stop failed: %s", exception.what());
    }
  }

  void joinActionThread() {
    if (action_thread_.joinable()) {
      action_thread_.join();
    }
  }

  std::string robot_ip_;
  std::string input_topic_;
  std::string output_topic_;
  double open_threshold_{0.8};
  double close_threshold_{0.2};
  double open_width_{0.08};
  double open_speed_{0.15};
  double grasp_width_{0.0};
  double grasp_speed_{0.15};
  double grasp_force_{20.0};
  double epsilon_inner_{0.005};
  double epsilon_outer_{0.005};
  double publish_rate_{30.0};

  std::unique_ptr<franka::Gripper> gripper_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr openness_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr openness_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<Intent> latest_intent_;
  std::optional<Intent> active_intent_;
  State state_{State::kIdle};
  bool action_finished_{false};
  bool action_failed_{false};
  bool action_running_{false};
  bool shutdown_requested_{false};

  std::thread worker_thread_;
  std::thread action_thread_;
};

}  // namespace franka_gripper

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<franka_gripper::GripperTopicNode>());
  rclcpp::shutdown();
  return 0;
}
