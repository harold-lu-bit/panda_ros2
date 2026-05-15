// Copyright (c) 2026 Franka Robotics GmbH
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

#pragma once

#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>

namespace franka_robot_state_broadcaster {

/**
 * @brief Real-time safe single-producer/single-consumer triple buffer.
 *
 * The producer writes to the free buffer and commits it into the mailbox. The
 * consumer atomically swaps the latest mailbox buffer into its active slot.
 */
template <typename T>
class AsyncBuffer {
 public:
  template <typename... Args>
  explicit AsyncBuffer(const Args&... args) {
    for (auto& buffer : buffers_) {
      buffer = std::make_unique<T>(args...);
    }
  }

  T& get_free_buffer() {
    free_buffer_checked_out_ = true;
    return *buffers_[state_.load(std::memory_order_acquire).free_index];
  }

  bool commit_free_buffer() {
    if (!free_buffer_checked_out_) {
      return false;
    }

    State current = state_.load(std::memory_order_acquire);
    State next;
    while (true) {
      next.active_index = current.active_index;
      next.mailbox_index = current.free_index;
      next.free_index = current.mailbox_index;
      next.mailbox_full = true;

      if (state_.compare_exchange_weak(current, next, std::memory_order_release,
                                       std::memory_order_acquire)) {
        break;
      }
    }

    assert(next.is_consistent());
    free_buffer_checked_out_ = false;
    return true;
  }

  T& get_active_buffer(bool& has_new_data) {
    State current = state_.load(std::memory_order_acquire);
    State next = current;
    while (true) {
      next = current;
      if (next.mailbox_full) {
        std::swap(next.active_index, next.mailbox_index);
        next.mailbox_full = false;
      }

      if (state_.compare_exchange_weak(current, next, std::memory_order_release,
                                       std::memory_order_acquire)) {
        break;
      }
    }

    assert(next.is_consistent());
    has_new_data = current.mailbox_full;
    return *buffers_[next.active_index];
  }

 private:
  struct State {
    [[nodiscard]] bool is_consistent() const {
      return active_index < 3 && mailbox_index < 3 && free_index < 3 &&
             active_index != mailbox_index && mailbox_index != free_index &&
             free_index != active_index;
    }

    uint8_t active_index = 0;
    uint8_t mailbox_index = 1;
    uint8_t free_index = 2;
    bool mailbox_full = false;
  };

  std::atomic<State> state_{State{}};
  static_assert(std::atomic<State>::is_always_lock_free,
                "AsyncBuffer::State must be lock-free atomically swappable");

  bool free_buffer_checked_out_ = false;
  std::array<std::unique_ptr<T>, 3> buffers_;
};

}  // namespace franka_robot_state_broadcaster
