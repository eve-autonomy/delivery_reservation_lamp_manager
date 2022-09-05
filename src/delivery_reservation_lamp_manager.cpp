// Copyright 2021 eve autonomy inc. All Rights Reserved.
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
// limitations under the License

#include <memory>
#include <utility>
#include <array>
#include "delivery_reservation_lamp_manager/delivery_reservation_lamp_manager.hpp"

namespace delivery_reservation_lamp_manager
{

DeliveryReservationLampManager::DeliveryReservationLampManager(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: Node("delivery_reservation_lamp_manager", options)
{
  receive_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  current_reservation_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  receive_shutdown_state_ = shutdown_manager_msgs::msg::StateShutdown::STATE_INACTIVE_FOR_SHUTDOWN;
  current_shutdown_state_ = shutdown_manager_msgs::msg::StateShutdown::STATE_INACTIVE_FOR_SHUTDOWN;

  sub_reservation_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateLock>(
    "/autoware_state_machine/lock_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&DeliveryReservationLampManager::callbackReservationStateMessage, this, std::placeholders::_1)
  );
  sub_shutdown_state_ = this->create_subscription<shutdown_manager_msgs::msg::StateShutdown>(
    "/shutdown_manager/state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&DeliveryReservationLampManager::callbackShutdownStateMessage, this, std::placeholders::_1)
  );
  pub_delivery_reservation_lamp_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "delivery_reservation_lamp_out",
    rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;

  // Main Proc Timer
  std::chrono::milliseconds main_proc_timer_period_msec;
  main_proc_timer_period_msec = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / 10.0));
  
  auto main_proc_timer_callback = std::bind(&DeliveryReservationLampManager::onTimer, this);
  main_proc_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(main_proc_timer_callback)>>(
    this->get_clock(), main_proc_timer_period_msec, std::move(main_proc_timer_callback),
    this->get_node_base_interface()->get_context()
  );

  // Blink Timer
  std::chrono::milliseconds blink_timer_period_msec;
  blink_timer_period_msec = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0));

  auto blink_timer_callback = std::bind(&DeliveryReservationLampManager::lampBlinkOperationCallback, this);
  blink_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(blink_timer_callback)>>(
    this->get_clock(), blink_timer_period_msec, std::move(blink_timer_callback),
    this->get_node_base_interface()->get_context()
  );

  this->get_node_timers_interface()->add_timer(main_proc_timer_, nullptr);
  this->get_node_timers_interface()->add_timer(blink_timer_, nullptr);
  blink_timer_->cancel();

  publishLamp(false);
}

DeliveryReservationLampManager::~DeliveryReservationLampManager()
{
  publishLamp(false);
  blink_timer_->cancel();
}

void DeliveryReservationLampManager::callbackReservationStateMessage(
  const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[DeliveryReservationLampManager::callbackReservationStateMessage]state: %d", msg->state);

  receive_reservation_state_ = msg->state;
}

void DeliveryReservationLampManager::callbackShutdownStateMessage(
  const shutdown_manager_msgs::msg::StateShutdown::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[DeliveryReservationLampManager::callbackShutdownStateMessage]state: %d", msg->state);

  receive_shutdown_state_ = msg->state;
}

void DeliveryReservationLampManager::onTimer(void)
{
  if ((receive_reservation_state_ == current_reservation_state_) &&
    (receive_shutdown_state_ == current_shutdown_state_)) {
    return;
  }
  current_reservation_state_ = receive_reservation_state_;
  current_shutdown_state_ = receive_shutdown_state_;

  blink_timer_->cancel();
  if ((current_shutdown_state_ == shutdown_manager_msgs::msg::StateShutdown::STATE_STANDBY_FOR_SHUTDOWN) ||
    (current_shutdown_state_ == shutdown_manager_msgs::msg::StateShutdown::STATE_START_OF_SHUTDOWN)) {
    startLampBlinkOperation(BlinkType::FAST_BLINK, BLINK_INDENFINITELY);
  } else if (current_shutdown_state_ == shutdown_manager_msgs::msg::StateShutdown::STATE_SUCCESSFUL_SHUTDOWN_INITIATION) {
    startLampBlinkOperation(BlinkType::TWO_BLINKS_UNTIL_EXPIRATION, TWO_BLINKS_MAX_RETRY_COUNT);
  } else {
    if (current_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_OFF) {
      publishLamp(false);
    } else if (current_reservation_state_ == autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION) {
      startLampBlinkOperation(BlinkType::SLOW_BLINK, BLINK_INDENFINITELY);
    } else {
      publishLamp(true);
    }
  }
}

void DeliveryReservationLampManager::publishLamp(const bool value)
{
  dio_ros_driver::msg::DIOPort msg;
  msg.value = active_polarity_ ? value : !value;
  pub_delivery_reservation_lamp_->publish(msg);
}

void DeliveryReservationLampManager::startLampBlinkOperation(const BlinkType type, const int max_blink_retry_count)
{
  publishLamp(true);
  blink_sequence_ = 0;
  blink_retry_count_ = 0;
  max_blink_retry_count_ = max_blink_retry_count;
  blink_type_ = type;
  double duration = getTimerDuration();

  setPeriod(duration);
}

double DeliveryReservationLampManager::getTimerDuration(void)
{
  if (blink_type_ == BlinkType::FAST_BLINK) {
    if (fast_blink_duration_table_.size() <= blink_sequence_) {
      blink_retry_count_++;
      blink_sequence_ = 0;
    }
    return fast_blink_duration_table_.at(blink_sequence_);
  } else if (blink_type_ == BlinkType::TWO_BLINKS_UNTIL_EXPIRATION) {
    if (two_blinks_duration_table_.size() <= blink_sequence_) {
      blink_retry_count_++;
      blink_sequence_ = 0;
    }
    return two_blinks_duration_table_.at(blink_sequence_);
  } else {
    if (slow_blink_duration_table_.size() <= blink_sequence_) {
      blink_retry_count_++;
      blink_sequence_ = 0;
    }
    return slow_blink_duration_table_.at(blink_sequence_);
  }
}

void DeliveryReservationLampManager::lampBlinkOperationCallback(void)
{
  blink_timer_->cancel();

  blink_sequence_++;
  double duration = getTimerDuration();

  // If the maximum count of retries is exceeded,
  //  the LED will turn off and stop blinking.
  if ((max_blink_retry_count_ != BLINK_INDENFINITELY) &&
    (blink_retry_count_ >= max_blink_retry_count_)) {
    publishLamp(false);
    return;
  }

  // odd sequence -> true : even sequence -> false
  publishLamp(blink_sequence_ % 2 ? true : false);

  setPeriod(duration);
}

void DeliveryReservationLampManager::setPeriod(const double new_period)
{
  int64_t old_period = 0;
  std::chrono::nanoseconds period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(new_period));

  int64_t timer_period = period.count();

  rcl_ret_t ret = rcl_timer_exchange_period(
    blink_timer_->get_timer_handle().get(), timer_period, &old_period);
  if (ret != RCL_RET_OK) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1.0,
      "Couldn't exchange_period");
  }
  blink_timer_->reset();
}

}  // namespace delivery_reservation_lamp_manager

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(delivery_reservation_lamp_manager::DeliveryReservationLampManager)
