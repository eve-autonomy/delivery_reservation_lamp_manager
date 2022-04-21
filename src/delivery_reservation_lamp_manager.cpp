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
  sub_state_ = this->create_subscription<autoware_state_machine_msgs::msg::StateLock>(
    "/autoware_state_machine/lock_state",
    rclcpp::QoS{3}.transient_local(),
    std::bind(&DeliveryReservationLampManager::callbackStateMessage, this, std::placeholders::_1)
  );
  pub_delivery_reservation_lamp_ = this->create_publisher<dio_ros_driver::msg::DIOPort>(
    "delivery_reservation_lamp_out",
    rclcpp::QoS{3}.transient_local());

  active_polarity_ = ACTIVE_POLARITY;

  // Timer
  std::chrono::milliseconds timer_period_msec;
  timer_period_msec = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0));

  auto timer_callback = std::bind(&DeliveryReservationLampManager::lampBlinkOperationCallback, this);
  blink_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), timer_period_msec, std::move(timer_callback),
    this->get_node_base_interface()->get_context()
  );
  this->get_node_timers_interface()->add_timer(blink_timer_, nullptr);
  blink_timer_->cancel();

  publishLamp(false);
}

DeliveryReservationLampManager::~DeliveryReservationLampManager()
{
  publishLamp(false);
  blink_timer_->cancel();
}

void DeliveryReservationLampManager::callbackStateMessage(
  const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(), 1.0,
    "[DeliveryReservationLampManager::callbackStateMessage]state: %d", msg->state);

  blink_timer_->cancel();
  bool value = false;
  if (msg->state == autoware_state_machine_msgs::msg::StateLock::STATE_ON) {
    value = true;
  }
  publishLamp(value);
  if (msg->state == autoware_state_machine_msgs::msg::StateLock::STATE_VERIFICATION) {
    startLampBlinkOperation();
  }
}

void DeliveryReservationLampManager::publishLamp(const bool value)
{
  dio_ros_driver::msg::DIOPort msg;
  msg.value = active_polarity_ ? value : !value;
  pub_delivery_reservation_lamp_->publish(msg);
}

void DeliveryReservationLampManager::startLampBlinkOperation(void)
{
  blink_sequence_ = 0;
  double duration = getTimerDuration();

  setPeriod(duration);
}

double DeliveryReservationLampManager::getTimerDuration(void)
{
  if (blink_duration_table_.size() <= blink_sequence_) {
    blink_sequence_ = 0;
  }
  return blink_duration_table_.at(blink_sequence_);
}

void DeliveryReservationLampManager::lampBlinkOperationCallback(void)
{
  blink_timer_->cancel();

  blink_sequence_++;
  double duration = getTimerDuration();

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
