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

#ifndef DELIVERY_RESERVATION_LAMP_MANAGER__DELIVERY_RESERVATION_LAMP_MANAGER_HPP_
#define DELIVERY_RESERVATION_LAMP_MANAGER__DELIVERY_RESERVATION_LAMP_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "autoware_state_machine_msgs/msg/state_lock.hpp"
#include "dio_ros_driver/msg/dio_port.hpp"
#include "shutdown_manager_msgs/msg/state_shutdown.hpp"

namespace delivery_reservation_lamp_manager
{
class DeliveryReservationLampManager : public rclcpp::Node
{
public:
  explicit DeliveryReservationLampManager(const rclcpp::NodeOptions & options);
  ~DeliveryReservationLampManager();

  enum BlinkType
  {
    FAST_BLINK,
    SLOW_BLINK,
    TWO_BLINKS
  };

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_delivery_reservation_lamp_;

  // Subscription
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateLock>::SharedPtr sub_reservation_lock_state_;
  rclcpp::Subscription<shutdown_manager_msgs::msg::StateShutdown>::SharedPtr sub_shutdown_state_;

  #define TWO_BLINKS_ON_DURATION (0.2)
  #define TWO_BLINKS_OFF_DURATION (0.2)
  #define TWO_BLINKS_IDLE_DURATION (1.5)
  #define BLINK_FAST_ON_DURATION (0.5)
  #define BLINK_FAST_OFF_DURATION (0.5)
  #define BLINK_SLOW_ON_DURATION (1.0)
  #define BLINK_SLOW_OFF_DURATION (1.0)
  #define ACTIVE_POLARITY (false)

  std::array<double, 4> two_blinks_duration_table_ = {
    TWO_BLINKS_IDLE_DURATION,
    TWO_BLINKS_ON_DURATION,
    TWO_BLINKS_OFF_DURATION,
    TWO_BLINKS_ON_DURATION
  };

  std::array<double, 2> fast_blink_duration_table_ = {
    BLINK_FAST_OFF_DURATION,
    BLINK_FAST_ON_DURATION
  };

  std::array<double, 2> slow_blink_duration_table_ = {
    BLINK_SLOW_OFF_DURATION,
    BLINK_SLOW_ON_DURATION
  };

  rclcpp::TimerBase::SharedPtr main_proc_timer_;
  rclcpp::TimerBase::SharedPtr blink_timer_;
  uint64_t blink_sequence_;
  BlinkType blink_type_;
  bool active_polarity_;
  uint16_t receive_reservation_lock_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  uint16_t current_reservation_lock_state_ = autoware_state_machine_msgs::msg::StateLock::STATE_OFF;
  uint16_t receive_shutdown_state_ = shutdown_manager_msgs::msg::StateShutdown::STATE_INACTIVE_FOR_SHUTDOWN;
  uint16_t current_shutdown_state_ = shutdown_manager_msgs::msg::StateShutdown::STATE_INACTIVE_FOR_SHUTDOWN;

  void callbackReservationStateMessage(const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg);
  void callbackShutdownStateMessage(const shutdown_manager_msgs::msg::StateShutdown::ConstSharedPtr msg);
  void onTimer(void);
  void publishLamp(const bool value);
  void startLampBlinkOperation(const BlinkType type);
  double getTimerDuration(void);
  void lampBlinkOperationCallback(void);
  void setPeriod(const double new_period);
};

}  // namespace delivery_reservation_lamp_manager
#endif  // DELIVERY_RESERVATION_LAMP_MANAGER__DELIVERY_RESERVATION_LAMP_MANAGER_HPP_
