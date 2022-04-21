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

namespace delivery_reservation_lamp_manager
{
class DeliveryReservationLampManager : public rclcpp::Node
{
public:
  explicit DeliveryReservationLampManager(const rclcpp::NodeOptions & options);
  ~DeliveryReservationLampManager();

  // Publisher
  rclcpp::Publisher<dio_ros_driver::msg::DIOPort>::SharedPtr pub_delivery_reservation_lamp_;

  // Subscription
  rclcpp::Subscription<autoware_state_machine_msgs::msg::StateLock>::SharedPtr sub_state_;

  #define BLINK_ON_DURATION (1.0)
  #define BLINK_OFF_DURATION (1.0)
  #define ACTIVE_POLARITY (false)

  std::array<double, 2> blink_duration_table_ = {
    BLINK_OFF_DURATION,
    BLINK_ON_DURATION
  };

  rclcpp::TimerBase::SharedPtr blink_timer_;
  uint64_t blink_sequence_;
  bool active_polarity_;

  void callbackStateMessage(const autoware_state_machine_msgs::msg::StateLock::ConstSharedPtr msg);
  void publishLamp(const bool value);
  void startLampBlinkOperation(void);
  double getTimerDuration(void);
  void lampBlinkOperationCallback(void);
  void setPeriod(const double new_period);
};

}  // namespace delivery_reservation_lamp_manager
#endif  // DELIVERY_RESERVATION_LAMP_MANAGER__DELIVERY_RESERVATION_LAMP_MANAGER_HPP_
