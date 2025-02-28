/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ctime>
#include <chrono>

#include "lora_msgs/msg/full_date_time.hpp"
#include "lora_msgs/srv/automatic_action_connection.hpp"
#include "lora_msgs/msg/automatic_action_combined_request_and_time.hpp"

using AutomaticActionConnection = lora_msgs::srv::AutomaticActionConnection;
using FullDateTime = lora_msgs::msg::FullDateTime;
using AutomaticActionCombinedRequestAndTime = lora_msgs::msg::AutomaticActionCombinedRequestAndTime;


struct OneTopicData {
  bool action_triggered;
  rclcpp::Time time_stamp;
  int data_validity_ms;
  FullDateTime time_stamp_local;
};

struct CombinedTopicsPublisher {
  AutomaticActionCombinedRequestAndTime config;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
};

#endif // DATA_STRUCTURES_HPP
