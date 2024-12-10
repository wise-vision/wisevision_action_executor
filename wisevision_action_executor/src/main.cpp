// Copyright (c) 2024, WiseVision. All rights reserved.
#include "automatic_action_service.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutomaticActionService>());
  rclcpp::shutdown();
  return 0;
}
