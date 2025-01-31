/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "automatic_action_service.hpp"
#include "lora_msgs/srv/automatic_action.hpp"
#include "lora_msgs/srv/automatic_action_combined_delete.hpp"
#include "lora_msgs/srv/automatic_action_delete.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"

class AutomaticActionServiceTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<AutomaticActionService> action_service;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("automatic_action_service_test_node");

    action_service = std::make_shared<AutomaticActionService>();
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(action_service);

    std::thread([this]() { executor->spin(); }).detach();
  }

  void TearDown() override {
    executor->cancel();
    rclcpp::shutdown();
  }

  template <typename ServiceT, typename RequestT>
  typename ServiceT::Response::SharedPtr callService(const std::string& service_name,
                                                     typename RequestT::SharedPtr request) {
    auto client = node->create_client<ServiceT>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(node->get_logger(), "Service %s not available", service_name.c_str());
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
      return future.get();
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", service_name.c_str());
      return nullptr;
    }
  }
};

TEST_F(AutomaticActionServiceTest, HandleCreateServiceTest) {
  auto request = std::make_shared<AutomaticAction::Request>();
  request->listen_topic = "test_topic";
  request->listen_message_type = "std_msgs/String";
  request->action_and_publisher_name = "action_and_publisher_name";
  request->value = "field_value";
  request->trigger_val = "10";
  request->trigger_type = "GreaterThan";
  request->data_validity_ms = 1000;

  auto response = callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleCreateServiceWithEmptyFieldsTest) {
  auto request = std::make_shared<AutomaticAction::Request>();
  request->listen_topic = "";
  request->listen_message_type = "std_msgs/String";
  request->action_and_publisher_name = "action_and_publisher_name";
  request->value = "field_value";
  request->trigger_val = "10";
  request->trigger_type = "GreaterThan";
  request->data_validity_ms = 1000;

  auto response = callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_FALSE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleDeleteServiceTest) {
  auto create_request = std::make_shared<AutomaticAction::Request>();
  create_request->listen_topic = "test_topic";
  create_request->listen_message_type = "std_msgs/String";
  create_request->action_and_publisher_name = "action_and_publisher_name";
  create_request->value = "field_value";
  create_request->trigger_val = "10";
  create_request->trigger_type = "GreaterThan";
  create_request->data_validity_ms = 1000;

  callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", create_request);

  auto delete_request = std::make_shared<AutomaticActionDelete::Request>();
  delete_request->listen_topic_to_delete = "action_and_publisher_name";

  auto delete_response =
      callService<AutomaticActionDelete, AutomaticActionDelete::Request>("delete_automatic_action", delete_request);

  ASSERT_TRUE(delete_response != nullptr);
  EXPECT_TRUE(delete_response->success);
}

TEST_F(AutomaticActionServiceTest, HandleDeleteServiceWithEmptyTopicTest) {
  auto request = std::make_shared<AutomaticActionDelete::Request>();
  request->listen_topic_to_delete = "";

  auto response =
      callService<AutomaticActionDelete, AutomaticActionDelete::Request>("delete_automatic_action", request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_FALSE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleDeleteServiceWithExistingCombinedServiceTest) {
  auto request = std::make_shared<AutomaticAction::Request>();
  request->listen_topic = "test_topic";
  request->listen_message_type = "std_msgs/String";
  request->action_and_publisher_name = "action_and_publisher_name";
  request->value = "field_value";
  request->trigger_val = "10";
  request->trigger_type = "GreaterThan";
  request->data_validity_ms = 1000;

  auto response = callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", request);

  auto request_second = std::make_shared<AutomaticAction::Request>();
  request_second->listen_topic = "test_topic";
  request_second->listen_message_type = "std_msgs/String";
  request_second->action_and_publisher_name = "action_and_publisher_name_second";
  request_second->value = "field_value";
  request_second->trigger_val = "10";
  request_second->trigger_type = "GreaterThan";
  request_second->data_validity_ms = 1000;

  auto response_second =
      callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", request_second);

  auto combined_request = std::make_shared<AutomaticActionConnection::Request>();
  combined_request->action_and_publisher_name = "combined_action_and_publisher_name";
  combined_request->listen_topics = {"action_and_publisher_name", "action_and_publisher_name_second"};
  combined_request->logic_expression = "action_and_publisher_name and action_and_publisher_name_second";
  combined_request->trigger_text = "Triggering combined action";
  combined_request->publication_method = 1;
  ;

  callService<AutomaticActionConnection, AutomaticActionConnection::Request>("create_combined_automatic_action",
                                                                             combined_request);

  auto delete_request = std::make_shared<AutomaticActionDelete::Request>();
  delete_request->listen_topic_to_delete = "action_and_publisher_name";

  auto delete_response =
      callService<AutomaticActionDelete, AutomaticActionDelete::Request>("delete_automatic_action", delete_request);

  EXPECT_FALSE(delete_response->success);
}

TEST_F(AutomaticActionServiceTest, HandleCombinedServiceTest) {
  auto request = std::make_shared<AutomaticActionConnection::Request>();
  request->action_and_publisher_name = "combined_action_and_publisher_name";
  request->listen_topics = {"action_and_publisher_name1", "action_and_publisher_name2"};
  request->logic_expression = "action_and_publisher_name1 and action_and_publisher_name2";
  request->trigger_text = "Triggering combined action";
  request->publication_method = 1;

  auto topic1_request = std::make_shared<AutomaticAction::Request>();
  topic1_request->listen_topic = "topic1";
  topic1_request->listen_message_type = "std_msgs/String";
  topic1_request->action_and_publisher_name = "action_and_publisher_name1";
  topic1_request->value = "field_value";
  topic1_request->trigger_val = "10";
  topic1_request->trigger_type = "GreaterThan";
  topic1_request->data_validity_ms = 1000;

  auto topic2_request = std::make_shared<AutomaticAction::Request>();
  topic2_request->listen_topic = "topic2";
  topic2_request->listen_message_type = "std_msgs/String";
  topic2_request->action_and_publisher_name = "action_and_publisher_name2";
  topic2_request->value = "field_value";
  topic2_request->trigger_val = "10";
  topic2_request->trigger_type = "GreaterThan";
  topic2_request->data_validity_ms = 1000;

  callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", topic1_request);
  callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", topic2_request);

  auto response =
      callService<AutomaticActionConnection, AutomaticActionConnection::Request>("create_combined_automatic_action",
                                                                                 request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleCombinedServiceWithInsufficientTopicsTest) {
  auto request = std::make_shared<AutomaticActionConnection::Request>();
  request->action_and_publisher_name = "combined_action_and_publisher_name";
  request->listen_topics = {"topic1"};
  request->logic_expression = "topic1";
  request->trigger_text = "Action triggered!";
  request->publication_method = 1;

  auto response =
      callService<AutomaticActionConnection, AutomaticActionConnection::Request>("create_combined_automatic_action",
                                                                                 request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_FALSE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleCombinedServiceWithEmptyLogicExpressionTest) {
  auto request = std::make_shared<AutomaticActionConnection::Request>();
  request->action_and_publisher_name = "combined_action_and_publisher_name";
  request->listen_topics = {"topic1", "topic2"};
  request->logic_expression = "";
  request->trigger_text = "Action triggered!";
  request->publication_method = 1;

  auto response =
      callService<AutomaticActionConnection, AutomaticActionConnection::Request>("create_combined_automatic_action",
                                                                                 request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_FALSE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleCombinedDeleteServiceTest) {
  auto combined_request = std::make_shared<AutomaticActionConnection::Request>();
  combined_request->action_and_publisher_name = "combined_action_and_publisher_name";
  combined_request->listen_topics = {"topic1", "topic2"};
  combined_request->logic_expression = "topic1 and topic2";

  callService<AutomaticActionConnection, AutomaticActionConnection::Request>("create_combined_automatic_action",
                                                                             combined_request);

  auto delete_request = std::make_shared<AutomaticActionCombinedDelete::Request>();
  delete_request->name_of_combined_topics_publisher = "combined_action_and_publisher_name";

  auto response = callService<AutomaticActionCombinedDelete, AutomaticActionCombinedDelete::Request>(
      "delete_combined_automatic_action",
      delete_request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_TRUE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleCombinedDeleteServiceWithEmptyNameTest) {

  auto request = std::make_shared<AutomaticActionCombinedDelete::Request>();
  request->name_of_combined_topics_publisher = "";

  auto response = callService<AutomaticActionCombinedDelete, AutomaticActionCombinedDelete::Request>(
      "delete_combined_automatic_action",
      request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_FALSE(response->success);
}

TEST_F(AutomaticActionServiceTest, HandleAvailableTopicsServiceTest) {
  auto create_request = std::make_shared<AutomaticAction::Request>();
  create_request->listen_topic = "test_topic";
  create_request->listen_message_type = "std_msgs/String";
  create_request->action_and_publisher_name = "action_and_publisher_name";
  create_request->value = "field_value";
  create_request->trigger_val = "10";
  create_request->trigger_type = "GreaterThan";
  create_request->data_validity_ms = 1000;

  auto create_response =
      callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", create_request);

  ASSERT_TRUE(create_response != nullptr);
  EXPECT_TRUE(create_response->success);

  auto available_topics_request = std::make_shared<AvailableTopics::Request>();
  auto available_topics_response =
      callService<AvailableTopics, AvailableTopics::Request>("available_topics", available_topics_request);

  ASSERT_TRUE(available_topics_response != nullptr);

  bool topic_found = false;
  for (const auto& topic_info : available_topics_response->available_topics_with_parameters_and_time) {
    if (topic_info.action_and_publisher_name == "action_and_publisher_name") {
      topic_found = true;
      break;
    }
  }

  EXPECT_TRUE(topic_found) << "Topic 'action_and_publisher_name' not found in available topics";
}

TEST_F(AutomaticActionServiceTest, TriggerNotificationOnTopicMessageTest) {
  auto request = std::make_shared<AutomaticAction::Request>();
  request->listen_topic = "test_trigger_topic";
  request->listen_message_type = "std_msgs/msg/String";
  request->action_and_publisher_name = "notification_topic";
  request->value = "test_value";
  request->trigger_val = "trigger_value";
  request->trigger_type = "EqualTo";
  request->data_validity_ms = 1000;
  request->trigger_text = "Trigger activated!";
  request->publication_method = 1;

  auto response = callService<AutomaticAction, AutomaticAction::Request>("create_automatic_action", request);

  ASSERT_TRUE(response != nullptr);
  EXPECT_TRUE(response->success);

  auto notification_subscriber = node->create_subscription<std_msgs::msg::String>(
      "notification_topic",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        EXPECT_EQ(msg->data, "Trigger activated!");
        RCLCPP_INFO(node->get_logger(), "Received notification message: %s", msg->data.c_str());
      });

  auto publisher = node->create_publisher<std_msgs::msg::String>("test_trigger_topic", 10);

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  auto message = std::make_shared<std_msgs::msg::String>();
  message->data = "trigger_value";
  publisher->publish(*message);

  rclcpp::sleep_for(std::chrono::milliseconds(500));
}