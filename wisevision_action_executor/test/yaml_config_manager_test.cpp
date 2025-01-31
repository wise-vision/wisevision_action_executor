/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <fstream>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "yaml_config_manager.hpp"

class YamlConfigManagerTest : public ::testing::Test {
protected:
  std::string test_filename = "test_config.yaml";

  void SetUp() override {
    std::ofstream file(test_filename);
    file << "subscriptions: []" << std::endl;
    file.close();
  }

  void TearDown() override {
    std::remove(test_filename.c_str());
  }
};

TEST_F(YamlConfigManagerTest, SaveConfigurationWithEmptyFieldsTest) {
  YamlConfigManager manager;
  auto config = std::make_shared<AutomaticActionRequestAndTime>();
  config->listen_topic = "";
  config->action_and_publisher_name = "output_topic";
  config->listen_message_type = "std_msgs/String";
  config->pub_message_type = "std_msgs/String";

  testing::internal::CaptureStderr();
  manager.saveConfigurationToFile(test_filename, config);
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(output.find("Mandatory fields (listen_topic, action_and_publisher_name, etc.) are empty.") != std::string::npos);

  YAML::Node config_yaml = YAML::LoadFile(test_filename);
  ASSERT_EQ(config_yaml["subscriptions"].size(), 0);
}

TEST_F(YamlConfigManagerTest, LoadConfigurationWithMissingFieldsTest) {
  YamlConfigManager manager;
  std::unordered_map<std::string, std::shared_ptr<AutomaticActionRequestAndTime>> configurations;
  std::unordered_map<std::string, std::shared_ptr<OneTopicData>> topics_data;

  std::ofstream file(test_filename);
  file << "subscriptions:\n"
       << "  - listen_topic: \"test_topic\"\n"
       << "    listen_message_type: \"std_msgs/String\"\n"
       << "    action_and_publisher_name: \"output_topic\"\n"
       << "    pub_message_type: \"std_msgs/String\"\n"
       << "    trigger_text: \"\"\n";
  file.close();

  testing::internal::CaptureStderr();
  manager.loadConfigurationFromFile(test_filename, configurations, topics_data);
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_EQ(configurations.size(), 0);
  ASSERT_EQ(topics_data.size(), 0);

  EXPECT_TRUE(output.find("Skipping subscription due to missing mandatory fields.") != std::string::npos);
}

TEST_F(YamlConfigManagerTest, SaveCombinedConfigurationWithEmptyFieldsTest) {
  YamlConfigManager manager;
  std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>> combined_topics;

  auto publisher = std::make_shared<CombinedTopicsPublisher>();
  publisher->config.action_and_publisher_name = "";
  publisher->config.logic_expression = "some_expression";

  combined_topics["publisher1"] = publisher;

  testing::internal::CaptureStderr();
  manager.saveCombinedConfigurationToFile(test_filename, combined_topics);
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(output.find("action_and_publisher_name or logic_expression is empty.") != std::string::npos);

  YAML::Node config_yaml = YAML::LoadFile(test_filename);
  ASSERT_EQ(config_yaml["combined_subscriptions"].size(), 0);
}

TEST_F(YamlConfigManagerTest, LoadCombinedConfigurationWithMissingFieldsTest) {
  YamlConfigManager manager;
  std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>> combined_topics;

  std::ofstream file(test_filename);
  file << "combined_subscriptions:\n"
       << "  - action_and_publisher_name: \"test_action_and_publisher_name\"\n"
       << "    logic_expression: \"\"\n"
       << "    publication_method: 1\n"
       << "    listen_topics: [\"topic1\", \"topic2\"]\n";
  file.close();

  testing::internal::CaptureStderr();
  manager.loadCombinedConfigurationFromFile(test_filename, combined_topics);
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_EQ(combined_topics.size(), 0);

  EXPECT_TRUE(output.find("Skipping combined subscription due to missing or empty fields.") != std::string::npos);
}

TEST_F(YamlConfigManagerTest, RemoveSubscriptionWithEmptyTopicTest) {
  YamlConfigManager manager;

  std::ofstream file(test_filename);
  file << "subscriptions:\n"
       << "  - listen_topic: \"test_topic_1\"\n"
       << "    action_and_publisher_name: \"output_topic_1\"\n";
  file.close();

  testing::internal::CaptureStderr();
  manager.removeSubscriptionFromFile(test_filename, "");
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(output.find("Topic to remove is empty.") != std::string::npos);

  YAML::Node config_yaml = YAML::LoadFile(test_filename);
  ASSERT_EQ(config_yaml["subscriptions"].size(), 1);
}

TEST_F(YamlConfigManagerTest, RemoveCombinedPublisherWithEmptyTopicTest) {
  YamlConfigManager manager;

  std::ofstream file(test_filename);
  file << "combined_subscriptions:\n"
       << "  - action_and_publisher_name: \"test_action_and_publisher_name\"\n"
       << "    logic_expression: \"some_expression\"\n";
  file.close();

  testing::internal::CaptureStderr();
  manager.removeCombinedPublisherFromFile(test_filename, "");
  std::string output = testing::internal::GetCapturedStderr();

  EXPECT_TRUE(output.find("action_and_publisher_name to remove is empty.") != std::string::npos);

  YAML::Node config_yaml = YAML::LoadFile(test_filename);
  ASSERT_EQ(config_yaml["combined_subscriptions"].size(), 1);
}