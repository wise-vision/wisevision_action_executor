/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "yaml_config_manager.hpp"

void YamlConfigManager::saveConfigurationToFile(const std::string& filename,
                                                const std::shared_ptr<AutomaticActionRequestAndTime>& config) {

  if (config->listen_topic.empty() || config->action_and_publisher_name.empty() ||
      config->listen_message_type.empty() || config->pub_message_type.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"),
                 "Mandatory fields (listen_topic, action_and_publisher_name, etc.) are empty.");
    return;
  }

  YAML::Node root;

  std::ifstream file_in(filename);
  if (file_in.is_open()) {
    try {
      root = YAML::Load(file_in);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Failed to load YAML file: %s", e.what());
    }
    file_in.close();
  }

  YAML::Node new_config;
  new_config["listen_topic"] = config->listen_topic;
  new_config["listen_message_type"] = config->listen_message_type;
  new_config["value"] = config->value;
  new_config["trigger_val"] = config->trigger_val;
  new_config["trigger_type"] = config->trigger_type;
  new_config["action_and_publisher_name"] = config->action_and_publisher_name;
  new_config["pub_message_type"] = config->pub_message_type;
  new_config["trigger_text"] = config->trigger_text;
  new_config["data_validity_ms"] = config->data_validity_ms;
  new_config["publication_method"] = static_cast<int>(config->publication_method);

  YAML::Node date_and_time;
  date_and_time["year"] = config->date_and_time.year;
  date_and_time["month"] = config->date_and_time.month;
  date_and_time["day"] = config->date_and_time.day;
  date_and_time["hour"] = config->date_and_time.hour;
  date_and_time["minute"] = config->date_and_time.minute;
  date_and_time["second"] = config->date_and_time.second;
  date_and_time["nanosecond"] = config->date_and_time.nanosecond;

  new_config["date_and_time"] = date_and_time;

  root["subscriptions"].push_back(new_config);

  std::ofstream file_out(filename);
  if (file_out.is_open()) {
    file_out << root;
    file_out.close();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file for writing: %s", filename.c_str());
  }
}

void YamlConfigManager::loadConfigurationFromFile(
    const std::string& filename,
    std::unordered_map<std::string, std::shared_ptr<AutomaticActionRequestAndTime>>& configurations,
    std::unordered_map<std::string, std::shared_ptr<OneTopicData>>& topics_data) {

  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                "File not found: %s, creating an empty configuration file.",
                filename.c_str());

    // Create an empty YAML file
    YAML::Node config;
    std::ofstream outFile(filename);
    outFile << config;
    outFile.close();

    RCLCPP_INFO(rclcpp::get_logger("YamlConfigManager"), "Empty configuration file created: %s", filename.c_str());
    return;
  }

  YAML::Node config = YAML::Load(file);
  file.close();

  for (const auto& node : config["subscriptions"]) {
    if (!node["listen_topic"] || !node["listen_message_type"] || !node["action_and_publisher_name"] ||
        !node["pub_message_type"] || !node["value"] || !node["trigger_val"] || !node["trigger_type"] ||
        !node["trigger_text"] || !node["data_validity_ms"] || !node["publication_method"] || !node["date_and_time"]) {
      RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"), "Skipping subscription due to missing mandatory fields.");
      continue;
    }

    auto request = std::make_shared<AutomaticActionRequestAndTime>();

    request->listen_topic = node["listen_topic"].as<std::string>();
    request->listen_message_type = node["listen_message_type"].as<std::string>();
    request->value = node["value"].as<std::string>();
    request->trigger_val = node["trigger_val"].as<std::string>();
    request->trigger_type = node["trigger_type"].as<std::string>();
    request->action_and_publisher_name = node["action_and_publisher_name"].as<std::string>();
    request->pub_message_type = node["pub_message_type"].as<std::string>();
    request->trigger_text = node["trigger_text"].as<std::string>();
    request->data_validity_ms = node["data_validity_ms"].as<int64_t>();
    request->publication_method = node["publication_method"].as<uint8_t>();
    request->date_and_time.year = node["date_and_time"]["year"].as<int32_t>();
    request->date_and_time.month = node["date_and_time"]["month"].as<int32_t>();
    request->date_and_time.day = node["date_and_time"]["day"].as<int32_t>();
    request->date_and_time.hour = node["date_and_time"]["hour"].as<int32_t>();
    request->date_and_time.minute = node["date_and_time"]["minute"].as<int32_t>();
    request->date_and_time.second = node["date_and_time"]["second"].as<int32_t>();
    request->date_and_time.nanosecond = node["date_and_time"]["nanosecond"].as<uint32_t>();

    configurations[request->action_and_publisher_name] = request;

    auto topic_data = std::make_shared<OneTopicData>();
    topic_data->action_triggered = false;
    topic_data->time_stamp = rclcpp::Clock().now();
    topic_data->data_validity_ms = request->data_validity_ms;

    topics_data[request->action_and_publisher_name] = topic_data;
  }
}

void YamlConfigManager::modifyConfigurationInFile(const std::string& filename,
                                                  const std::shared_ptr<AutomaticActionRequestAndTime>& config) {

  YAML::Node root;
  std::ifstream file_in(filename);
  if (file_in.is_open()) {
    try {
      root = YAML::Load(file_in);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Failed to load YAML file: %s", e.what());
      return;
    }
    file_in.close();
  }

  bool updated = false;

  for (std::size_t i = 0; i < root["subscriptions"].size(); ++i) {
    YAML::Node node = root["subscriptions"][i];
    if (node["action_and_publisher_name"] &&
        node["action_and_publisher_name"].as<std::string>() == config->action_and_publisher_name) {
      if (!config->listen_message_type.empty() &&
          config->listen_message_type != node["listen_message_type"].as<std::string>()) {
        node["listen_message_type"] = config->listen_message_type;
        updated = true;
      }
      if (!config->value.empty() && config->value != node["value"].as<std::string>()) {
        node["value"] = config->value;
        updated = true;
      }
      if (!config->trigger_val.empty() && config->trigger_val != node["trigger_val"].as<std::string>()) {
        node["trigger_val"] = config->trigger_val;
        updated = true;
      }
      if (!config->trigger_type.empty() && config->trigger_type != node["trigger_type"].as<std::string>()) {
        node["trigger_type"] = config->trigger_type;
        updated = true;
      }
      if (!config->action_and_publisher_name.empty() &&
          config->action_and_publisher_name != node["action_and_publisher_name"].as<std::string>()) {
        node["action_and_publisher_name"] = config->action_and_publisher_name;
        updated = true;
      }
      if (!config->pub_message_type.empty() && config->pub_message_type != node["pub_message_type"].as<std::string>()) {
        node["pub_message_type"] = config->pub_message_type;
        updated = true;
      }
      if (!config->trigger_text.empty() && config->trigger_text != node["trigger_text"].as<std::string>()) {
        node["trigger_text"] = config->trigger_text;
        updated = true;
      }
      if (config->data_validity_ms != node["data_validity_ms"].as<int64_t>()) {
        node["data_validity_ms"] = config->data_validity_ms;
        updated = true;
      }
      if (config->publication_method != node["publication_method"].as<uint8_t>()) {
        node["publication_method"] = static_cast<int>(config->publication_method);
        updated = true;
      }

      if (config->date_and_time.year != node["date_and_time"]["year"].as<int32_t>() ||
          config->date_and_time.month != node["date_and_time"]["month"].as<int32_t>() ||
          config->date_and_time.day != node["date_and_time"]["day"].as<int32_t>() ||
          config->date_and_time.hour != node["date_and_time"]["hour"].as<int32_t>() ||
          config->date_and_time.minute != node["date_and_time"]["minute"].as<int32_t>() ||
          config->date_and_time.second != node["date_and_time"]["second"].as<int32_t>() ||
          config->date_and_time.nanosecond != node["date_and_time"]["nanosecond"].as<uint32_t>()) {

        node["date_and_time"]["year"] = config->date_and_time.year;
        node["date_and_time"]["month"] = config->date_and_time.month;
        node["date_and_time"]["day"] = config->date_and_time.day;
        node["date_and_time"]["hour"] = config->date_and_time.hour;
        node["date_and_time"]["minute"] = config->date_and_time.minute;
        node["date_and_time"]["second"] = config->date_and_time.second;
        node["date_and_time"]["nanosecond"] = config->date_and_time.nanosecond;
        updated = true;
      }
      break;
    }
  }

  if (!updated) {
    RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                "Configuration for listen_topic '%s' not found, no changes made.",
                config->listen_topic.c_str());
    return;
  }

  std::ofstream file_out(filename);
  if (file_out.is_open()) {
    file_out << root;
    file_out.close();
    RCLCPP_INFO(rclcpp::get_logger("YamlConfigManager"), "Configuration updated in file: %s", filename.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file for writing: %s", filename.c_str());
  }
}

void YamlConfigManager::saveCombinedConfigurationToFile(
    const std::string& filename,
    const std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>>&
        combined_topics_publishers_and_info) {
  YAML::Node root;

  for (const auto& pair : combined_topics_publishers_and_info) {
    const auto& combined_publisher = pair.second;

    if (combined_publisher->config.action_and_publisher_name.empty() ||
        combined_publisher->config.logic_expression.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "action_and_publisher_name or logic_expression is empty.");
      continue;
    }

    YAML::Node new_config;
    new_config["action_and_publisher_name"] = combined_publisher->config.action_and_publisher_name;
    new_config["logic_expression"] = combined_publisher->config.logic_expression;
    new_config["trigger_text"] = combined_publisher->config.trigger_text;
    new_config["publication_method"] = static_cast<int>(combined_publisher->config.publication_method);

    YAML::Node listen_topics_node;
    for (const auto& topic : combined_publisher->config.listen_topics) {
      if (!topic.empty()) {
        listen_topics_node.push_back(topic);
      }
    }
    new_config["listen_topics"] = listen_topics_node;

    YAML::Node date_and_time;
    date_and_time["year"] = combined_publisher->config.date_and_time.year;
    date_and_time["month"] = combined_publisher->config.date_and_time.month;
    date_and_time["day"] = combined_publisher->config.date_and_time.day;
    date_and_time["hour"] = combined_publisher->config.date_and_time.hour;
    date_and_time["minute"] = combined_publisher->config.date_and_time.minute;
    date_and_time["second"] = combined_publisher->config.date_and_time.second;
    date_and_time["nanosecond"] = combined_publisher->config.date_and_time.nanosecond;

    new_config["date_and_time"] = date_and_time;

    root["combined_subscriptions"].push_back(new_config);
  }

  std::ofstream file_out(filename);
  if (file_out.is_open()) {
    file_out << root;
    file_out.close();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file for writing: %s", filename.c_str());
  }
}

void YamlConfigManager::loadCombinedConfigurationFromFile(
    const std::string& filename,
    std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>>& combined_topics_publishers_and_info) {

  std::ifstream file(filename);
  if (!file.is_open()) {
    RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                "File not found: %s, creating an empty configuration file.",
                filename.c_str());

    // Create an empty YAML file
    YAML::Node config;
    std::ofstream outFile(filename);
    outFile << config;
    outFile.close();

    RCLCPP_INFO(rclcpp::get_logger("YamlConfigManager"), "Empty configuration file created: %s", filename.c_str());
    return;
  }

  YAML::Node config = YAML::Load(file);
  file.close();

  for (const auto& node : config["combined_subscriptions"]) {
    if (!node["action_and_publisher_name"] || node["action_and_publisher_name"].as<std::string>().empty() ||
        !node["logic_expression"] || node["logic_expression"].as<std::string>().empty() ||
        !node["publication_method"] || !node["listen_topics"] || !node["date_and_time"]) {
      RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                  "Skipping combined subscription due to missing or empty fields.");
      continue;
    }

    auto request = std::make_shared<AutomaticActionCombinedRequestAndTime>();
    request->action_and_publisher_name = node["action_and_publisher_name"].as<std::string>();
    request->logic_expression = node["logic_expression"].as<std::string>();
    request->trigger_text = node["trigger_text"].as<std::string>();
    request->publication_method = node["publication_method"].as<uint8_t>();

    for (const auto& topic : node["listen_topics"]) {
      if (!topic.IsDefined() || topic.as<std::string>().empty()) {
        RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"), "Invalid or empty topic found in listen_topics.");
        continue;
      }
      request->listen_topics.push_back(topic.as<std::string>());
    }

    if (request->listen_topics.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                  "Skipping combined subscription due to empty listen_topics.");
      continue;
    }

    request->date_and_time.year = node["date_and_time"]["year"].as<int32_t>();
    request->date_and_time.month = node["date_and_time"]["month"].as<int32_t>();
    request->date_and_time.day = node["date_and_time"]["day"].as<int32_t>();
    request->date_and_time.hour = node["date_and_time"]["hour"].as<int32_t>();
    request->date_and_time.minute = node["date_and_time"]["minute"].as<int32_t>();
    request->date_and_time.second = node["date_and_time"]["second"].as<int32_t>();
    request->date_and_time.nanosecond = node["date_and_time"]["nanosecond"].as<uint32_t>();

    auto combined_publisher = std::make_shared<CombinedTopicsPublisher>();
    combined_publisher->config = *request;

    combined_topics_publishers_and_info[request->action_and_publisher_name] = combined_publisher;
  }
}

void YamlConfigManager::modifyCombinedConfigurationInFile(
    const std::string& filename,
    const std::shared_ptr<AutomaticActionCombinedRequestAndTime>& config) {

  YAML::Node root;
  std::ifstream file_in(filename);
  if (file_in.is_open()) {
    try {
      root = YAML::Load(file_in);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Failed to load YAML file: %s", e.what());
      return;
    }
    file_in.close();
  }

  bool updated = false;

  for (YAML::iterator it = root["combined_subscriptions"].begin(); it != root["combined_subscriptions"].end(); ++it) {
    YAML::Node node = *it;

    if (node["action_and_publisher_name"] &&
        node["action_and_publisher_name"].as<std::string>() == config->action_and_publisher_name) {

      if (!config->logic_expression.empty() && config->logic_expression != node["logic_expression"].as<std::string>()) {
        node["logic_expression"] = config->logic_expression;
        updated = true;
      }

      if (!config->trigger_text.empty() && config->trigger_text != node["trigger_text"].as<std::string>()) {
        node["trigger_text"] = config->trigger_text;
        updated = true;
      }

      if (config->publication_method != node["publication_method"].as<uint8_t>()) {
        node["publication_method"] = static_cast<int>(config->publication_method);
        updated = true;
      }

      YAML::Node listen_topics_node = YAML::Node(YAML::NodeType::Sequence);
      for (const auto& topic : config->listen_topics) {
        listen_topics_node.push_back(topic);
      }

      bool listen_topics_different = false;
      if (node["listen_topics"].size() == listen_topics_node.size()) {
        for (size_t i = 0; i < node["listen_topics"].size(); ++i) {
          if (node["listen_topics"][i].as<std::string>() != listen_topics_node[i].as<std::string>()) {
            listen_topics_different = true;
            break;
          }
        }
      } else {
        listen_topics_different = true;
      }

      if (listen_topics_different) {
        node["listen_topics"] = listen_topics_node;
        updated = true;
      }

      if (config->date_and_time.year != node["date_and_time"]["year"].as<int32_t>() ||
          config->date_and_time.month != node["date_and_time"]["month"].as<int32_t>() ||
          config->date_and_time.day != node["date_and_time"]["day"].as<int32_t>() ||
          config->date_and_time.hour != node["date_and_time"]["hour"].as<int32_t>() ||
          config->date_and_time.minute != node["date_and_time"]["minute"].as<int32_t>() ||
          config->date_and_time.second != node["date_and_time"]["second"].as<int32_t>() ||
          config->date_and_time.nanosecond != node["date_and_time"]["nanosecond"].as<uint32_t>()) {

        node["date_and_time"]["year"] = config->date_and_time.year;
        node["date_and_time"]["month"] = config->date_and_time.month;
        node["date_and_time"]["day"] = config->date_and_time.day;
        node["date_and_time"]["hour"] = config->date_and_time.hour;
        node["date_and_time"]["minute"] = config->date_and_time.minute;
        node["date_and_time"]["second"] = config->date_and_time.second;
        node["date_and_time"]["nanosecond"] = config->date_and_time.nanosecond;
        updated = true;
      }

      break;
    }
  }

  if (!updated) {
    RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                "Configuration for action_and_publisher_name '%s' not found, no changes made.",
                config->action_and_publisher_name.c_str());
    return;
  }

  std::ofstream file_out(filename);
  if (file_out.is_open()) {
    file_out << root;
    file_out.close();
    RCLCPP_INFO(rclcpp::get_logger("YamlConfigManager"), "Configuration updated in file: %s", filename.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file for writing: %s", filename.c_str());
  }
}

void YamlConfigManager::removeSubscriptionFromFile(const std::string& filename, const std::string& topic_to_remove) {
  if (topic_to_remove.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Topic to remove is empty.");
    return;
  }

  YAML::Node root;
  std::ifstream file_in(filename);
  if (file_in.is_open()) {
    try {
      root = YAML::Load(file_in);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Failed to load YAML file: %s", e.what());
      file_in.close();
      return;
    }
    file_in.close();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file: %s", filename.c_str());
    return;
  }

  if (!root["subscriptions"]) {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "No subscriptions found in the file.");
    return;
  }

  YAML::Node subscriptions = root["subscriptions"];
  YAML::Node new_subscriptions;
  bool topic_found = false;

  for (const auto& node : subscriptions) {
    if (node["action_and_publisher_name"].as<std::string>() != topic_to_remove) {
      new_subscriptions.push_back(node);
    } else {
      topic_found = true;
    }
  }

  if (!topic_found) {
    RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                "Topic %s not found in subscriptions.",
                topic_to_remove.c_str());
    return;
  }

  root["subscriptions"] = new_subscriptions;

  std::ofstream file_out(filename);
  if (file_out.is_open()) {
    file_out << root;
    file_out.close();
    RCLCPP_INFO(rclcpp::get_logger("YamlConfigManager"), "Topic %s successfully removed.", topic_to_remove.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file for writing: %s", filename.c_str());
  }
}

void YamlConfigManager::removeCombinedPublisherFromFile(const std::string& filename,
                                                        const std::string& pub_topic_to_remove) {
  if (pub_topic_to_remove.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "action_and_publisher_name to remove is empty.");
    return;
  }

  YAML::Node root;
  std::ifstream file_in(filename);
  if (file_in.is_open()) {
    try {
      root = YAML::Load(file_in);
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Failed to load YAML file: %s", e.what());
      file_in.close();
      return;
    }
    file_in.close();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file: %s", filename.c_str());
    return;
  }

  if (!root["combined_subscriptions"]) {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "No combined subscriptions found in the file.");
    return;
  }

  YAML::Node combined_subscriptions = root["combined_subscriptions"];
  YAML::Node new_combined_subscriptions;
  bool topic_found = false;

  for (const auto& node : combined_subscriptions) {
    if (node["action_and_publisher_name"].as<std::string>() != pub_topic_to_remove) {
      new_combined_subscriptions.push_back(node);
    } else {
      topic_found = true;
    }
  }

  if (!topic_found) {
    RCLCPP_WARN(rclcpp::get_logger("YamlConfigManager"),
                "Combined subscription %s not found.",
                pub_topic_to_remove.c_str());
    return;
  }

  root["combined_subscriptions"] = new_combined_subscriptions;

  std::ofstream file_out(filename);
  if (file_out.is_open()) {
    file_out << root;
    file_out.close();
    RCLCPP_INFO(rclcpp::get_logger("YamlConfigManager"),
                "Combined subscription %s successfully removed.",
                pub_topic_to_remove.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("YamlConfigManager"), "Unable to open file for writing: %s", filename.c_str());
  }
}
