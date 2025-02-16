/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "automatic_action_service.hpp"

AutomaticActionService::AutomaticActionService() : Node("automatic_action_service") {
  m_add_automatic_action_service = this->create_service<AutomaticAction>(
      "create_automatic_action",
      std::bind(&AutomaticActionService::handleCreateService, this, std::placeholders::_1, std::placeholders::_2));

  m_delete_service = this->create_service<AutomaticActionDelete>(
      "delete_automatic_action",
      std::bind(&AutomaticActionService::handleDeleteService, this, std::placeholders::_1, std::placeholders::_2));

  m_connection_service = this->create_service<AutomaticActionConnection>(
      "create_combined_automatic_action",
      std::bind(&AutomaticActionService::handleCombinedService, this, std::placeholders::_1, std::placeholders::_2));

  m_available_topics_service = this->create_service<AvailableTopics>(
      "available_topics",
      std::bind(&AutomaticActionService::handleAvailableTopicsService, this, std::placeholders::_2));

  m_available_topics_combined_service = this->create_service<AvailableTopicsCombined>(
      "available_topics_combined",
      std::bind(&AutomaticActionService::handleAvailableTopicsCombinedService, this, std::placeholders::_2));

  m_change_automatic_action_service =
      this->create_service<ChangeAutomaticAction>("change_automatic_action",
                                                  std::bind(&AutomaticActionService::handleChangeAutomaticActionService,
                                                            this,
                                                            std::placeholders::_1,
                                                            std::placeholders::_2));

  m_change_automatic_action_combined_service = this->create_service<ChangeAutomaticActionCombined>(
      "change_combined_automatic_action",
      std::bind(&AutomaticActionService::handleChangeAutomaticActionCombinedService,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  m_combined_delete_service = this->create_service<AutomaticActionCombinedDelete>(
      "delete_combined_automatic_action",
      std::bind(&AutomaticActionService::handleCombinedDeleteService,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  m_notification_publisher = this->create_publisher<Notification>("notifications", 10);

  m_yaml_config_manager->loadConfigurationFromFile("config.yaml", m_configurations, m_topics_data);
  initializeSubscriptions();
  m_yaml_config_manager->loadCombinedConfigurationFromFile("config_combined.yaml",
                                                           m_combined_topics_publishers_and_info);
  initializeCombinedPublishers();
}

void AutomaticActionService::initializeSubscriptions() {
  for (const auto& [topic, request] : m_configurations) {
    auto captured_request = request; // Create a copy of request for safe capture
    auto m_subscription =
        this->create_generic_subscription(request->listen_topic,
                                          request->listen_message_type,
                                          10,
                                          [this, captured_request](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                                            this->messageCallback(msg, captured_request);
                                          });
    m_subscriptions[request->action_and_publisher_name] = m_subscription;
  }
}

void AutomaticActionService::initializeCombinedPublishers() {
  for (const auto& [action_and_publisher_name, combined_publisher] : m_combined_topics_publishers_and_info) {
    if (m_publishers.find(action_and_publisher_name) == m_publishers.end()) {
      auto publisher = this->create_publisher<std_msgs::msg::String>(action_and_publisher_name, 10);
      combined_publisher->publisher = publisher;
      m_publishers[action_and_publisher_name] = publisher;
    } else {
      combined_publisher->publisher = m_publishers[action_and_publisher_name];
    }
  }
}

void AutomaticActionService::handleCreateService(const std::shared_ptr<AutomaticAction::Request> request,
                                                 std::shared_ptr<AutomaticAction::Response> response) {

  if (request->listen_topic.empty() || request->listen_message_type.empty() ||
      request->action_and_publisher_name.empty() || request->value.empty() || request->trigger_val.empty() ||
      request->trigger_type.empty() || request->publication_method > 6) {
    RCLCPP_WARN(this->get_logger(), "Failed to create subscription: some fields in the request are empty or invalid.");
    response->success = false;
    response->error_message = "Some fields in the request are empty or invalid.";
    return;
  }

  if (m_subscriptions.find(request->listen_topic) != m_subscriptions.end()) {
    RCLCPP_WARN(this->get_logger(), "Already monitoring topic: %s", request->listen_topic.c_str());
    response->success = false;
    response->error_message = "Already monitoring topic: " + request->listen_topic;
    return;
  }

  auto action_and_publisher_name = request->action_and_publisher_name;
  std::shared_ptr<AutomaticActionRequestAndTime> request_and_time = std::make_shared<AutomaticActionRequestAndTime>();
  request_and_time->listen_topic = request->listen_topic;
  request_and_time->listen_message_type = request->listen_message_type;
  request_and_time->value = request->value;
  request_and_time->trigger_val = request->trigger_val;
  request_and_time->trigger_type = request->trigger_type;
  request_and_time->action_and_publisher_name = action_and_publisher_name;
  request_and_time->pub_message_type = request->pub_message_type;
  request_and_time->trigger_text = request->trigger_text;
  request_and_time->data_validity_ms = request->data_validity_ms;
  request_and_time->publication_method = request->publication_method;

  RCLCPP_INFO(this->get_logger(), "Received request to monitor topic: %s", request->listen_topic.c_str());

  auto subscription =
      this->create_generic_subscription(request->listen_topic,
                                        request->listen_message_type,
                                        10,
                                        [this, request_and_time](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                                          this->messageCallback(msg, request_and_time);
                                        });

  m_subscriptions[request->action_and_publisher_name] = subscription;
  m_configurations[request->action_and_publisher_name] = request_and_time;
  auto topic_data = std::make_shared<OneTopicData>();
  topic_data->action_triggered = false;
  topic_data->time_stamp = this->now();
  topic_data->data_validity_ms = request->data_validity_ms;
  m_topics_data[request->action_and_publisher_name] = topic_data;
  response->success = true;

  m_yaml_config_manager->saveConfigurationToFile("config.yaml", request_and_time);
}

void AutomaticActionService::handleDeleteService(const std::shared_ptr<AutomaticActionDelete::Request> request,
                                                 std::shared_ptr<AutomaticActionDelete::Response> response) {
  if (request->listen_topic_to_delete.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to delete subscription: listen_topic_to_delete is empty.");
    response->success = false;
    response->error_message = "listen_topic_to_delete is empty.";
    return;
  }

  for (const auto& [key, combined_topics_publisher] : m_combined_topics_publishers_and_info) {
    const auto& topics = combined_topics_publisher->config.listen_topics;
    if (std::find(topics.begin(), topics.end(), request->listen_topic_to_delete) != topics.end()) {
      response->success = false;
      response->error_message = "Cannot delete topic that is part of a combined publisher: " +
                                combined_topics_publisher->config.action_and_publisher_name;
      return;
    }
  }

  auto topic_to_delete = m_subscriptions.find(request->listen_topic_to_delete);
  if (topic_to_delete != m_subscriptions.end()) {
    m_subscriptions.erase(topic_to_delete);
    m_topics_data.erase(request->listen_topic_to_delete);
    m_configurations.erase(request->listen_topic_to_delete);
    RCLCPP_INFO(this->get_logger(), "Stopped monitoring topic: %s", request->listen_topic_to_delete.c_str());
    response->success = true;
    m_yaml_config_manager->removeSubscriptionFromFile("config.yaml", request->listen_topic_to_delete);
  } else {
    RCLCPP_WARN(this->get_logger(), "Topic not found: %s", request->listen_topic_to_delete.c_str());
    response->success = false;
    response->error_message = "Topic not found: " + request->listen_topic_to_delete;
  }
}

void AutomaticActionService::replaceExactString(std::string& source, const std::string& from, const std::string& to) {
  size_t start_pos = 0;
  while ((start_pos = source.find(from, start_pos)) != std::string::npos) {
    if ((start_pos == 0 || !isalnum(source[start_pos - 1])) &&
        (start_pos + from.length() == source.length() || !isalnum(source[start_pos + from.length()]))) {
      source.replace(start_pos, from.length(), to);
      start_pos += to.length();
    } else {
      start_pos += from.length();
    }
  }
}

void AutomaticActionService::handleChangeAutomaticActionService(
    const std::shared_ptr<ChangeAutomaticAction::Request> request,
    std::shared_ptr<ChangeAutomaticAction::Response> response) {

  if (request->action_and_publisher_name_to_change.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to change action: action_and_publisher_name_to_change is empty.");
    response->success = false;
    response->error_message = "action_and_publisher_name_to_change is empty.";
    return;
  }

  auto it = m_configurations.find(request->action_and_publisher_name_to_change);
  if (it == m_configurations.end()) {
    RCLCPP_WARN(this->get_logger(), "Action with specified listen_topic not found: %s", request->listen_topic.c_str());
    response->success = false;
    response->error_message = "Action with specified listen_topic not found: " + request->listen_topic;
    return;
  }

  bool requires_recreation = request->new_action_and_publisher_name != it->second->action_and_publisher_name ||
                             request->listen_topic != it->second->listen_topic ||
                             request->listen_message_type != it->second->listen_message_type ||
                             request->pub_message_type != it->second->pub_message_type;

  if (requires_recreation) {
    auto delete_request = std::make_shared<AutomaticActionDelete::Request>();
    delete_request->listen_topic_to_delete = it->second->action_and_publisher_name;
    auto delete_response = std::make_shared<AutomaticActionDelete::Response>();
    handleDeleteService(delete_request, delete_response);

    if (!delete_response->success) {
      RCLCPP_WARN(this->get_logger(), "Failed to delete action before recreation.");
      response->success = false;
      response->error_message = "Failed to delete action before recreation.";
      return;
    }

    auto create_request = std::make_shared<AutomaticAction::Request>();
    create_request->listen_topic = request->listen_topic;
    create_request->listen_message_type = request->listen_message_type;
    create_request->value = request->value;
    create_request->trigger_val = request->trigger_val;
    create_request->trigger_type = request->trigger_type;
    create_request->action_and_publisher_name = request->new_action_and_publisher_name;
    create_request->pub_message_type = request->pub_message_type;
    create_request->trigger_text = request->trigger_text;
    create_request->data_validity_ms = request->data_validity_ms;
    create_request->publication_method = request->publication_method;
    auto create_response = std::make_shared<AutomaticAction::Response>();
    handleCreateService(create_request, create_response);

    if (!create_response->success) {
      RCLCPP_WARN(this->get_logger(), "Failed to create action after deletion.");
      response->success = false;
      response->error_message = "Failed to create action after deletion.";
      return;
    }

    std::string old_action_name = request->action_and_publisher_name_to_change;
    std::string new_action_name = request->new_action_and_publisher_name;

    for (auto& [key, combined_publisher] : m_combined_topics_publishers_and_info) {
      bool updated = false;

      for (auto& topic : combined_publisher->config.listen_topics) {
        if (topic == old_action_name) {
          topic = new_action_name;
          updated = true;
          RCLCPP_INFO(this->get_logger(),
                      "Updated combined topic %s to use new action name %s in listen_topics.",
                      key.c_str(),
                      new_action_name.c_str());
        }
      }

      std::string new_logic_expression = combined_publisher->config.logic_expression;
      replaceExactString(new_logic_expression, old_action_name, new_action_name);
      std::cout << "New logic expression: " << new_logic_expression << std::endl;

      if (new_logic_expression != combined_publisher->config.logic_expression) {
        combined_publisher->config.logic_expression = new_logic_expression;
        updated = true;
        RCLCPP_INFO(this->get_logger(),
                    "Updated logic expression for combined topic %s to: %s",
                    key.c_str(),
                    combined_publisher->config.logic_expression.c_str());
      }

      if (updated) {
        AutomaticActionCombinedRequestAndTime combined_config;
        combined_config.action_and_publisher_name = combined_publisher->config.action_and_publisher_name;
        std::cout << "Combined config name: " << combined_config.action_and_publisher_name << std::endl;
        combined_config.listen_topics = combined_publisher->config.listen_topics;
        combined_config.logic_expression = combined_publisher->config.logic_expression;
        std::cout << "Combined config logic expression: " << combined_config.logic_expression << std::endl;

        auto config_ptr = std::make_shared<AutomaticActionCombinedRequestAndTime>(combined_config);
        m_yaml_config_manager->modifyCombinedConfigurationInFile("config_combined.yaml", config_ptr);
      }
    }

    response->success = true;
    return;
  }

  it->second->value = request->value;
  it->second->trigger_val = request->trigger_val;
  it->second->trigger_type = request->trigger_type;
  it->second->trigger_text = request->trigger_text;
  it->second->data_validity_ms = request->data_validity_ms;
  it->second->publication_method = request->publication_method;

  if (m_topics_data.find(request->listen_topic) != m_topics_data.end()) {
    m_topics_data[request->listen_topic]->data_validity_ms = request->data_validity_ms;
  }

  m_yaml_config_manager->modifyConfigurationInFile("config.yaml", it->second);

  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Action updated successfully.");
}

void AutomaticActionService::handleAvailableTopicsService(std::shared_ptr<AvailableTopics::Response> response) {
  for (const auto& pair : m_configurations) {
    response->available_topics_with_parameters_and_time.push_back(*pair.second);
  }
}
void AutomaticActionService::handleCombinedService(const std::shared_ptr<AutomaticActionConnection::Request> request,
                                                   std::shared_ptr<AutomaticActionConnection::Response> response) {
  if (request->listen_topics.size() < 2 || request->logic_expression.empty() ||
      request->action_and_publisher_name.empty() || request->trigger_text.empty() || request->publication_method > 6) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to create combined publisher: some fields in the request are empty or invalid.");
    response->success = false;
    response->error_message = "Some fields in the request are empty or invalid.";
    return;
  }

  bool all_found = true;
  for (const auto& topic : request->listen_topics) {
    if (m_configurations.find(topic) == m_configurations.end()) {
      all_found = false;
      break;
    }
  }
  if (!all_found) {
    RCLCPP_WARN(this->get_logger(), "Not every topic found");
    response->success = false;
    response->error_message = "Not every topic found";
    return;
  }

  if (m_combined_topics_publishers_and_info.find(request->action_and_publisher_name) !=
      m_combined_topics_publishers_and_info.end()) {
    RCLCPP_INFO(this->get_logger(),
                "Publisher already exists for topic: %s",
                request->action_and_publisher_name.c_str());
  } else {
    auto publisher = this->create_publisher<std_msgs::msg::String>(request->action_and_publisher_name, 10);
    auto combined_publisher = std::make_shared<CombinedTopicsPublisher>();
    combined_publisher->config.listen_topics = request->listen_topics;
    combined_publisher->config.logic_expression = request->logic_expression;
    combined_publisher->config.action_and_publisher_name = request->action_and_publisher_name;
    combined_publisher->config.trigger_text = request->trigger_text;
    combined_publisher->config.publication_method = request->publication_method;
    combined_publisher->publisher = publisher;
    m_combined_topics_publishers_and_info[request->action_and_publisher_name] = combined_publisher;
  }

  auto combined_publisher = std::make_shared<CombinedTopicsPublisher>();
  combined_publisher->config.listen_topics = request->listen_topics;
  combined_publisher->config.logic_expression = request->logic_expression;
  combined_publisher->config.action_and_publisher_name = request->action_and_publisher_name;
  combined_publisher->config.trigger_text = request->trigger_text;
  combined_publisher->config.publication_method = request->publication_method;
  combined_publisher->publisher = m_combined_topics_publishers_and_info[request->action_and_publisher_name]->publisher;

  m_combined_topics_publishers_and_info[request->action_and_publisher_name] = combined_publisher;

  m_yaml_config_manager->saveCombinedConfigurationToFile("config_combined.yaml", m_combined_topics_publishers_and_info);

  response->success = true;
}

void AutomaticActionService::handleCombinedDeleteService(
    const std::shared_ptr<AutomaticActionCombinedDelete::Request> request,
    std::shared_ptr<AutomaticActionCombinedDelete::Response> response) {
  if (request->name_of_combined_topics_publisher.empty()) {
    RCLCPP_WARN(this->get_logger(), "Failed to delete combined publisher: name_of_combined_topics_publisher is empty.");
    response->success = false;
    response->error_message = "name_of_combined_topics_publisher is empty.";
    return;
  }

  auto it = m_combined_topics_publishers_and_info.find(request->name_of_combined_topics_publisher);
  if (it != m_combined_topics_publishers_and_info.end()) {
    m_combined_topics_publishers_and_info.erase(it);
    RCLCPP_INFO(this->get_logger(),
                "Removed combined publisher: %s",
                request->name_of_combined_topics_publisher.c_str());
    response->success = true;

    m_yaml_config_manager->removeCombinedPublisherFromFile("config_combined.yaml",
                                                           request->name_of_combined_topics_publisher);
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Combined publisher not found: %s",
                request->name_of_combined_topics_publisher.c_str());
    response->success = false;
  }
}

bool AutomaticActionService::parseExpression(const std::string& logic_expression,
                                             const std::unordered_map<std::string, bool>& topic_values) {
  std::stack<bool> value_stack;
  std::stack<std::string> operator_stack;

  std::istringstream iss(logic_expression);
  std::string token;

  auto apply_operator = [](bool left, bool right, const std::string& op) {
    if (op == "and") {
      return left && right;
    } else if (op == "or") {
      return left || right;
    }
    throw std::runtime_error("Unknown operator: " + op);
  };

  while (iss >> token) {
    if (token == "(") {
      operator_stack.push(token);
    } else if (token == ")") {
      while (!operator_stack.empty() && operator_stack.top() != "(") {
        if (value_stack.size() < 2) {
          throw std::runtime_error("Invalid expression: not enough values for operation");
        }
        bool right = value_stack.top();
        value_stack.pop();
        bool left = value_stack.top();
        value_stack.pop();
        std::string op = operator_stack.top();
        operator_stack.pop();
        value_stack.push(apply_operator(left, right, op));
      }
      if (operator_stack.empty()) {
        throw std::runtime_error("Mismatched parentheses");
      }
      operator_stack.pop(); // Remove the "("
    } else if (token == "and" || token == "or") {
      while (!operator_stack.empty() && operator_stack.top() != "(") {
        if (value_stack.size() < 2) {
          throw std::runtime_error("Invalid expression: not enough values for operation");
        }
        bool right = value_stack.top();
        value_stack.pop();
        bool left = value_stack.top();
        value_stack.pop();
        std::string op = operator_stack.top();
        operator_stack.pop();
        value_stack.push(apply_operator(left, right, op));
      }
      operator_stack.push(token);
    } else {
      token.erase(std::remove_if(token.begin(), token.end(), [](char c) { return c == '(' || c == ')'; }),
                  token.end()); // Remove potential parentheses around topics

      if (topic_values.find(token) != topic_values.end()) {
        value_stack.push(topic_values.at(token));
      } else {
        throw std::runtime_error("Unknown topic in expression: " + token);
      }
    }
  }

  while (!operator_stack.empty()) {
    if (value_stack.size() < 2) {
      throw std::runtime_error("Invalid expression: not enough values for operation");
    }
    bool right = value_stack.top();
    value_stack.pop();
    bool left = value_stack.top();
    value_stack.pop();
    std::string op = operator_stack.top();
    operator_stack.pop();
    value_stack.push(apply_operator(left, right, op));
  }

  if (value_stack.size() != 1) {
    throw std::runtime_error("Invalid expression: mismatched operators and values");
  }

  return value_stack.top();
}

void AutomaticActionService::fillCurrentDateTime(FullDateTime& message) {
  auto now = std::chrono::system_clock::now();

  std::time_t time_now = std::chrono::system_clock::to_time_t(now);
  std::tm local_time = *std::localtime(&time_now);

  message.year = local_time.tm_year + 1900;
  message.month = local_time.tm_mon + 1;
  message.day = local_time.tm_mday;
  message.hour = local_time.tm_hour;
  message.minute = local_time.tm_min;
  message.second = local_time.tm_sec;

  auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  message.nanosecond = now_ns.time_since_epoch().count() % 1000000000;
}

void AutomaticActionService::messageConnectionCallback() {
  auto pub_msg = std::make_shared<std_msgs::msg::String>();
  pub_msg->data = "Combined action triggered";
  auto notification_msg = std::make_shared<Notification>();

  for (const auto& pair : m_combined_topics_publishers_and_info) {
    const auto& combined_publisher = pair.second;
    std::unordered_map<std::string, bool> topic_values;
    notification_msg->source = combined_publisher->config.action_and_publisher_name;
    notification_msg->info = combined_publisher->config.trigger_text;
    notification_msg->publication_method = combined_publisher->config.publication_method;
    notification_msg->severity = Notification::WARNING;

    for (const auto& topic : combined_publisher->config.listen_topics) {
      auto it = m_topics_data.find(topic);

      if (it == m_topics_data.end()) {
        RCLCPP_ERROR(this->get_logger(), "Topic data for %s not found", topic.c_str());
        topic_values[topic] = false;
        continue;
      }

      std::shared_ptr<OneTopicData> topic_data = it->second;
      int64_t elapsed_time_ms = (this->now().nanoseconds() - topic_data->time_stamp.nanoseconds()) / 1000000;
      bool is_valid = topic_data->action_triggered && (elapsed_time_ms <= topic_data->data_validity_ms);
      topic_values[topic] = is_valid;
    }

    bool expression_result = parseExpression(combined_publisher->config.logic_expression, topic_values);

    if (expression_result) {
      combined_publisher->publisher->publish(*pub_msg);
      m_notification_publisher->publish(*notification_msg);
      auto it = m_combined_topics_publishers_and_info.find(combined_publisher->config.action_and_publisher_name);
      if (it != m_combined_topics_publishers_and_info.end()) {
        fillCurrentDateTime(it->second->config.date_and_time);
        auto config_ptr = std::make_shared<AutomaticActionCombinedRequestAndTime>(it->second->config);
        m_yaml_config_manager->modifyCombinedConfigurationInFile("config_combined.yaml", config_ptr);
      }
      RCLCPP_INFO(this->get_logger(),
                  "Published message to %s",
                  combined_publisher->config.action_and_publisher_name.c_str());
      for (const auto& topic : combined_publisher->config.listen_topics) {
        m_topics_data[topic]->action_triggered = false;
      }
    }
  }
}

void AutomaticActionService::handleChangeAutomaticActionCombinedService(
    const std::shared_ptr<ChangeAutomaticActionCombined::Request> request,
    std::shared_ptr<ChangeAutomaticActionCombined::Response> response) {

  auto it = m_combined_topics_publishers_and_info.find(request->action_and_publisher_name_to_change);
  if (it == m_combined_topics_publishers_and_info.end()) {
    RCLCPP_WARN(this->get_logger(),
                "Combined publisher not found: %s",
                request->action_and_publisher_name_to_change.c_str());
    response->success = false;
    response->error_message = "Combined publisher not found: " + request->action_and_publisher_name_to_change;
    return;
  }

  bool needs_recreation = request->new_action_and_publisher_name != request->action_and_publisher_name_to_change;

  bool all_topics_exist =
      std::all_of(request->listen_topics.begin(), request->listen_topics.end(), [this](const std::string& topic) {
        return m_configurations.find(topic) != m_configurations.end();
      });

  if (!all_topics_exist) {
    RCLCPP_WARN(this->get_logger(), "One or more topics in listen_topics do not exist in configurations.");
    response->success = false;
    response->error_message = "One or more topics in listen_topics do not exist in configurations.";
    return;
  }

  if (needs_recreation) {
    auto delete_request = std::make_shared<AutomaticActionCombinedDelete::Request>();
    delete_request->name_of_combined_topics_publisher = request->action_and_publisher_name_to_change;
    auto delete_response = std::make_shared<AutomaticActionCombinedDelete::Response>();
    handleCombinedDeleteService(delete_request, delete_response);

    if (!delete_response->success) {
      RCLCPP_WARN(this->get_logger(), "Failed to delete old combined publisher before recreation.");
      response->success = false;
      response->error_message = "Failed to delete old combined publisher before recreation.";
      return;
    }

    auto create_request = std::make_shared<AutomaticActionConnection::Request>();
    create_request->listen_topics = request->listen_topics;
    create_request->logic_expression = request->logic_expression;
    create_request->action_and_publisher_name = request->new_action_and_publisher_name;
    create_request->trigger_text = request->trigger_text;
    create_request->publication_method = request->publication_method;
    auto create_response = std::make_shared<AutomaticActionConnection::Response>();
    handleCombinedService(create_request, create_response);

    response->success = create_response->success;
  } else {
    auto& combined_publisher = it->second;

    if (request->listen_topics != combined_publisher->config.listen_topics) {
      combined_publisher->config.listen_topics = request->listen_topics;
    }
    if (request->logic_expression != combined_publisher->config.logic_expression) {
      combined_publisher->config.logic_expression = request->logic_expression;
    }
    if (request->trigger_text != combined_publisher->config.trigger_text) {
      combined_publisher->config.trigger_text = request->trigger_text;
    }
    if (request->publication_method != combined_publisher->config.publication_method) {
      combined_publisher->config.publication_method = request->publication_method;
    }

    auto combined_publisher_convert = std::make_shared<AutomaticActionCombinedRequestAndTime>();
    combined_publisher_convert->listen_topics = request->listen_topics;
    combined_publisher_convert->logic_expression = request->logic_expression;
    combined_publisher_convert->action_and_publisher_name = request->new_action_and_publisher_name;
    combined_publisher_convert->trigger_text = request->trigger_text;
    combined_publisher_convert->publication_method = request->publication_method;

    m_yaml_config_manager->modifyCombinedConfigurationInFile("config_combined.yaml", combined_publisher_convert);

    response->success = true;
  }
}

void AutomaticActionService::handleAvailableTopicsCombinedService(
    std::shared_ptr<AvailableTopicsCombined::Response> response) {
  for (const auto& pair : m_combined_topics_publishers_and_info) {
    response->available_combined_topics_with_parameters_and_time.push_back(pair.second->config);
  }
}

void AutomaticActionService::messageCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg,
                                             const std::shared_ptr<AutomaticActionRequestAndTime> config) {
  auto introspection_ts = m_value_extractor.getIntrospectionTypeSupport(config->listen_message_type);
  if (introspection_ts.members == nullptr) {
    RCLCPP_ERROR(this->get_logger(),
                 "Unable to get introspection type support for: %s",
                 config->listen_message_type.c_str());
    return;
  }

  auto value = m_value_extractor.extractValue(*msg, introspection_ts, config->value);
  if (!value.has_value()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract value for field: %s", config->value.c_str());
    return;
  }

  std::variant<double,
               float,
               int8_t,
               int16_t,
               int32_t,
               int64_t,
               uint8_t,
               uint16_t,
               uint32_t,
               uint64_t,
               bool,
               char,
               std::string>
      field_value;

  try {
    if (auto v = std::any_cast<double>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<float>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<int8_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<int16_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<int32_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<int64_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<uint8_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<uint16_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<uint32_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<uint64_t>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<bool>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<char>(&*value)) {
      field_value = *v;
    } else if (auto v = std::any_cast<std::string>(&*value)) {
      field_value = *v;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported type for field: %s", config->value.c_str());
      return;
    }
  } catch (const std::bad_any_cast& e) {
    RCLCPP_ERROR(this->get_logger(), "Bad any cast: %s", e.what());
    return;
  }
  bool trigger_condition = false;
  double trigger_val = std::stod(config->trigger_val);

  std::visit(
      [&](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_arithmetic_v<T>) {
          if (config->trigger_type == AutomaticActionService::LESS_THAN_TRIGGER) {
            trigger_condition = arg < static_cast<T>(trigger_val);
          } else if (config->trigger_type == AutomaticActionService::GREATER_THAN_TRIGGER) {
            trigger_condition = arg > static_cast<T>(trigger_val);
          } else if (config->trigger_type == AutomaticActionService::EQUAL_TO_TRIGGER) {
            trigger_condition = arg == static_cast<T>(trigger_val);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported trigger type: %s", config->trigger_type.c_str());
          }
        } else if constexpr (std::is_same_v<T, std::string>) {
          if (config->trigger_type == AutomaticActionService::EQUAL_TO_TRIGGER) {
            trigger_condition = arg == config->trigger_val;
          } else {
            RCLCPP_ERROR(this->get_logger(), "String comparisons are only supported with 'EqualTo'");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Unsupported type for field: %s", config->trigger_type.c_str());
        }
      },
      field_value);

  auto data_validity_ms = config->data_validity_ms;

  if (trigger_condition) {
    performAction(config);
    auto topic_data = std::make_shared<OneTopicData>();
    topic_data->action_triggered = true;
    topic_data->time_stamp = this->now();
    topic_data->data_validity_ms = data_validity_ms;
    fillCurrentDateTime(topic_data->time_stamp_local);
    auto it = m_configurations.find(config->action_and_publisher_name);
    if (it == m_configurations.end()) {
      RCLCPP_ERROR(this->get_logger(), "Topic data for %s not found", config->action_and_publisher_name.c_str());
      return;
    } else {
      it->second->date_and_time = topic_data->time_stamp_local;
      std::cout << "Date and time: " << it->second->date_and_time.year << "-" << it->second->date_and_time.month << "-"
                << it->second->date_and_time.day << " " << it->second->date_and_time.hour << ":"
                << it->second->date_and_time.minute << ":" << it->second->date_and_time.second << "."
                << it->second->date_and_time.nanosecond << std::endl;
      m_yaml_config_manager->modifyConfigurationInFile("config.yaml", it->second);
    }

    m_topics_data[config->action_and_publisher_name] = topic_data;
    messageConnectionCallback(); // function to check all combined subscribers
  }
}

void AutomaticActionService::performAction(const std::shared_ptr<AutomaticActionRequestAndTime> config) {
  auto pub_msg = std::make_shared<std_msgs::msg::String>();
  pub_msg->data = config->trigger_text;
  auto notification_msg = std::make_shared<Notification>();
  notification_msg->source = config->action_and_publisher_name;
  notification_msg->info = config->trigger_text;
  notification_msg->publication_method = config->publication_method;
  notification_msg->severity = Notification::WARNING;
  m_notification_publisher->publish(*notification_msg);

  if (m_publishers.find(config->action_and_publisher_name) == m_publishers.end()) {
    auto publisher = this->create_publisher<std_msgs::msg::String>(config->action_and_publisher_name, 10);
    m_publishers[config->action_and_publisher_name] = publisher;
  }

  m_publishers[config->action_and_publisher_name]->publish(*pub_msg);
  RCLCPP_INFO(this->get_logger(), "Published message to %s", config->action_and_publisher_name.c_str());
}
