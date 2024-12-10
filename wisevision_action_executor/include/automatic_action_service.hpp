// Copyright (c) 2024, WiseVision. All rights reserved.
// Code prove functionality of the AutomaticActionService class, which is a ROS2 node that provides services to create,
// delete, and connect to topics. The class also provides a service to get the available topics and a service to delete
// a combined publisher. The class has a constructor that initializes the services and loads the configurations from the
// yaml files.
#ifndef AUTOMATIC_ACTION_SERVICE_HPP
#define AUTOMATIC_ACTION_SERVICE_HPP

#include <algorithm>
#include <any>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_cpp/message_type_support_dispatch.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <stack>
#include <string>
#include <unordered_map>
#include <variant>

#include "data_structures.hpp"
#include "gtest/gtest.h"
#include "lora_msgs/msg/automatic_action_request_and_time.hpp"
#include "lora_msgs/srv/automatic_action.hpp"
#include "lora_msgs/srv/automatic_action_combined_delete.hpp"
#include "lora_msgs/srv/automatic_action_connection.hpp"
#include "lora_msgs/srv/automatic_action_delete.hpp"
#include "lora_msgs/srv/available_topics.hpp"
#include "lora_msgs/srv/available_topics_combined.hpp"
#include "lora_msgs/srv/change_automatic_action.hpp"
#include "lora_msgs/srv/change_automatic_action_combined.hpp"
#include "notification_msgs/msg/notification.hpp"
#include "value_extractor.hpp"
#include "yaml_config_manager.hpp"

using AutomaticAction = lora_msgs::srv::AutomaticAction;
using AutomaticActionDelete = lora_msgs::srv::AutomaticActionDelete;
using AutomaticActionConnection = lora_msgs::srv::AutomaticActionConnection;
using AvailableTopics = lora_msgs::srv::AvailableTopics;
using AutomaticActionCombinedDelete = lora_msgs::srv::AutomaticActionCombinedDelete;
using Notification = notification_msgs::msg::Notification;
using AutomaticActionRequestAndTime = lora_msgs::msg::AutomaticActionRequestAndTime;
using AvailableTopicsCombined = lora_msgs::srv::AvailableTopicsCombined;
using ChangeAutomaticAction = lora_msgs::srv::ChangeAutomaticAction;
using ChangeAutomaticActionCombined = lora_msgs::srv::ChangeAutomaticActionCombined;

class ExtractValueTest;

class AutomaticActionService : public rclcpp::Node {
public:
  // Constructor, which initializes the services and loads the configurations from the yaml files
  AutomaticActionService();

  // Intiliazes existing subscriptions
  void initializeSubscriptions();

  // Intiliazes existing combined publishers
  void initializeCombinedPublishers();

  static constexpr const char* LESS_THAN_TRIGGER = "LessThan";
  static constexpr const char* GREATER_THAN_TRIGGER = "GreaterThan";
  static constexpr const char* EQUAL_TO_TRIGGER = "EqualTo";

private:
  // Function to handle the create service request, which creates a new subscription to a topic
  //
  // @param request: the request message containing the topic name and the action to be performed
  //
  // @param response: the response message containing the result of the service
  void handleCreateService(const std::shared_ptr<AutomaticAction::Request> request,

                           std::shared_ptr<AutomaticAction::Response> response);

  // Function to handle the delete service request, which deletes a subscription to a topic
  //
  // @param request: the request message containing the topic name
  //
  // @param response: the response message containing the result of the service
  void handleDeleteService(const std::shared_ptr<AutomaticActionDelete::Request> request,

                           std::shared_ptr<AutomaticActionDelete::Response> response);

  void messageCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg,
                       const std::shared_ptr<AutomaticActionRequestAndTime> config);

  // Function to handle the connection service request, which creates a publisher for multiple topics
  //
  // @param request: the request message containing the topics and logical expression to be performed
  //
  // @param response: the response message containing the result of the service
  void handleCombinedService(const std::shared_ptr<AutomaticActionConnection::Request> request,
                             std::shared_ptr<AutomaticActionConnection::Response> response);

  // Function to handle the available topics service request, which returns the available topics
  //
  // @param response: the response message containing the available topics
  void handleAvailableTopicsService(std::shared_ptr<AvailableTopics::Response> response);

  // Function to handle the combined delete service request, which deletes a publisher for multiple topics
  //
  // @param request: the request message containing the name of the publisher
  //
  // @param response: the response message containing the result of the service
  void handleCombinedDeleteService(const std::shared_ptr<AutomaticActionCombinedDelete::Request> request,
                                   std::shared_ptr<AutomaticActionCombinedDelete::Response> response);

  // Function to handle the available topics combined service request, which returns the available topics with
  // parameters
  //
  // @param response: the response message containing the available topics with parameters
  void handleAvailableTopicsCombinedService(std::shared_ptr<AvailableTopicsCombined::Response> response);

  // Function to handle the change automatic action service request, which changes the configuration of a subscription
  //
  // @param request: the request message containing the new configuration
  //
  // @param response: the response message containing the result of the service
  void handleChangeAutomaticActionService(const std::shared_ptr<ChangeAutomaticAction::Request> request,
                                          std::shared_ptr<ChangeAutomaticAction::Response> response);

  // Function to handle the change automatic action combined service request, which changes the configuration of a
  // publisher
  //
  // @param request: the request message containing the new configuration
  //
  // @param response: the response message containing the result of the service
  void handleChangeAutomaticActionCombinedService(const std::shared_ptr<ChangeAutomaticActionCombined::Request> request,
                                                  std::shared_ptr<ChangeAutomaticActionCombined::Response> response);

  // Function to parse the logical expression and return the result
  //
  // @param logic_expression: the logical expression to be parsed
  //
  // @param topic_values: the values of the topics to be used in the logical expression
  bool parseExpression(const std::string& logic_expression, const std::unordered_map<std::string, bool>& topic_values);

  // Function creates a publisher for multiple topics
  void messageConnectionCallback();

  // Function performs the action specified in the configuration
  //
  // @param config: the configuration containing the action to be performed
  void performAction(const std::shared_ptr<AutomaticActionRequestAndTime> config);

  // Function replaces the exact string in the source string
  //
  // @param source: the source string
  //
  // @param from: the string to be replaced
  //
  // @param to: the string to replace with
  void replaceExactString(std::string& source, const std::string& from, const std::string& to);

  // Function fills the current date and time in the message
  //
  // @param message: the message to be filled with the current date and time
  void fillCurrentDateTime(FullDateTime& message);

  std::shared_ptr<YamlConfigManager> m_yaml_config_manager;
  ValueExtractor m_value_extractor;
  rclcpp::Service<AutomaticAction>::SharedPtr m_add_automatic_action_service;
  rclcpp::Service<AutomaticActionDelete>::SharedPtr m_delete_service;
  rclcpp::Service<AutomaticActionConnection>::SharedPtr m_connection_service;
  rclcpp::Service<AvailableTopics>::SharedPtr m_available_topics_service;
  rclcpp::Service<AvailableTopicsCombined>::SharedPtr m_available_topics_combined_service;
  rclcpp::Service<AutomaticActionCombinedDelete>::SharedPtr m_combined_delete_service;
  rclcpp::Service<ChangeAutomaticAction>::SharedPtr m_change_automatic_action_service;
  rclcpp::Service<ChangeAutomaticActionCombined>::SharedPtr m_change_automatic_action_combined_service;
  rclcpp::Publisher<Notification>::SharedPtr m_notification_publisher;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> m_subscriptions;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> m_publishers;
  std::unordered_map<std::string, std::shared_ptr<AutomaticActionRequestAndTime>> m_configurations;
  std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>> m_combined_topics_publishers_and_info;
  std::unordered_map<std::string, std::shared_ptr<OneTopicData>> m_topics_data;
};

#endif // AUTOMATIC_ACTION_SERVICE_HPP
