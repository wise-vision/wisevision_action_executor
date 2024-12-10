// Copyright (c) 2024, WiseVision. All rights reserved.
#ifndef YAML_CONFIG_MANAGER_HPP
#define YAML_CONFIG_MANAGER_HPP

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include "data_structures.hpp"
#include "lora_msgs/msg/automatic_action_request_and_time.hpp"
#include "lora_msgs/srv/automatic_action.hpp"
#include "lora_msgs/srv/automatic_action_combined_delete.hpp"
#include "lora_msgs/srv/automatic_action_connection.hpp"
#include "lora_msgs/srv/automatic_action_delete.hpp"

using AutomaticAction = lora_msgs::srv::AutomaticAction;
using AutomaticActionConnection = lora_msgs::srv::AutomaticActionConnection;
using AutomaticActionRequestAndTime = lora_msgs::msg::AutomaticActionRequestAndTime;

class YamlConfigManager {
public:
  // Function to save the configuration of subscribed topics to a yaml file
  //
  // @param filename: the name of the file to save the configuration to
  //
  // @param config: the configuration to save
  void saveConfigurationToFile(const std::string& filename,
                               const std::shared_ptr<AutomaticActionRequestAndTime>& config);

  // Function to load the configuration of subscribed topics from a yaml file
  //
  // @param filename: the name of the file to load the configuration from
  //
  // @param configurations: the map to store the configurations
  //
  // @param topics_data: the map to store the data of the topics
  void loadConfigurationFromFile(
      const std::string& filename,
      std::unordered_map<std::string, std::shared_ptr<AutomaticActionRequestAndTime>>& configurations,
      std::unordered_map<std::string, std::shared_ptr<OneTopicData>>& topics_data);

  // Function to modify the configuration of subscribed topics in a yaml file
  //
  // @param filename: the name of the file to modify the configuration in
  //
  // @param config: the configuration to modify
  void modifyConfigurationInFile(const std::string& filename,
                                 const std::shared_ptr<AutomaticActionRequestAndTime>& config);

  // Function to save the configuration of combined publishers to a yaml file
  //
  // @param filename: the name of the file to save the configuration to
  //
  // @param combined_topics_publishers_and_info: the configuration to save
  void saveCombinedConfigurationToFile(const std::string& filename,
                                       const std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>>&
                                           combined_topics_publishers_and_info);

  // Function to load the configuration of combined publishers from a yaml file
  //
  // @param filename: the name of the file to load the configuration from
  //
  // @param combined_topics_publishers_and_info: the map to store the configurations
  void loadCombinedConfigurationFromFile(
      const std::string& filename,
      std::unordered_map<std::string, std::shared_ptr<CombinedTopicsPublisher>>& combined_topics_publishers_and_info);

  // Function to modify the configuration of combined publishers in a yaml file
  //
  // @param filename: the name of the file to modify the configuration in
  //
  // @param config: the configuration to modify
  void modifyCombinedConfigurationInFile(const std::string& filename,
                                         const std::shared_ptr<AutomaticActionCombinedRequestAndTime>& config);

  // Function to remove a subscription from a yaml file
  //
  // @param filename: the name of the file to remove the subscription from
  //
  // @param topic_to_remove: the topic to remove
  void removeSubscriptionFromFile(const std::string& filename, const std::string& topic_to_remove);

  // Function to remove a combined publisher from a yaml file
  //
  // @param filename: the name of the file to remove the combined publisher from
  //
  // @param pub_topic_to_remove: the combined publisher to remove
  void removeCombinedPublisherFromFile(const std::string& filename, const std::string& pub_topic_to_remove);
};

#endif // YAML_CONFIG_MANAGER_HPP
