/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef VALUE_EXTRACTOR_HPP
#define VALUE_EXTRACTOR_HPP
#include <algorithm>
#include <any>
#include <dlfcn.h>
#include <functional>
#include <iostream>
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
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <variant>

class ValueExtractor : public rclcpp::Node {
public:
  // Constructor for the ValueExtractor class
  ValueExtractor();

  // Destructor for the ValueExtractor class
  ~ValueExtractor();

  // Structure to store the type support information
  struct TypeSupportInfo {
    const rosidl_typesupport_introspection_cpp::MessageMembers* members;
    const rosidl_message_type_support_t* type_support_handle;
    void* introspection_library_handle;
    void* type_support_library_handle;
  };

  // Function to extract the value from the serialized message
  //
  // @param msg The serialized message
  //
  // @param type_support_info The type support information
  //
  // @param field The field to extract the value from
  //
  // @return The extracted value
  std::optional<std::any> extractValue(const rclcpp::SerializedMessage& msg,
                                       const TypeSupportInfo& type_support_info,
                                       const std::string& field);

  // Function to get the introspection type support
  //
  // @param type The type to get the introspection type support for
  TypeSupportInfo getIntrospectionTypeSupport(const std::string& type);

private:

  // Function to extract the value recursively
  //
  // @param data The data to extract the value from
  //
  // @param members The message members
  //
  // @param fields The fields to extract the value from
  //
  // @param index The index to extract the value from
  std::optional<std::any> extractValueRecursive(const uint8_t* data,
                                                const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                                                const std::vector<std::string>& fields,
                                                size_t index);

  // Function to extract the array value
  //
  // @param data The data to extract the value from
  //
  // @param member The message member
  //
  // @param fields The fields to extract the value from
  //
  // @param index The index to extract the value from
  //
  // @param array_condition The array condition
  std::optional<std::any> extractArrayValue(const uint8_t* data,
                                            const rosidl_typesupport_introspection_cpp::MessageMember& member,
                                            const std::vector<std::string>& fields,
                                            size_t index,
                                            const std::optional<std::string>& array_condition);

  // Function to compare the values
  //
  // @param value_any The value to compare
  //
  // @param condition_value The condition value
  bool compareValues(const std::any& value_any, const std::string& condition_value);

  // Function to extract the primitive value
  //
  // @param data The data to extract the value from
  //
  // @param type_id The type id of the value
  std::optional<std::any> extractPrimitiveValue(const uint8_t* data, uint8_t type_id);

  TypeSupportInfo m_type_support_info;
};

#endif // VALUE_EXTRACTOR_HPP
