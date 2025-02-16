/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "value_extractor.hpp"

ValueExtractor::ValueExtractor() : Node("value_extractor") {
  m_type_support_info.members = nullptr;
  m_type_support_info.type_support_handle = nullptr;
  m_type_support_info.introspection_library_handle = nullptr;
  m_type_support_info.type_support_library_handle = nullptr;
}

ValueExtractor::~ValueExtractor() {
  if (m_type_support_info.introspection_library_handle) {
    dlclose(m_type_support_info.introspection_library_handle);
  }
  if (m_type_support_info.type_support_library_handle) {
    dlclose(m_type_support_info.type_support_library_handle);
  }
}

std::optional<std::any> ValueExtractor::extractValue(const rclcpp::SerializedMessage& msg,
                                                     const TypeSupportInfo& type_support_info,
                                                     const std::string& field) {
  const auto* members = type_support_info.members;
  const auto* type_support_handle = type_support_info.type_support_handle;

  std::istringstream iss(field);
  std::string token;
  std::vector<std::string> fields;

  while (std::getline(iss, token, '.')) {
    fields.push_back(token);
  }

  void* message = malloc(members->size_of_);
  if (!message) {
    RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for the message.");
    return std::nullopt;
  }
  memset(message, 0, members->size_of_);
  members->init_function(message, rosidl_runtime_cpp::MessageInitialization::ALL);

  rclcpp::SerializationBase serializer(type_support_handle);
  rclcpp::SerializedMessage tmp_msg(msg);
  serializer.deserialize_message(&tmp_msg, message);

  auto result = extractValueRecursive(reinterpret_cast<const uint8_t*>(message), members, fields, 0);

  members->fini_function(message);
  free(message);

  return result;
}

std::optional<std::any>
ValueExtractor::extractValueRecursive(const uint8_t* data,
                                      const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                                      const std::vector<std::string>& fields,
                                      size_t index) {
  if (index >= fields.size()) {
    return std::nullopt;
  }

  std::string current_field = fields[index];
  std::string field_name = current_field;
  std::optional<std::string> array_subscript;

  size_t pos_start = current_field.find('[');
  size_t pos_end = current_field.find(']');
  if (pos_start != std::string::npos && pos_end != std::string::npos && pos_end > pos_start) {
    field_name = current_field.substr(0, pos_start);
    array_subscript = current_field.substr(pos_start + 1, pos_end - pos_start - 1);
  }

  auto member_it = std::find_if(
      members->members_,
      members->members_ + members->member_count_,
      [&](const rosidl_typesupport_introspection_cpp::MessageMember& member) { return member.name_ == field_name; });

  if (member_it == members->members_ + members->member_count_) {
    RCLCPP_ERROR(this->get_logger(), "Field '%s' not found.", field_name.c_str());
    return std::nullopt;
  }

  const auto& member = *member_it;
  const uint8_t* member_data = data + member.offset_;

  if (member.is_array_) {
    return extractArrayValue(member_data, member, fields, index, array_subscript);
  } else if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    const rosidl_typesupport_introspection_cpp::MessageMembers* sub_members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
    return extractValueRecursive(member_data, sub_members, fields, index + 1);
  } else {
    if (index + 1 == fields.size()) {
      return extractPrimitiveValue(member_data, member.type_id_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Field '%s' is not a nested message.", member.name_);
      return std::nullopt;
    }
  }
}

std::optional<std::any>
ValueExtractor::extractArrayValue(const uint8_t* data,
                                  const rosidl_typesupport_introspection_cpp::MessageMember& member,
                                  const std::vector<std::string>& fields,
                                  size_t index,
                                  const std::optional<std::string>& array_subscript) {
  size_t array_size = 0;

  if (member.is_array_) {
    if (member.array_size_ == 0 || member.is_upper_bound_) {
      array_size = member.size_function(const_cast<void*>(reinterpret_cast<const void*>(data)));
    } else {
      array_size = member.array_size_;
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Field is not an array or sequence.");
    return std::nullopt;
  }

  const rosidl_typesupport_introspection_cpp::MessageMembers* element_members = nullptr;

  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    element_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
  }

  if (array_subscript) {
    const std::string& subscript = *array_subscript;
    if (subscript.find('=') != std::string::npos) {
      std::string condition_field, condition_value;
      size_t eq_pos = subscript.find('=');
      if (eq_pos != std::string::npos) {
        condition_field = subscript.substr(0, eq_pos);
        condition_value = subscript.substr(eq_pos + 1);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid condition: '%s'.", subscript.c_str());
        return std::nullopt;
      }

      std::vector<std::string> condition_fields;
      std::istringstream iss(condition_field);
      std::string token;
      while (std::getline(iss, token, '.')) {
        condition_fields.push_back(token);
      }

      for (size_t i = 0; i < array_size; ++i) {
        const void* element_ptr = member.get_const_function(const_cast<void*>(reinterpret_cast<const void*>(data)), i);
        const uint8_t* element_data = reinterpret_cast<const uint8_t*>(element_ptr);

        auto condition_value_any = extractValueRecursive(element_data, element_members, condition_fields, 0);
        if (!condition_value_any.has_value()) {
          continue;
        }

        bool condition_met = compareValues(*condition_value_any, condition_value);

        if (condition_met) {
          if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            if (index + 1 == fields.size()) {
              return std::any(element_data);
            }
            return extractValueRecursive(element_data, element_members, fields, index + 1);
          } else {
            if (index + 1 == fields.size()) {
              return extractPrimitiveValue(element_data, member.type_id_);
            } else {
              RCLCPP_ERROR(this->get_logger(), "Element in array is not a message; cannot proceed.");
              return std::nullopt;
            }
          }
        }
      }

      RCLCPP_ERROR(this->get_logger(), "No element satisfying the condition '%s' found.", subscript.c_str());
      return std::nullopt;

    } else {
      try {
        size_t array_index = std::stoul(subscript);
        if (array_index >= array_size) {
          RCLCPP_ERROR(this->get_logger(), "Array index out of bounds: %zu", array_index);
          return std::nullopt;
        }

        const void* element_ptr =
            member.get_const_function(const_cast<void*>(reinterpret_cast<const void*>(data)), array_index);
        const uint8_t* element_data = reinterpret_cast<const uint8_t*>(element_ptr);

        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
          return extractValueRecursive(element_data, element_members, fields, index + 1);
        } else {
          if (index + 1 == fields.size()) {
            return extractPrimitiveValue(element_data, member.type_id_);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Unexpected additional fields after array index.");
            return std::nullopt;
          }
        }
      } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid array index: '%s'", subscript.c_str());
        return std::nullopt;
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "No index or condition provided for array.");
    return std::nullopt;
  }
}

bool ValueExtractor::compareValues(const std::any& value_any, const std::string& condition_value) {
  try {
    if (value_any.type() == typeid(bool)) {
      bool value = std::any_cast<bool>(value_any);
      bool cond_value = (condition_value == "true" || condition_value == "1");
      return (value == cond_value);
    } else if (value_any.type() == typeid(uint8_t)) {
      uint8_t value = std::any_cast<uint8_t>(value_any);
      uint64_t cond_value = std::stoull(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(int8_t)) {
      int8_t value = std::any_cast<int8_t>(value_any);
      int64_t cond_value = std::stoll(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(uint16_t)) {
      uint16_t value = std::any_cast<uint16_t>(value_any);
      uint64_t cond_value = std::stoull(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(int16_t)) {
      int16_t value = std::any_cast<int16_t>(value_any);
      int64_t cond_value = std::stoll(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(uint32_t)) {
      uint32_t value = std::any_cast<uint32_t>(value_any);
      uint64_t cond_value = std::stoull(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(int32_t)) {
      int32_t value = std::any_cast<int32_t>(value_any);
      int64_t cond_value = std::stoll(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(uint64_t)) {
      uint64_t value = std::any_cast<uint64_t>(value_any);
      uint64_t cond_value = std::stoull(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(int64_t)) {
      int64_t value = std::any_cast<int64_t>(value_any);
      int64_t cond_value = std::stoll(condition_value);
      return (value == cond_value);
    } else if (value_any.type() == typeid(float)) {
      float value = std::any_cast<float>(value_any);
      float cond_value = std::stof(condition_value);
      const float EPSILON = 1e-6f; // Adjust as needed
      return (std::fabs(value - cond_value) < EPSILON);
    } else if (value_any.type() == typeid(double)) {
      double value = std::any_cast<double>(value_any);
      double cond_value = std::stod(condition_value);
      const double EPSILON = 1e-9; // Adjust as needed
      return (std::fabs(value - cond_value) < EPSILON);
    } else if (value_any.type() == typeid(std::string)) {
      std::string value = std::any_cast<std::string>(value_any);
      return (value == condition_value);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported condition field type.");
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in compareValues: %s", e.what());
    return false;
  }
}

std::optional<std::any> ValueExtractor::extractPrimitiveValue(const uint8_t* data, uint8_t type_id) {
  switch (type_id) {
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
    return std::any(*reinterpret_cast<const bool*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    return std::any(*reinterpret_cast<const uint8_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
    return std::any(*reinterpret_cast<const uint16_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    return std::any(*reinterpret_cast<const uint32_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    return std::any(*reinterpret_cast<const uint64_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
    return std::any(*reinterpret_cast<const int8_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
    return std::any(*reinterpret_cast<const int16_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
    return std::any(*reinterpret_cast<const int32_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
    return std::any(*reinterpret_cast<const int64_t*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
    return std::any(*reinterpret_cast<const float*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
    return std::any(*reinterpret_cast<const double*>(data));
  case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
    const std::string* str = reinterpret_cast<const std::string*>(data);
    return std::any(*str);
  }
  default:
    RCLCPP_ERROR(this->get_logger(), "Unsupported data type: %d", type_id);
    return std::nullopt;
  }
}

ValueExtractor::TypeSupportInfo
ValueExtractor::getIntrospectionTypeSupport(const std::string& type)
{
  auto first_pos = type.find('/');
  if (first_pos == std::string::npos) {
    throw std::runtime_error("Invalid message type format.");
  }
  auto second_pos = type.find('/', first_pos + 1);
  if (second_pos == std::string::npos) {
    throw std::runtime_error("Invalid message type format.");
  }
  std::string package_name = type.substr(0, first_pos);
  std::string interface_type = type.substr(first_pos + 1, second_pos - first_pos - 1);
  std::string message_name = type.substr(second_pos + 1);

  std::string introspection_library_name = "lib" + package_name + "__rosidl_typesupport_introspection_cpp.so";
  void* introspection_handle = dlopen(introspection_library_name.c_str(), RTLD_LAZY);
  if (!introspection_handle) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open introspection library: %s", dlerror());
    return { nullptr, nullptr, nullptr, nullptr };
  }

  std::string introspection_symbol_name = "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
                                          package_name + "__" + interface_type + "__" + message_name;

  auto get_introspection_ts = (const rosidl_message_type_support_t*(*)())dlsym(introspection_handle, introspection_symbol_name.c_str());
  const char* introspection_dlsym_error = dlerror();
  if (introspection_dlsym_error) {
    RCLCPP_ERROR(this->get_logger(), "Cannot load introspection symbol: %s", introspection_dlsym_error);
    dlclose(introspection_handle);
    return { nullptr, nullptr, nullptr, nullptr };
  }

  const rosidl_message_type_support_t* introspection_ts = get_introspection_ts();

  std::string rmw_implementation(rmw_get_implementation_identifier());
  std::string type_support_impl;

  if (rmw_implementation == "rmw_fastrtps_cpp") {
    type_support_impl = "fastrtps_cpp";
  } else if (rmw_implementation == "rmw_cyclonedds_cpp") {
    type_support_impl = "cyclonedds_cpp";
  } else if (rmw_implementation == "rmw_connext_cpp") {
    type_support_impl = "connext_cpp";
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported RMW: %s", rmw_implementation.c_str());
    dlclose(introspection_handle);
    return { nullptr, nullptr, nullptr, nullptr };
  }

  std::string type_support_library_name = "lib" + package_name + "__rosidl_typesupport_" + type_support_impl + ".so";
  void* type_support_handle = dlopen(type_support_library_name.c_str(), RTLD_LAZY);
  if (!type_support_handle) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open type support library: %s", dlerror());
    dlclose(introspection_handle);
    return { nullptr, nullptr, nullptr, nullptr };
  }

  std::string type_support_symbol_name = "rosidl_typesupport_" + type_support_impl + "__get_message_type_support_handle__" +
                                         package_name + "__" + interface_type + "__" + message_name;

  auto get_type_support_ts = (const rosidl_message_type_support_t*(*)())dlsym(type_support_handle, type_support_symbol_name.c_str());
  const char* type_support_dlsym_error = dlerror();
  if (type_support_dlsym_error) {
    RCLCPP_ERROR(this->get_logger(), "Cannot load type support symbol: %s", type_support_dlsym_error);
    dlclose(introspection_handle);
    dlclose(type_support_handle);
    return { nullptr, nullptr, nullptr, nullptr };
  }

  const rosidl_message_type_support_t* ts = get_type_support_ts();

  if (!ts || !introspection_ts) {
    RCLCPP_ERROR(this->get_logger(), "Failed to obtain type support handles.");
    return { nullptr, nullptr, nullptr, nullptr };
  }

  const rosidl_typesupport_introspection_cpp::MessageMembers* members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_ts->data);

  m_type_support_info.members = members;
  m_type_support_info.type_support_handle = ts;
  m_type_support_info.introspection_library_handle = introspection_handle;
  m_type_support_info.type_support_library_handle = type_support_handle;

  return m_type_support_info;
}
