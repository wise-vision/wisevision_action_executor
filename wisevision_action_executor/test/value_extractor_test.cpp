// Copyright (c) 2024, WiseVision. All rights reserved.
#include <gtest/gtest.h>
#include <lora_msgs/msg/micro_publisher.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <std_msgs/msg/int32.hpp>

#include "value_extractor.hpp"
class ExtractValueTest : public ::testing::Test {

protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  template <typename T>
  rclcpp::SerializedMessage serializeMessage(const T& msg) {
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, &serialized_msg);
    return serialized_msg;
  }
};

void debugSerialization(const lora_msgs::msg::MicroPublisher& msg) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<lora_msgs::msg::MicroPublisher> serializer;
  serializer.serialize_message(&msg, &serialized_msg);

  const uint8_t* buffer = serialized_msg.get_rcl_serialized_message().buffer;
  std::cout << "Serialized buffer: ";
  for (size_t i = 0; i < serialized_msg.size(); ++i) {
    std::cout << std::hex << static_cast<int>(buffer[i]) << " ";
  }
  std::cout << std::dec << std::endl;
}

TEST_F(ExtractValueTest, ExtractInt32Value) {
  std_msgs::msg::Int32 msg;
  msg.data = 123;
  rclcpp::SerializedMessage serialized_msg = serializeMessage(msg);
  ValueExtractor service;

  auto introspection_ts = service.getIntrospectionTypeSupport("std_msgs/msg/Int32");
  ASSERT_NE(introspection_ts.members, nullptr);

  auto result = service.extractValue(serialized_msg, introspection_ts, "data");

  ASSERT_TRUE(result.has_value());

  try {
    auto extracted_value = std::any_cast<int32_t>(result.value());
    EXPECT_EQ(extracted_value, 123);
  } catch (const std::bad_any_cast& e) {
    FAIL() << "Bad any cast: " << e.what();
  }
}

TEST_F(ExtractValueTest, ExtractMapMetaDataValue) {
  nav_msgs::msg::MapMetaData msg;
  msg.origin.position.x = 45.67;

  rclcpp::SerializedMessage serialized_msg = serializeMessage(msg);
  ValueExtractor service;

  auto introspection_ts = service.getIntrospectionTypeSupport("nav_msgs/msg/MapMetaData");
  ASSERT_NE(introspection_ts.members, nullptr);

  auto result = service.extractValue(serialized_msg, introspection_ts, "origin.position.x");

  ASSERT_TRUE(result.has_value());

  try {
    auto extracted_value = std::any_cast<double>(result.value());
    EXPECT_DOUBLE_EQ(extracted_value, 45.67);
  } catch (const std::bad_any_cast& e) {
    FAIL() << "Bad any cast: " << e.what();
  }
}

TEST_F(ExtractValueTest, ExtractSensorValueById) {
  lora_msgs::msg::MicroPublisher msg;

  lora_msgs::msg::E5GenericMessage sensor1;
  sensor1.id = 101;
  sensor1.values.push_back(75.0f);

  lora_msgs::msg::E5GenericMessage sensor2;
  sensor2.id = 102;

  lora_msgs::msg::E5GenericMessage sensor3;
  sensor3.id = 103;

  msg.sensors_generic_data.push_back(sensor1);
  msg.sensors_generic_data.push_back(sensor2);
  msg.sensors_generic_data.push_back(sensor3);
  debugSerialization(msg);

  rclcpp::SerializedMessage serialized_msg = serializeMessage(msg);

  ValueExtractor service;

  auto introspection_ts = service.getIntrospectionTypeSupport("lora_msgs/msg/MicroPublisher");
  ASSERT_NE(introspection_ts.members, nullptr);

  auto result = service.extractValue(serialized_msg, introspection_ts, "sensors_generic_data[id=101].values[0]");
  ASSERT_TRUE(result.has_value());
  try {
    auto extracted_value = std::any_cast<float>(result.value());
    std::cout << "Extracted value: " << extracted_value << std::endl;
    EXPECT_EQ(extracted_value, 75);
  } catch (const std::bad_any_cast& e) {
    FAIL() << "Bad any cast: " << e.what();
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
