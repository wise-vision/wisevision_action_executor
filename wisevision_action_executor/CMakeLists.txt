cmake_minimum_required(VERSION 3.8)
project(wisevision_action_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
${YAML_CPP_INCLUDEDIR}
include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lora_msgs REQUIRED)
find_package(notification_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(automatic_action_service_lib
  src/yaml_config_manager.cpp
  src/value_extractor.cpp
  src/automatic_action_service.cpp
)
target_link_libraries(automatic_action_service_lib ${YAML_CPP_LIBRARIES})

add_executable(talker 
  src/talker.cpp
)

add_executable(automatic_action_service 
  src/main.cpp
)

ament_target_dependencies(talker 
  rclcpp 
  std_msgs
)

ament_target_dependencies(automatic_action_service_lib 
  rclcpp std_msgs 
  lora_msgs
  notification_msgs   
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp 
  yaml-cpp
)

target_link_libraries(automatic_action_service automatic_action_service_lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(lora_msgs REQUIRED)
  ament_add_gtest(value_extractor_test test/value_extractor_test.cpp test/yaml_config_manager_test.cpp test/automatic_action_service_test.cpp)
  target_include_directories(value_extractor_test PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(value_extractor_test automatic_action_service_lib)
  ament_target_dependencies(value_extractor_test 
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  lora_msgs
  rosidl_typesupport_introspection_cpp
)
endif()

install(TARGETS
  talker
  automatic_action_service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
