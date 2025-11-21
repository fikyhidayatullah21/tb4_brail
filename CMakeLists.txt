cmake_minimum_required(VERSION 3.8)
project(tb4_brail)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME}_node
  src/pick_place_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  nav2_msgs
  irobot_create_msgs
)

install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

