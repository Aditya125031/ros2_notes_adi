cmake_minimum_required(VERSION 3.8)
project(num_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies rclcpp std_msgs)

# Build executable nodes
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher ${dependencies})

add_executable(number_subscriber src/number_subscriber.cpp)
ament_target_dependencies(number_subscriber ${dependencies})

# Install node executables
install(TARGETS
  number_publisher
  number_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Optional: Linting/Test support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
