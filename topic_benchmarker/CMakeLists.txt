cmake_minimum_required(VERSION 3.8)
project(topic_benchmarker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  std_msgs
)

add_executable(benchmarkable_simple_subscription
  src/examples/benchmarkable_simple_subscription.cpp
  # src/topic_benchmarker.cpp
  )


ament_target_dependencies(benchmarkable_simple_subscription 
  ${dependencies})

install(TARGETS
  benchmarkable_simple_subscription
  DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(topic_benchmarker)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
