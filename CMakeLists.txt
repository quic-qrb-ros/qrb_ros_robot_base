cmake_minimum_required(VERSION 3.5)
project(qrb_ros_robot_base VERSION 1.2.3)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_adapter REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(qrb_ros_robot_base_msgs REQUIRED)
find_package(qrb_robot_base_manager REQUIRED)

add_executable(robot_base
  src/robot_base.cpp
  src/robot_base/parameter_controller.cpp
  src/robot_base/sensor_controller.cpp
  src/robot_base/client_controller.cpp
  src/robot_base/motion_controller.cpp
  src/robot_base/odom_controller.cpp
  src/robot_base/emergency_controller.cpp
  src/robot_base/charger_controller.cpp
  src/robot_base/error_controller.cpp
)

ament_target_dependencies(robot_base
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  rosidl_adapter
  qrb_ros_robot_base_msgs
  qrb_robot_base_manager
)

set_target_properties(robot_base PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_base
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(ENABLE_TEST)
  add_definitions(-DENABLE_TEST)
endif()

if(ODOM_DATA_TEST)
  add_definitions(-DODOM_DATA_TEST)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
