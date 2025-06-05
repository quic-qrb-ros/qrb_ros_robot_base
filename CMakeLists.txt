cmake_minimum_required(VERSION 3.5)
project(qrb_robot_base_manager VERSION 1.2.3)

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

find_package(ament_cmake REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/robot_base_manager.cpp
  src/qrc_utils.cpp
  src/watchdog.cpp
  src/parameter_manager.cpp
  src/time_manager.cpp
  src/sensor_manager.cpp
  src/client_manager.cpp
  src/motion_manager.cpp
  src/odom_manager.cpp
  src/emergency_manager.cpp
  src/charger_manager.cpp
  src/error.cpp
)

target_link_libraries(${PROJECT_NAME} qrc)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

if(ENABLE_TEST)
  add_definitions(-DENABLE_TEST)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
