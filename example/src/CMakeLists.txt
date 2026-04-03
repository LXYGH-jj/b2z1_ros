cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 (targets below use cxx_std_20)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/common include/nlohmann)
link_directories(src)

set(Z1_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../z1_sdk")
set(Z1_SDK_AVAILABLE FALSE)
if(EXISTS "${Z1_SDK_ROOT}/include" AND EXISTS "${Z1_SDK_ROOT}/lib")
  set(Z1_SDK_AVAILABLE TRUE)
  include_directories("${Z1_SDK_ROOT}/include")
  link_directories("${Z1_SDK_ROOT}/lib")
  message(STATUS "Found z1_sdk at ${Z1_SDK_ROOT}")
else()
  message(WARNING "z1_sdk not found at ${Z1_SDK_ROOT}, b2z1_stand_example will be built without Z1 control")
endif()

set(
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(b2_stand_example src/b2/b2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2_stand_example PRIVATE cxx_std_20)

add_executable(b2z1_stand_example src/b2/b2z1_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2z1_stand_example PRIVATE cxx_std_20)

add_executable(b2_sport_client src/b2/b2_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2_sport_client PRIVATE cxx_std_20)

ament_target_dependencies(b2_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(b2z1_stand_example ${DEPENDENCY_LIST})
ament_target_dependencies(b2_sport_client ${DEPENDENCY_LIST})

if(Z1_SDK_AVAILABLE)
  target_compile_definitions(b2z1_stand_example PRIVATE WITH_Z1_SDK=1)
  target_link_libraries(b2z1_stand_example Z1_SDK_${CMAKE_SYSTEM_PROCESSOR})
endif()

install(TARGETS
  b2_sport_client
  b2_stand_example
  b2z1_stand_example
  DESTINATION
)

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
