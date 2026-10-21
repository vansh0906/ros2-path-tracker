cmake_minimum_required(VERSION 3.8)
project(path_tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# The controller core is header-only C++17 with no ROS dependency.
add_library(tracker_core INTERFACE)
target_include_directories(tracker_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(bench bench/bench.cpp)
target_link_libraries(bench PRIVATE tracker_core)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  # ---------------- ROS 2 build (colcon) ----------------
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  foreach(node path_publisher sim_robot tracker)
    add_executable(${node}_node src/${node}_node.cpp)
    # include dirs instead of linking tracker_core: never mixes with ament's target_link_libraries style
    target_include_directories(${node}_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(${node}_node rclcpp geometry_msgs nav_msgs std_msgs tf2_ros)
  endforeach()

  install(TARGETS path_publisher_node sim_robot_node tracker_node bench DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_core test/test_core.cpp)
    target_include_directories(test_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  ament_package()
else()
  # ---------------- plain CMake build (no ROS): benchmark + unit tests ----------------
  message(STATUS "ament_cmake not found: building the ROS-free core, benchmark and tests only")
  find_package(GTest QUIET)
  if(GTest_FOUND)
    enable_testing()
    add_executable(test_core test/test_core.cpp)
    target_link_libraries(test_core PRIVATE tracker_core GTest::gtest_main)
    add_test(NAME test_core COMMAND test_core)
  else()
    message(STATUS "GoogleTest not found: skipping unit tests (they run in Docker / CI)")
  endif()
endif()
