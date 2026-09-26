cmake_minimum_required(VERSION 3.16)
project(yaro_check CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Werror)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)

# The robot descriptions live at the repository root (models/), three levels above this package.
get_filename_component(YARO_MODELS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../models" ABSOLUTE)

# Core library: plain C++ and Eigen, no ROS node code, so it is easy to unit test.
add_library(yaro_check_core
  src/kinematics.cpp
  src/limits.cpp
  src/model_audit.cpp
  src/datasheet.cpp)
target_include_directories(yaro_check_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(yaro_check_core PUBLIC Eigen3::Eigen urdf::urdf)

add_executable(joint_limit_monitor src/joint_limit_monitor_node.cpp)
target_link_libraries(joint_limit_monitor yaro_check_core rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${diagnostic_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(yaro_audit src/audit_cli.cpp)
target_link_libraries(yaro_audit yaro_check_core)

add_executable(yaro_fk src/fk_cli.cpp)
target_link_libraries(yaro_fk yaro_check_core)

add_executable(yaro_bench src/bench_cli.cpp)
target_link_libraries(yaro_bench yaro_check_core kdl_parser::kdl_parser orocos-kdl)

install(TARGETS yaro_check_core joint_limit_monitor yaro_audit yaro_fk yaro_bench
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  foreach(t kinematics limits audit datasheet)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} yaro_check_core)
    target_compile_definitions(test_${t} PRIVATE YARO_MODELS_DIR="${YARO_MODELS_DIR}")
  endforeach()

  ament_add_gtest(test_kdl_crosscheck test/test_kdl_crosscheck.cpp)
  target_link_libraries(test_kdl_crosscheck yaro_check_core kdl_parser::kdl_parser orocos-kdl)
  target_compile_definitions(test_kdl_crosscheck PRIVATE YARO_MODELS_DIR="${YARO_MODELS_DIR}")

  add_launch_test(test/test_monitor_launch.py
    TIMEOUT 120
    ENV YARO_MODELS_DIR=${YARO_MODELS_DIR})
endif()

ament_package()
