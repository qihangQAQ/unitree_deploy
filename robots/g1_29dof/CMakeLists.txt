cmake_minimum_required(VERSION 3.16)
project(g1_29dof_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(UNITREE_DEPLOY_WITH_ROS2 "Build the ROS2 RealSense depth source" OFF)
option(UNITREE_DEPLOY_BUILD_TESTS "Build deployment unit tests" ON)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to an ONNX Runtime installation")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

if(NOT ONNXRUNTIME_ROOT)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
    set(ONNXRUNTIME_ROOT
      "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
  else()
    message(FATAL_ERROR
      "ONNXRUNTIME_ROOT must point to an ${CMAKE_SYSTEM_PROCESSOR} ONNX Runtime installation")
  endif()
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include"
  NO_DEFAULT_PATH)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime libonnxruntime.so.1
  HINTS "${ONNXRUNTIME_ROOT}/lib"
  NO_DEFAULT_PATH)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "Could not find ONNX Runtime under ONNXRUNTIME_ROOT='${ONNXRUNTIME_ROOT}'")
endif()
message(STATUS "ONNX Runtime root: ${ONNXRUNTIME_ROOT}")

file(GLOB_RECURSE G1_CONTROLLER_SOURCES CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/src/*.cpp")

if(UNITREE_DEPLOY_WITH_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
else()
  list(REMOVE_ITEM G1_CONTROLLER_SOURCES
    "${PROJECT_SOURCE_DIR}/src/Ros2DepthSource.cpp")
endif()

add_library(${PROJECT_NAME}_lib STATIC ${G1_CONTROLLER_SOURCES})
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    /usr/include/eigen3
    /usr/local/include/ddscxx
    /usr/local/include/iceoryx/v2.0.2
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/../../include"
    "${ONNXRUNTIME_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    unitree_sdk2
    ddsc
    ddscxx
    rt
    Threads::Threads
    Boost::program_options
    yaml-cpp
    fmt
    "${ONNXRUNTIME_LIBRARY}")

if(UNITREE_DEPLOY_WITH_ROS2)
  target_compile_definitions(${PROJECT_NAME}_lib PUBLIC UNITREE_DEPLOY_WITH_ROS2=1)
  target_link_libraries(${PROJECT_NAME}_lib
    PUBLIC
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs_library)
endif()

add_executable(g1_ctrl main.cpp)
target_link_libraries(g1_ctrl PRIVATE ${PROJECT_NAME}_lib)

if(UNITREE_DEPLOY_WITH_ROS2)
  add_executable(depth_capture tools/depth_capture.cpp)
  target_link_libraries(depth_capture PRIVATE ${PROJECT_NAME}_lib)
endif()

if(UNITREE_DEPLOY_BUILD_TESTS)
  enable_testing()
  add_executable(depth_preprocessor_test tests/depth_preprocessor_test.cpp)
  target_include_directories(depth_preprocessor_test PRIVATE
    "${PROJECT_SOURCE_DIR}/../../include")
  target_link_libraries(depth_preprocessor_test PRIVATE yaml-cpp)
  add_test(NAME depth_preprocessor_test COMMAND depth_preprocessor_test)

  add_executable(ort_runner_test tests/ort_runner_test.cpp)
  target_link_libraries(ort_runner_test PRIVATE ${PROJECT_NAME}_lib)
  target_compile_definitions(ort_runner_test PRIVATE
    TEST_POLICY_PATH="${PROJECT_SOURCE_DIR}/config/policy/velocity/v0/exported/policy.onnx"
    TEST_DEPLOY_PATH="${PROJECT_SOURCE_DIR}/config/policy/velocity/v0/params/deploy.yaml")
  add_test(NAME ort_runner_test COMMAND ort_runner_test)
endif()
