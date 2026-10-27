cmake_minimum_required(VERSION 3.16)
project(g1_29dof_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

option(UNITREE_DEPLOY_WITH_ROS2 "Build the ROS2 RealSense depth source" OFF)
option(UNITREE_DEPLOY_BUILD_TESTS "Build deployment unit tests" ON)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to an ONNX Runtime installation")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/ros2_enabled"
  CONTENT "${UNITREE_DEPLOY_WITH_ROS2}\n")

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(ONNXRUNTIME_ARCH x64)
  set(ONNXRUNTIME_ELF_MACHINE 3e00)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(ONNXRUNTIME_ARCH aarch64)
  set(ONNXRUNTIME_ELF_MACHINE b700)
else()
  message(FATAL_ERROR "Unsupported CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT
    "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-${ONNXRUNTIME_ARCH}-1.22.0")
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
file(READ "${ONNXRUNTIME_LIBRARY}" ONNXRUNTIME_ELF_HEADER LIMIT 6 HEX)
file(READ "${ONNXRUNTIME_LIBRARY}" ONNXRUNTIME_LIBRARY_MACHINE OFFSET 18 LIMIT 2 HEX)
string(TOLOWER "${ONNXRUNTIME_ELF_HEADER}" ONNXRUNTIME_ELF_HEADER)
string(TOLOWER "${ONNXRUNTIME_LIBRARY_MACHINE}" ONNXRUNTIME_LIBRARY_MACHINE)
if(NOT ONNXRUNTIME_ELF_HEADER STREQUAL "7f454c460201" OR
   NOT ONNXRUNTIME_LIBRARY_MACHINE STREQUAL ONNXRUNTIME_ELF_MACHINE)
  message(FATAL_ERROR
    "ONNX Runtime library '${ONNXRUNTIME_LIBRARY}' is not a Linux ${CMAKE_SYSTEM_PROCESSOR} ELF library")
endif()
message(STATUS "ONNX Runtime root: ${ONNXRUNTIME_ROOT}")

# SDK2 may be installed outside the linker's default search paths on the board.
# Resolve the actual library without requiring the SDK's CMake package.
find_library(UNITREE_SDK2_LIBRARY
  NAMES unitree_sdk2
  PATHS /opt/unitree_robotics/lib /opt/unitree_robotics/lib64
  DOC "Full path to the target machine's Unitree SDK2 library")
if(NOT UNITREE_SDK2_LIBRARY OR NOT EXISTS "${UNITREE_SDK2_LIBRARY}")
  message(FATAL_ERROR
    "Could not find the Unitree SDK2 library on the target machine. "
    "Searched standard library locations and /opt/unitree_robotics/lib{,64}. "
    "Set -DUNITREE_SDK2_LIBRARY=/absolute/path/to/libunitree_sdk2.a "
    "to the SDK library installed for the target architecture.")
endif()

# Pair the channel headers with the selected library. Searching the system
# headers independently can mix /usr/local headers with an /opt SDK library
# whose ChannelFactory::Init has a different signature.
# Support <prefix>/lib, <prefix>/lib64 and <prefix>/lib/<architecture> layouts.
get_filename_component(UNITREE_SDK2_LIBRARY_DIR "${UNITREE_SDK2_LIBRARY}" DIRECTORY)
get_filename_component(UNITREE_SDK2_PREFIX "${UNITREE_SDK2_LIBRARY_DIR}" DIRECTORY)
if(UNITREE_SDK2_LIBRARY_DIR MATCHES "/lib(64)?/[^/]+$")
  get_filename_component(UNITREE_SDK2_PREFIX "${UNITREE_SDK2_PREFIX}" DIRECTORY)
endif()
# Recompute this path on every configure so switching libraries cannot retain
# a cached include directory from a different SDK installation.
set(UNITREE_SDK2_INCLUDE_DIR "${UNITREE_SDK2_PREFIX}/include")
foreach(UNITREE_SDK2_HEADER channel_factory.hpp channel_publisher.hpp channel_subscriber.hpp)
  if(NOT EXISTS "${UNITREE_SDK2_INCLUDE_DIR}/unitree/robot/channel/${UNITREE_SDK2_HEADER}")
    message(FATAL_ERROR
      "Missing SDK2 header '${UNITREE_SDK2_INCLUDE_DIR}/unitree/robot/channel/${UNITREE_SDK2_HEADER}' "
      "for library '${UNITREE_SDK2_LIBRARY}'. "
      "Install the matching SDK2 headers under the same prefix, or set "
      "UNITREE_SDK2_LIBRARY to a complete SDK2 installation's library. "
      "Headers from another SDK installation will not be used as a fallback.")
  endif()
endforeach()

# Match the dependencies exported by the board's unitree_sdk2Targets.cmake:
# SDK/DDS C headers in include/, DDS C++ headers in include/ddscxx, and both
# DDS shared libraries beside libunitree_sdk2.a. Resolving only SDK2 is not
# sufficient: /usr/local's ddscxx headers can require a different DDS C API.
# Recompute every path from the selected SDK instead of caching DDS separately.
set(UNITREE_DDS_CXX_INCLUDE_DIR "${UNITREE_SDK2_INCLUDE_DIR}/ddscxx")
set(UNITREE_DDS_C_LIBRARY "${UNITREE_SDK2_LIBRARY_DIR}/libddsc.so")
set(UNITREE_DDS_CXX_LIBRARY "${UNITREE_SDK2_LIBRARY_DIR}/libddscxx.so")
foreach(UNITREE_DDS_FILE
    "${UNITREE_SDK2_INCLUDE_DIR}/dds/dds.h"
    "${UNITREE_SDK2_INCLUDE_DIR}/dds/features.h"
    "${UNITREE_DDS_CXX_INCLUDE_DIR}/dds/dds.hpp"
    "${UNITREE_DDS_C_LIBRARY}"
    "${UNITREE_DDS_CXX_LIBRARY}")
  if(NOT EXISTS "${UNITREE_DDS_FILE}")
    message(FATAL_ERROR
      "Missing SDK2 DDS dependency '${UNITREE_DDS_FILE}'. "
      "SDK2, DDS C and DDS C++ must come from one matching installation. "
      "Install the matching DDS headers/libraries for '${UNITREE_SDK2_LIBRARY}', "
      "or select a complete SDK installation with UNITREE_SDK2_LIBRARY. "
      "System or ROS DDS files will not be used as a fallback.")
  endif()
endforeach()
message(STATUS "Unitree SDK2 library: ${UNITREE_SDK2_LIBRARY}")
message(STATUS "Unitree SDK2 headers: ${UNITREE_SDK2_INCLUDE_DIR}")
message(STATUS "Unitree DDS C++ headers: ${UNITREE_DDS_CXX_INCLUDE_DIR}")
message(STATUS "Unitree DDS C library: ${UNITREE_DDS_C_LIBRARY}")
message(STATUS "Unitree DDS C++ library: ${UNITREE_DDS_CXX_LIBRARY}")

# Keep the HG message definitions together: older board SDK2 installations may
# provide channel/DDS support without the G1 LowCmd/LowState IDL headers.
set(UNITREE_HG_INCLUDE_DIR
  "${PROJECT_SOURCE_DIR}/../../thirdparty/unitree_sdk2_hg/include")
foreach(UNITREE_HG_HEADER LowCmd_ LowState_ MotorCmd_ MotorState_ IMUState_)
  if(NOT EXISTS "${UNITREE_HG_INCLUDE_DIR}/unitree/idl/hg/${UNITREE_HG_HEADER}.hpp")
    message(FATAL_ERROR
      "Missing bundled HG header ${UNITREE_HG_HEADER}.hpp. "
      "Sync the complete thirdparty/unitree_sdk2_hg directory to the board.")
  endif()
endforeach()
message(STATUS "Unitree HG headers: ${UNITREE_HG_INCLUDE_DIR}")

file(GLOB_RECURSE G1_CONTROLLER_SOURCES CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/src/*.cpp")
# Compile the ROS implementation separately so Foxy's Cyclone DDS headers do
# not enter translation units that use the SDK2 Cyclone DDS installation.
list(REMOVE_ITEM G1_CONTROLLER_SOURCES
  "${PROJECT_SOURCE_DIR}/src/Ros2DepthSource.cpp")

if(UNITREE_DEPLOY_WITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_library(g1_depth_ros2 STATIC
    "${PROJECT_SOURCE_DIR}/src/Ros2DepthSource.cpp")
  target_include_directories(g1_depth_ros2 PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/../../include")
  target_link_libraries(g1_depth_ros2 PUBLIC Threads::Threads fmt)
  ament_target_dependencies(g1_depth_ros2 PUBLIC rclcpp sensor_msgs)

  # Foxy's ament_target_dependencies has no PRIVATE mode. Retain its local
  # compile settings, but make its exported dependencies link-only, as for a
  # static library's PRIVATE dependencies. Ros2DepthSource.h exposes no ROS
  # types, so consumers need neither ROS include paths nor ROS definitions.
  get_target_property(G1_DEPTH_ROS2_LINK_LIBRARIES
    g1_depth_ros2 INTERFACE_LINK_LIBRARIES)
  set_target_properties(g1_depth_ros2 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ""
    INTERFACE_COMPILE_DEFINITIONS ""
    INTERFACE_LINK_LIBRARIES "$<LINK_ONLY:${G1_DEPTH_ROS2_LINK_LIBRARIES}>")
  message(STATUS "ROS2 depth backend: isolated from Unitree DDS headers")
endif()

add_library(${PROJECT_NAME}_lib STATIC ${G1_CONTROLLER_SOURCES})
# Keep the bundled wrappers/HG messages first, then the matching SDK/DDS headers
# ahead of the compiler's default /usr/local/include. PUBLIC also applies this
# order to g1_ctrl and other controller consumers, but not the ROS2 backend.
target_include_directories(${PROJECT_NAME}_lib BEFORE
  PUBLIC
    "${PROJECT_SOURCE_DIR}/../../thirdparty/unitree_dds_wrapper/include"
    "${UNITREE_HG_INCLUDE_DIR}"
    "${UNITREE_SDK2_INCLUDE_DIR}"
    "${UNITREE_DDS_CXX_INCLUDE_DIR}"
    /usr/include/eigen3
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/../../include"
    "${ONNXRUNTIME_INCLUDE_DIR}")
# Link all three files explicitly, with DDS C++ before its DDS C dependency.
# Bare -lddsc/-lddscxx could select another installation despite correct headers.
target_link_libraries(${PROJECT_NAME}_lib
  "${UNITREE_SDK2_LIBRARY}"
  "${UNITREE_DDS_CXX_LIBRARY}"
  "${UNITREE_DDS_C_LIBRARY}"
  rt
  Threads::Threads
  Boost::program_options
  yaml-cpp
  fmt
  "${ONNXRUNTIME_LIBRARY}")

if(UNITREE_DEPLOY_WITH_ROS2)
  target_compile_definitions(${PROJECT_NAME}_lib PUBLIC UNITREE_DEPLOY_WITH_ROS2=1)
  target_link_libraries(${PROJECT_NAME}_lib g1_depth_ros2)
endif()

add_executable(g1_ctrl main.cpp)
target_link_libraries(g1_ctrl PRIVATE ${PROJECT_NAME}_lib)
# CMake adds the DDS directory to the build RPATH from the explicit libraries.
# Retain that selected installation for the installed controller as well.
set_property(TARGET g1_ctrl APPEND PROPERTY INSTALL_RPATH "${UNITREE_SDK2_LIBRARY_DIR}")

if(UNITREE_DEPLOY_WITH_ROS2)
  add_executable(depth_capture tools/depth_capture.cpp)
  target_include_directories(depth_capture PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
    "${PROJECT_SOURCE_DIR}/../../include")
  target_link_libraries(depth_capture PRIVATE g1_depth_ros2 yaml-cpp)
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
    TEST_PROJECT_DIR="${PROJECT_SOURCE_DIR}"
    TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/config/config.yaml")
  add_test(NAME ort_runner_test COMMAND ort_runner_test)
endif()

install(TARGETS g1_ctrl RUNTIME DESTINATION bin)
if(UNITREE_DEPLOY_WITH_ROS2)
  install(TARGETS depth_capture RUNTIME DESTINATION bin)
  install(PROGRAMS "${PROJECT_SOURCE_DIR}/../../run_real_depth.sh" DESTINATION bin)
endif()
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/" DESTINATION config)
install(DIRECTORY "${ONNXRUNTIME_ROOT}/lib/" DESTINATION lib
  FILES_MATCHING PATTERN "*.so*")
install(PROGRAMS "${PROJECT_SOURCE_DIR}/../../run_real.sh" DESTINATION bin)
install(FILES
  "${PROJECT_SOURCE_DIR}/../../scripts/ros_env.sh"
  "${CMAKE_CURRENT_BINARY_DIR}/ros2_enabled"
  DESTINATION bin)
