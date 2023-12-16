#
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_compile_options(-Wall -Wextra -Wpedantic -Werror)
# find dependencies
# find_package(MetavisionSDK COMPONENTS driver REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)

set(ROS2_DEPENDENCIES
  "event_camera_msgs")

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})

#
# --------- encoding/decoding library
#
add_library(codec  src/encoder.cpp)
target_include_directories(codec
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(codec ${ROS2_DEPENDENCIES})
ament_export_targets(codecTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS2_DEPENDENCIES})

#
# ---- performance test tool
#
add_executable(codec_perf src/perf.cpp)
target_link_libraries(codec_perf codec)

#
# ---- metavision test code
#
#ament_auto_add_executable(metavision_test src/metavision_test.cpp)
#target_link_libraries(metavision_test MetavisionSDK::driver)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS codec
  EXPORT codecTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  codec_perf
#  metavision_test
  DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(LANGUAGE c++)
  ament_cpplint(FILTERS "-build/include,-runtime/indentation_namespace")
  ament_clang_format(CONFIG_FILE .clang-format)
  ament_flake8()
  ament_lint_cmake()
  ament_pep257()
  ament_xmllint()
endif()

if(EVENT_CAMERA_CODECS_BUILD_TESTS)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosbag2_cpp REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_decoder_test test/decoder_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    target_include_directories(${PROJECT_NAME}_decoder_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_decoder_test rclcpp rosbag2_cpp)
  target_link_libraries(${PROJECT_NAME}_decoder_test codec)
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(${ROS2_DEPENDENCIES})
ament_export_dependencies(class_loader)

ament_export_include_directories(include)
ament_export_libraries(codec)

ament_package()
