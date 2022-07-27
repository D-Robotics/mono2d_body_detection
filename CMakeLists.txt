# Copyright (c) 2022，Horizon Robotics.
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

cmake_minimum_required(VERSION 3.5)
project(mono2d_body_detection)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_mot REQUIRED)

option(CV_BRIDGE_PKG "using cv_bridge pkg" OFF)
if (${CV_BRIDGE_PKG})
  message("using cv_bridge pkg")
  find_package(cv_bridge REQUIRED)
  add_definitions(-DCV_BRIDGE_PKG_ENABLED)
else()
  message("not using cv_bridge pkg")
endif ()

# BUILD_HBMEM is set in aarch64_toolchainfile.cmake
if (${BUILD_HBMEM})
  message("using shared mem")
  find_package(hbm_img_msgs REQUIRED)
  add_definitions(-DSHARED_MEM_ENABLED)
else()
  message("not using shared mem")
endif ()

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  ${CMAKE_SYSROOT}/usr/include/
  ${CMAKE_SYSROOT}/usr/include/opencv/
)

if(MOT_LIB_INSTALL_PATH)
  message("MOT_LIB_INSTALL_PATH is defined as " ${MOT_LIB_INSTALL_PATH})
else()
  set(MOT_LIB_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
  message("set MOT_LIB_INSTALL_PATH as " ${MOT_LIB_INSTALL_PATH})
endif()

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  ${MOT_LIB_INSTALL_PATH}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mono2d_body_det_node.cpp
  src/image_utils.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  hobot_mot
)

if (${BUILD_HBMEM})
  target_link_libraries(${PROJECT_NAME}
    hbmem
  )
  ament_target_dependencies(
    ${PROJECT_NAME}
    hbm_img_msgs
  )
endif ()

include_directories(include
${PROJECT_SOURCE_DIR}
)

if (${CV_BRIDGE_PKG})
ament_target_dependencies(
  ${PROJECT_NAME}
  cv_bridge
)
endif()

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
