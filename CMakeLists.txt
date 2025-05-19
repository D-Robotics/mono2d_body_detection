# Copyright (c) 2024，D-Robotics.
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
find_package(cv_bridge REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(rclcpp_components REQUIRED)


find_package(hbm_img_msgs REQUIRED)
add_definitions(-DSHARED_MEM_ENABLED)

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DBPU_LIBDNN)
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH x3)
elseif(PLATFORM_Rdkultra)
  message("build ${PROJECT_NAME} for platform Rdkultra")
  add_definitions(-DBPU_LIBDNN)
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH rdkultra)
elseif(PLATFORM_X5)
  message("build ${PROJECT_NAME} for platform X5")
  add_definitions(-DBPU_LIBDNN)
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH x5)
elseif(PLATFORM_S100)
  message("build ${PROJECT_NAME} for platform S100")
  set(SYS_ROOT ${CMAKE_SYSROOT})
  set(PREFIX_PATH s100)
elseif(PLATFORM_X86)
  message("build platform X86")
  message("THIRD_PARTY is " ${THIRD_PARTY})
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x3)
  set(SYS_ROOT ${THIRD_PARTY})
  link_directories(${THIRD_PARTY}/usr/lib)
else()
  message("invalid platform, build platform X3 default")
  set(PREFIX_PATH x3)
  set(SYS_ROOT ${CMAKE_SYSROOT})
endif()

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/hobot/include/
  ${SYS_ROOT}/usr/include/hobot/
  ${SYS_ROOT}/usr/include/opencv/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/hobot/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
)

if (NOT PLATFORM_X86)
  find_package(hobot_mot REQUIRED)
endif()

add_library(${PROJECT_NAME}_component SHARED
  src/main.cpp
  src/mono2d_body_det_node.cpp
  src/post_process/yolo_pose_parser.cpp
)

target_compile_definitions(${PROJECT_NAME}_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
if (NOT PLATFORM_X86)
  ament_target_dependencies(
    ${PROJECT_NAME}_component
    hobot_mot
  )
endif()

ament_target_dependencies(
  ${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  dnn_node
  sensor_msgs
  ai_msgs
  cv_bridge
  hobot_cv
  hbm_img_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "Mono2dBodyDetNode"
  EXECUTABLE ${PROJECT_NAME}
)

include_directories(include
${PROJECT_SOURCE_DIR}
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}_component
    DESTINATION lib/
  )

install(DIRECTORY
${PROJECT_SOURCE_DIR}/config/${PREFIX_PATH}/
DESTINATION lib/${PROJECT_NAME}/config/
)

install(FILES
  ${PROJECT_SOURCE_DIR}/config/iou2_euclid_method_param.json
  DESTINATION lib/${PROJECT_NAME}/config/
)
install(FILES
  ${PROJECT_SOURCE_DIR}/config/iou2_method_param.json
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(FILES
  ${PROJECT_SOURCE_DIR}/config/person_body.jpg
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
