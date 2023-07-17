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
find_package(cv_bridge REQUIRED)

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

if(PLATFORM_X3)
  message("build platform X3")
  set(SYS_ROOT ${CMAKE_SYSROOT})
elseif(PLATFORM_X5ultra)
  message("build ${PROJECT_NAME} for platform X5ultra")
  set(SYS_ROOT ${CMAKE_SYSROOT})
elseif(PLATFORM_X86)
  message("build platform X86")
  message("THIRD_PARTY is " ${THIRD_PARTY})
  add_definitions(-DPLATFORM_X86)
  set(SYS_ROOT ${THIRD_PARTY})
  link_directories(${THIRD_PARTY}/usr/lib)
else()
  message("invalid platform, build platform X3 default")
  set(PLATFORM_X3 ON)
  set(SYS_ROOT ${CMAKE_SYSROOT})
endif()

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/include/opencv/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
)

if (NOT PLATFORM_X86)
  find_package(hobot_mot REQUIRED)

  # 设置一个默认MOT库的安装路径
  set(MOT_LIB_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
  # 如果find_package成功，使用查找到的路径更新MOT_LIB_INSTALL_PATH
  if(hobot_mot_LIBRARIES)
    message("hobot_mot_LIBRARIES is ${hobot_mot_LIBRARIES}")
    # 查找库路径的最后一个/符号
    string(FIND ${hobot_mot_LIBRARIES} "/" index REVERSE)
    # 获取库路径
    string(SUBSTRING ${hobot_mot_LIBRARIES} 0 ${index} MOT_LIB_INSTALL_PATH)
    message("update MOT_LIB_INSTALL_PATH as " ${MOT_LIB_INSTALL_PATH})
  endif()
    link_directories(
      ${MOT_LIB_INSTALL_PATH}
    )
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mono2d_body_det_node.cpp
  src/image_utils.cpp
)

if (NOT PLATFORM_X86)
  ament_target_dependencies(
    ${PROJECT_NAME}
    hobot_mot
  )
  message("MOT_LIB_INSTALL_PATH is " ${MOT_LIB_INSTALL_PATH})
endif()

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  dnn_node
  sensor_msgs
  ai_msgs
  cv_bridge
)

if (${BUILD_HBMEM})
  ament_target_dependencies(
    ${PROJECT_NAME}
    hbm_img_msgs
  )
endif ()

include_directories(include
${PROJECT_SOURCE_DIR}
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(PLATFORM_X5ultra)
  install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/X5ultra/
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
else()
  install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
  )
endif()


install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
