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
find_package(cv_bridge REQUIRED)

option(SHARED_MEM "using shared mem" ON)
if (${SHARED_MEM})
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

message("MOT_LIB_INSTALL_PATH is " ${MOT_LIB_INSTALL_PATH})

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
  cv_bridge
)

if (${SHARED_MEM})
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
