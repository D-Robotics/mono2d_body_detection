# 功能介绍

mono2d_body_detection package是使用hobot_dnn package开发的单目rgb人体检测算法示例，在地平线X3开发板上使用模型和图像数据利用BPU处理器进行模型推理。
检测模型为fasterRcnn，模型输出包含人体、人头、人脸、人手框和人体关键点检测结果。

示例订阅图片数据image msg，发布自定义的感知结果hobot ai msg，用户可以订阅发布的ai msg用于应用开发。

# 编译

## 依赖库


ros package：

- dnn_node
- ai_msgs
- hbm_img_msgs
- hobot_mot

dnn_node是在地平线X3开发板上利用BPU处理器进行模型推理的pkg，定义在hobot_dnn中。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

hbm_img_msgs为自定义的图片消息格式，用于shared mem场景下的图片传输，hbm_img_msgs pkg定义在hobot_msgs中。

hobot_mot是多目标跟踪（MOT）package，用于检测框的跟踪、ID分配。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### 编译选项

1. BUILD_HBMEM
   - 零拷贝传输方式使能开关。Docker交叉编译时默认打开(ON), 编译时可以通过-DBUILD_HBMEM=OFF关闭。
   - 在板端编译时，零拷贝传输方式使能开关默认是关闭的。如果需要依赖零拷贝，可以通过-DBUILD_HBMEM=ON打开。
   - 如果打开，编译会依赖hbm_img_msgs package，并且需要使用tros进行编译。
   - 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
   - 对于零拷贝通信方式，当前只支持订阅nv12格式图片。

### Ubuntu板端编译X3版本

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select mono2d_body_detection --cmake-args -DBUILD_HBMEM=ON`

### Docker交叉编译X3版本

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

   ```
   export TARGET_ARCH=aarch64
   export TARGET_TRIPLE=aarch64-linux-gnu
   export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

   colcon build --packages-select mono2d_body_detection \
      --merge-install \
      --cmake-force-configure \
      --cmake-args \
      --no-warn-unused-cli \
      -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
   ```

### X86 Ubuntu系统上编译 X86版本

1. 编译环境确认

   - x86 ubuntu版本: ubuntu20.04

2. 编译

   - 编译命令：

   ```
   colcon build --packages-select mono2d_body_detection  \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DBUILD_HBMEM=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## 注意事项

1. 板端使用launch启动，需要安装依赖，安装命令：`pip3 install lark-parser`。设备上只需要配置一次，断电重启不需要重新配置。

2. 第一次运行web展示需要启动webserver服务，运行方法为:

- cd 到websocket的部署路径下：`cd install/lib/websocket/webservice/`（如果是板端编译（无--merge-install编译选项）执行命令为`cd install/websocket/lib/websocket/webservice`）
- 启动nginx：`chmod +x ./sbin/nginx && ./sbin/nginx -p .`

# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名                | 类型        | 解释                                                                                                                                  | 是否必须 | 支持的配置           | 默认值                                               |
| --------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------- | -------- | -------------------- | ---------------------------------------------------- |
| is_sync_mode          | int         | 同步/异步推理模式。0：异步模式；1：同步模式                                                                                           | 否       | 0/1                  | 0                                                    |
| model_file_name       | std::string | 推理使用的模型文件                                                                                                                    | 否       | 根据实际模型路径配置 | config/multitask_body_head_face_hand_kps_960x544.hbm |
| is_shared_mem_sub     | int         | 是否使用shared mem通信方式订阅图片消息。0：关闭；1：打开。打开和关闭shared mem通信方式订阅图片的topic名分别为/hbmem_img和/image_raw。 | 否       | 0/1                  | 1                                                    |
| ai_msg_pub_topic_name | std::string | 发布包含人体、人头、人脸、人手框和人体关键点感知结果的AI消息的topic名                                                                 | 否       | 根据实际部署环境配置 | /hobot_mono2d_body_detection                         |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **X3 Ubuntu**

运行方式1，使用ros2 run启动：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .

# 启动图片发布pkg
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动web展示pkg
ros2 run websocket websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection --log-level error &

# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
ros2 run mono2d_body_detection mono2d_body_detection

```
运行方式2，使用launch文件启动：
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono2d_body_detection/config/ .

# 启动launch文件
ros2 launch install/share/mono2d_body_detection/launch/hobot_mono2d_body_detection.launch.py

```

### **X3 Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .

# 启动图片发布pkg
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=960 -p image_height:=544 -p io_method:=shared_mem --log-level error &
# 启动jpeg图片编码&发布pkg
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &
# 启动web展示pkg
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/hobot_mono2d_body_detection --log-level error &

# 启动单目rgb人体、人头、人脸、人手框和人体关键点检测pkg
./install/lib/mono2d_body_detection/mono2d_body_detection

```

### **X86 Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono2d_body_detection/config/ .

# 启动launch文件
export CAM_TYPE=usb
ros2 launch install/share/mono2d_body_detection/launch/hobot_mono2d_body_detection.launch.py

```

## 注意事项

1. 板端使用launch启动，需要安装依赖，安装命令：`pip3 install lark-parser`。设备上只需要配置一次，断电重启不需要重新配置。

2. 第一次运行web展示需要启动webserver服务，运行方法为:

- cd 到websocket的部署路径下：`cd install/lib/websocket/webservice/`（如果是板端编译（无--merge-install编译选项）执行命令为`cd install/websocket/lib/websocket/webservice`）
- 启动nginx：`chmod +x ./sbin/nginx && ./sbin/nginx -p .`

# 结果分析

## X3结果展示

```
[INFO] [1652174901.762272397] [mono2d_body_det]: Recved img encoding: nv12, h: 544, w: 960, step: 960, index: 237654, stamp: 1652174901_761011505, data size: 777600
[INFO] [1652174901.764373787] [mono2d_body_det]: Output from, frame_id: 237653, stamp: 1652174901_727517262
[INFO] [1652174901.764692229] [mono2d_body_det]: Output box type: body, rect size: 1
[INFO] [1652174901.764930425] [mono2d_body_det]: rect: 219.975 3.49968 838.203 445.801, 0.996765
[INFO] [1652174901.765215035] [mono2d_body_det]: Output box type: head, rect size: 0
[INFO] [1652174901.765380609] [mono2d_body_det]: Output box type: face, rect size: 0
[INFO] [1652174901.765565100] [mono2d_body_det]: Output box type: hand, rect size: 2
[INFO] [1652174901.765739674] [mono2d_body_det]: rect: 476.16 185.68 632.479 375.666, 0.995581
[INFO] [1652174901.765891749] [mono2d_body_det]: rect: 799.999 191.013 957.232 305.319, 0.921992
[WARN] [1652174901.767166849] [mono2d_body_det]: Publish frame_id: 237653, time_stamp: 1652174901_727517262
targets.size: 3
target track_id: 1, rois.size: 1, body
target track_id: 1, rois.size: 1, hand
target track_id: 2, rois.size: 1, hand
disappeared_targets.size: 0

[INFO] [1652174901.796131372] [mono2d_body_det]: Recved img encoding: nv12, h: 544, w: 960, step: 960, index: 237655, stamp: 1652174901_794324342, data size: 777600
[INFO] [1652174901.796131330] [mono2d_body_det]: Output from, frame_id: 237654, stamp: 1652174901_761011505
[INFO] [1652174901.798575535] [mono2d_body_det]: Output box type: body, rect size: 1
[INFO] [1652174901.798992305] [mono2d_body_det]: rect: 251.877 1.97849 815.793 444.43, 0.995042
[INFO] [1652174901.799290498] [mono2d_body_det]: Output box type: head, rect size: 0
[INFO] [1652174901.799514278] [mono2d_body_det]: Output box type: face, rect size: 1
[INFO] [1652174901.799806221] [mono2d_body_det]: rect: 449.343 1.86275 585.734 69.0991, 0.805632
[INFO] [1652174901.800046625] [mono2d_body_det]: Output box type: hand, rect size: 2
[INFO] [1652174901.800300778] [mono2d_body_det]: rect: 476.13 187.937 626.792 373.693, 0.992971
[INFO] [1652174901.800536391] [mono2d_body_det]: rect: 804.774 194.527 958.743 305.478, 0.892469
[WARN] [1652174901.802478664] [mono2d_body_det]: Publish frame_id: 237654, time_stamp: 1652174901_761011505
targets.size: 4
target track_id: 1, rois.size: 1, body
target track_id: 1, rois.size: 1, face
target track_id: 1, rois.size: 1, hand
target track_id: 2, rois.size: 1, hand
disappeared_targets.size: 0
```

以上log显示，订阅到了nv12 encoding格式的图片，发布包含人体、人头、人脸、人手框和人体关键点感知结果的ai msg。

## web效果展示

# 常见问题
