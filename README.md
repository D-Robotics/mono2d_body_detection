# mono2d_body_detection

## Intro

mono2d_body_detection package是使用hobot_dnn package开发的单目rgb人体检测算法示例，在地平线X3开发板上使用模型和图像数据利用BPU处理器进行模型推理。
检测模型为fasterRcnn，模型输出包含人体、人头、人脸、人手框和人体关键点检测结果。
图像数据来源于订阅到的image msg。
模型推理结果，即检测框和人体关键点感知结果使用自定义的hobot ai msg发布出去，默认发布topic名为“hobot_mono2d_body_detection”，支持启动时通过参数-p msg_pub_topic_name重新指定。用户可以订阅此topic的ai msg用于应用开发。

## Build

### Dependency

ros package：

- sensor_msgs
- cv_bridge
- dnn_node
- hbm_img_msgs
- ai_msgs

其中cv_bridge为ROS开源的package，需要手动安装，具体安装方法（仅限X3 Ubuntu系统上编译有效）：

```cpp
# 方法1，直接使用apt安装，以cv_bridge安装举例
sudo apt-get install ros-foxy-cv-bridge -y

# 方法2，使用rosdep检查并自动安装pkg编译的依赖项
# 安装ros pkg依赖下载⼯具rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
# 在ros的⼯程路径下执⾏安装依赖，需要指定pkg所在路径。默认为所有pkg安装依赖，也可以指定为某个pkg安装依赖
rosdep install -i --from-path . --rosdistro foxy -y
```

dnn_node是在地平线X3开发板上利用BPU处理器进行模型推理的pkg，定义在hobot_dnn中。

hbm_img_msgs为自定义的图片消息格式，用于shared mem场景下的图片传输，hbm_img_msgs pkg定义在hobot_msgs中，因此如果使用shared mem进行图片传输，需要下载hobot_msgs。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

### 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

### 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。

#### 编译选项

1、CV_BRIDGE_PKG

- cv_bridge pkg依赖的使能开关，默认关闭（OFF），编译时使用-DCV_BRIDGE_PKG=ON命令打开。
- 如果打开，编译和运行会依赖cv_bridge pkg，支持使用订阅到的rgb8和nv12格式图片进行模型推理。
- 如果关闭，编译和运行不依赖cv_bridge pkg，只支持使用订阅到的nv12格式图片进行模型推理。

2、BUILD_HBMEM

- shared mem（共享内存传输）使能开关，默认关闭（OFF），只有在docker中使用tros编译时才会打开（aarch64_toolchainfile.cmake中设置）。
- 如果打开，编译和运行会依赖hbm_img_msgs pkg，并且需要使用tros进行编译。
- 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
- 对于shared mem通信方式，当前只支持订阅nv12格式图片。

#### X3 Ubuntu系统上编译

1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已安装dnn node package
- 已安装cv_bridge package（安装方法见Dependency部分）
- 已安装ai_msgs

2、编译

- 编译命令：`colcon build --packages-select mono2d_body_detection --cmake-args -DCV_BRIDGE_PKG=ON`
- 编译和运行会依赖cv_bridge pkg，不使用shared mem通信方式。支持使用订阅到的rgb8和nv12格式图片进行模型推理。

#### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已安装dnn node package
- 已安装hbm_img_msgs package（编译方法见Dependency部分）
- 已安装ai_msgs

2、编译

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

- 使用默认的编译选项（关闭cv_bridge pkg依赖，打开了shared mem通信方式），只支持订阅nv12格式图片进行推理。


## Usage

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

## X3 Ubuntu系统上运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是docker中编译安装路径为install/lib/mono2d_body_detection/config/，拷贝命令为cp -r install/lib/mono2d_body_detection/config/ .。
# 也可以在启动指令中指定模型文件路径：-p model_file_name:=./multitask_body_head_face_hand_kps_960x544.hbm，如果不指定，默认模型文件路径为config/multitask_body_head_face_hand_kps_960x544.hbm
cp -r install/mono2d_body_detection/lib/mono2d_body_detection/config/ .

# 运行模式1：使用订阅到的ROS image msg(topic为/image_raw)进行预测，并设置log级别为warn
ros2 run mono2d_body_detection mono2d_body_detection --ros-args --log-level warn

# 运行模式2：使用shared mem通信方式订阅image msg(topic为/hbmem_img)进行预测，指定发布的ai msg topic为mono2d_body_det，并设置log级别为warn
ros2 run mono2d_body_detection mono2d_body_detection --ros-args -p is_shared_mem_sub:=1 -p msg_pub_topic_name:=mono2d_body_det --ros-args --log-level warn

```

## X3 yocto系统上运行

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，回灌使用的本地图片
cp -r install/lib/mono2d_body_detection/config/ .

# 运行模式1：使用订阅到的image msg(topic为/image_raw)通过异步模式进行预测，并设置log级别为warn
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args --log-level warn

# 运行模式2：使用shared mem通信方式(topic为/hbmem_img)通过异步模式进行预测，指定发布的ai msg topic为mono2d_body_det，并设置log级别为warn
./install/lib/mono2d_body_detection/mono2d_body_detection --ros-args -p is_shared_mem_sub:=1 -p msg_pub_topic_name:=mono2d_body_det --ros-args --log-level warn

```
