// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef PLATFORM_X86
#include "hobot_mot/hobot_mot.h"
#endif

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

#ifndef MONO2D_BODY_DET_NODE_H_
#define MONO2D_BODY_DET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::Model;
using hobot::dnn_node::ModelInferTask;
using hobot::dnn_node::ModelManager;
using hobot::dnn_node::ModelRoiInferTask;

using hobot::dnn_node::OutputParser;
using hobot::dnn_node::parser_fasterrcnn::FasterRcnnKpsParserPara;
using hobot::dnn_node::parser_fasterrcnn::LandmarksResult;
using ai_msgs::msg::PerceptionTargets;

// 使用output manage解决异步多线程情况下模型输出乱序的问题
class NodeOutputManage {
 public:
  void Feed(uint64_t ts_ms);
  std::vector<std::shared_ptr<DnnNodeOutput>> Feed(
      const std::shared_ptr<DnnNodeOutput>& node_output);
  void Erase(uint64_t ts_ms);

 private:
  std::set<uint64_t> cache_frame_;
  std::map<uint64_t, std::shared_ptr<DnnNodeOutput>> cache_node_output_;
  // 如果图像采集频率是30fps，能够缓存10帧，对应要求端到端的推理耗时最长不能超过1000/30*10=750ms
  const uint8_t cache_size_limit_ = 10;
  std::mutex mtx_;
  std::condition_variable cv_;
  const uint64_t smart_output_timeout_ms_ = 1000;
};

struct FasterRcnnOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
  struct timespec preprocess_timespec_start;
  struct timespec preprocess_timespec_end;
};

class Mono2dBodyDetNode : public DnnNode {
 public:
  Mono2dBodyDetNode(const std::string& node_name,
                    const NodeOptions& options = NodeOptions());
  ~Mono2dBodyDetNode() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput>& outputs) override;

 private:
  std::string model_file_name_ =
      "config/multitask_body_head_face_hand_kps_960x544.hbm";
  std::string model_name_ = "multitask_body_head_face_hand_kps_960x544";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;


  std::atomic<double> width_scale_{1.0};
  std::atomic<double> height_scale_{1.0};

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  const int32_t model_output_count_ = 9;
  const int32_t body_box_output_index_ = 1;
  const int32_t head_box_output_index_ = 3;
  const int32_t face_box_output_index_ = 5;
  const int32_t hand_box_output_index_ = 7;
  const std::vector<int32_t> box_outputs_index_ = {body_box_output_index_,
                                                   head_box_output_index_,
                                                   face_box_output_index_,
                                                   hand_box_output_index_};
  const int32_t kps_output_index_ = 8;
  std::unordered_map<int32_t, std::string> box_outputs_index_type_ = {
      {body_box_output_index_, "body"},
      {head_box_output_index_, "head"},
      {face_box_output_index_, "face"},
      {hand_box_output_index_, "hand"}};

  std::shared_ptr<FasterRcnnKpsParserPara> parser_para_ = nullptr;

  // key is mot processing type, body/face/head/hand
  // val is config file path
#ifndef PLATFORM_X86
  std::unordered_map<std::string, std::string> hobot_mot_configs_{
      {"body", "config/iou2_method_param.json"},
      {"face", "config/iou2_method_param.json"},
      {"head", "config/iou2_method_param.json"},
      {"hand", "config/iou2_euclid_method_param.json"}};

  // key is mot processing type, body/face/head/hand
  // val is mot instance
  std::unordered_map<std::string, std::shared_ptr<HobotMot>> hobot_mots_;
#endif
  
  int image_gap_ = 1;
 
  int is_sync_mode_ = 0;

  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 1;

  std::string ai_msg_pub_topic_name_ = "hobot_mono2d_body_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  int Predict(std::vector<std::shared_ptr<DNNInput>>& inputs,
              const std::shared_ptr<std::vector<hbDNNRoi>> rois,
              std::shared_ptr<DnnNodeOutput> dnn_output);

#ifdef SHARED_MEM_ENABLED
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::shared_ptr<NodeOutputManage> node_output_manage_ptr_ =
      std::make_shared<NodeOutputManage>();
#ifndef PLATFORM_X86
  int DoMot(
      const time_t& time_stamp,
      const std::unordered_map<int32_t, std::vector<MotBox>>& in_rois,
      std::unordered_map<int32_t, std::vector<MotBox>>& out_rois,
      std::unordered_map<int32_t, std::vector<std::shared_ptr<MotTrackId>>>&
          out_disappeared_ids);
#endif
};

#endif  // MONO2D_BODY_DET_NODE_H_
