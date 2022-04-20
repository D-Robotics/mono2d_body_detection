// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif

#include "sensor_msgs/msg/image.hpp"

#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#include "dnn_node/dnn_node.h"

#include "include/image_utils.h"

#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/capture_targets.hpp"

#ifndef MONO2D_BODY_DET_NODE_H_
#define MONO2D_BODY_DET_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::TaskId;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::NV12PyramidInput;

using hobot::dnn_node::Model;
using hobot::dnn_node::ModelInferTask;
using hobot::dnn_node::ModelManager;
using hobot::dnn_node::ModelRoiInferTask;

using hobot::dnn_node::OutputParser;

using ai_msgs::msg::PerceptionTargets;

struct FasterRcnnOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header = nullptr;
};

class Mono2dBodyDetNode : public DnnNode {
 public:
  Mono2dBodyDetNode(
  const std::string & node_name,
  const NodeOptions & options = NodeOptions());
  ~Mono2dBodyDetNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs)
    override;

 private:
  std::string model_file_name_ =
  "config/multitask_body_head_face_hand_kps_960x544.hbm";
  std::string model_name_ = "multitask_body_head_face_hand_kps_960x544";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  const int32_t model_output_count_ = 9;
  const int32_t body_box_output_index_ = 1;
  const int32_t head_box_output_index_ = 3;
  const int32_t face_box_output_index_ = 5;
  const int32_t hand_box_output_index_ = 7;
  const std::vector<int32_t> box_outputs_index_ = {
    body_box_output_index_,
    head_box_output_index_,
    face_box_output_index_,
    hand_box_output_index_
  };
  const int32_t kps_output_index_ = 8;
  std::unordered_map<int32_t, std::string> box_outputs_index_type_ = {
    {body_box_output_index_, "body"},
    {head_box_output_index_, "head"},
    {face_box_output_index_, "face"},
    {hand_box_output_index_, "hand"}
  };

  int is_sync_mode_ = 0;

  // 使用shared mem通信方式订阅图片
  int is_shared_mem_sub_ = 0;

  std::chrono::high_resolution_clock::time_point output_tp_;
  int output_frameCount_ = 0;
  std::mutex frame_stat_mtx_;

  std::string msg_pub_topic_name_ = "hobot_mono2d_body_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr
  msg_publisher_ = nullptr;

  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output);

#ifdef SHARED_MEM_ENABLED
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
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
};

#endif  // MONO2D_BODY_DET_NODE_H_
