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

#include "rclcpp/rclcpp.hpp"
#ifdef CV_BRIDGE_PKG_ENABLED
#include "cv_bridge/cv_bridge.h"
#endif

#include "dnn_node/dnn_node.h"

#include "include/image_utils.h"
#include "include/image_subscriber.h"

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

// todo OutputParser should set in dnn_node
using hobot::easy_dnn::OutputParser;

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

  int Run();

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PreProcess(std::vector<std::shared_ptr<DNNInput>> &inputs,
                const TaskId& task_id,
                const std::shared_ptr<std::vector<hbDNNRoi>> rois = nullptr)
                override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs)
    override;

 private:
  std::string model_file_name_ = "config/multitask_body_kps_960x544.hbm";
  std::string model_name_ = "multitask_body_kps_960x544";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  const int model_input_width_ = 960;
  const int model_input_height_ = 544;
  const int32_t model_output_count_ = 3;
  // box output index is 1
  const int32_t box_output_index_ = 1;
  // kps output index is 2
  const int32_t kps_output_index_ = 2;

  int is_sync_mode_ = 0;

  std::shared_ptr<ImageSubscriber> image_subscriber_ = nullptr;

  std::chrono::high_resolution_clock::time_point output_tp_;
  int output_frameCount_ = 0;
  std::mutex frame_stat_mtx_;

  std::string msg_pub_topic_name = "hobot_mono2d_body_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr
  msg_publisher_;

  int Predict(std::vector<std::shared_ptr<DNNInput>> &inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output);
  int Feed();

#ifdef SHARED_MEM_ENABLED
  void SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr &msg);
#endif
};

#endif  // MONO2D_BODY_DET_NODE_H_
