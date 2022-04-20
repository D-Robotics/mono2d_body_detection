// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>
#include <fstream>
#include <vector>
#include <utility>
#include <unistd.h>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "include/mono2d_body_det_node.h"
#include "include/image_utils.h"
#include "util/image_proc.h"
#include "include/fasterrcnn_kps_output_parser.h"
#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif

Mono2dBodyDetNode::Mono2dBodyDetNode(
    const std::string & node_name,
    const NodeOptions & options) :
    DnnNode(node_name, options) {
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("msg_pub_topic_name",
  msg_pub_topic_name_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);

  std::stringstream ss;
  ss << "Parameter:"
  << "\n is_sync_mode_: " << is_sync_mode_
  << "\n model_file_name_: " << model_file_name_
  << "\n is_shared_mem_sub: " << is_shared_mem_sub_
  << "\n msg_pub_topic_name_: " << msg_pub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
      "Init failed!");
  }

  msg_publisher_ =
      this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        msg_pub_topic_name_, 10);

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
      "Get model input size fail!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
    "The model input width is %d and height is %d",
    model_input_width_, model_input_height_);
  }

  if (is_shared_mem_sub_) {
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
      "Create hbmem_subscription with topic_name: %s",
      sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            sharedmem_img_topic_name_, 10,
            std::bind(&Mono2dBodyDetNode::SharedMemImgProcess, this,
                      std::placeholders::_1));
#else
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
        "Unsupport shared mem");
#endif
  } else {
    RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
      "Create subscription with topic_name: %s", ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10,
        std::bind(&Mono2dBodyDetNode::RosImgProcess, this,
                  std::placeholders::_1));
  }
}

Mono2dBodyDetNode::~Mono2dBodyDetNode() {
}

int Mono2dBodyDetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int Mono2dBodyDetNode::SetOutputParser() {
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "Set output parser.");
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
      "Error! Model %s output count is %d, unmatch with count %d",
      dnn_node_para_ptr_->model_name.c_str(),
      model_manage->GetOutputCount(),
      model_output_count_);
    return -1;
  }

  // 1 set box paser
  // 使用easy dnn中定义的FaceHandDetectionOutputParser后处理进行更新
  std::shared_ptr<OutputParser> box_out_parser =
      std::make_shared<hobot::easy_dnn::FaceHandDetectionOutputParser>();
  for (auto parse_idx : box_outputs_index_) {
    model_manage->SetOutputParser(parse_idx, box_out_parser);
  }

  // 2 set kps paser
  // 2.1 set dependencies
  auto output_desc =
      std::make_shared<OutputDescription>(model_manage, kps_output_index_,
      "body_kps_branch");
  output_desc->GetDependencies().push_back(body_box_output_index_);
  output_desc->SetType("body_kps");
  model_manage->SetOutputDescription(output_desc);
  // 2.2 create kps parser para and set para with model info
  auto parser_para = std::make_shared<FasterRcnnKpsParserPara>();
  hbDNNTensorProperties tensor_properties;
  model_manage->GetOutputTensorProperties(tensor_properties,
    kps_output_index_);
  parser_para->aligned_kps_dim.clear();
  parser_para->kps_shifts_.clear();
  for (int i = 0; i < tensor_properties.alignedShape.numDimensions; i++) {
    parser_para->aligned_kps_dim.push_back(
      tensor_properties.alignedShape.dimensionSize[i]);
  }
  for (int i = 0; i < tensor_properties.shift.shiftLen; i++) {
    parser_para->kps_shifts_.push_back(static_cast<uint8_t>(
      tensor_properties.shift.shiftData[i]));
  }

  std::stringstream ss;
  ss << "aligned_kps_dim:";
  for (const auto& val : parser_para->aligned_kps_dim) {
    ss << " " << val;
  }
  ss << "\nkps_shifts: ";
  for (const auto& val : parser_para->kps_shifts_) {
    ss << " " << val;
  }
  ss << "\n";
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

  // 2.3 create and set kps parser
  std::shared_ptr<OutputParser> kps_out_parser =
      std::make_shared<FasterRcnnKpsOutputParser>(parser_para);
  model_manage->SetOutputParser(kps_output_index_, kps_out_parser);

  return 0;
}

int Mono2dBodyDetNode::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &node_output) {
  if (!msg_publisher_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
    "Invalid msg_publisher_");
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
    "outputs.size():%d", node_output->outputs.size());

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);
  ai_msgs::msg::Perf perf;
  perf.set__type("PostProcess");
  perf.stamp_start.sec = time_start.tv_sec;
  perf.stamp_start.nanosec = time_start.tv_nsec;

  auto fasterRcnn_output =
    std::dynamic_pointer_cast<FasterRcnnOutput>(node_output);
  if (fasterRcnn_output) {
    std::stringstream ss;
    ss << "Output from";
    if (fasterRcnn_output->image_msg_header) {
      ss << ", frame_id: " << fasterRcnn_output->image_msg_header->frame_id
         << ", stamp: " << fasterRcnn_output->image_msg_header->stamp.sec
         << "." << fasterRcnn_output->image_msg_header->stamp.nanosec;
    }
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
  }

  const auto& outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
    "outputs size: %d", outputs.size());
  if (outputs.empty() ||
    static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Invalid outputs");
    return -1;
  }

  int smart_fps = 0;
  {
    auto tp_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - output_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
        "Smart fps = %d", output_frameCount_);
      smart_fps = output_frameCount_;
      output_frameCount_ = 0;
      output_tp_ = std::chrono::system_clock::now();
    }
  }

  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  if (fasterRcnn_output->image_msg_header) {
    pub_data->header.set__stamp(fasterRcnn_output->image_msg_header->stamp);
    pub_data->header.set__frame_id(
      fasterRcnn_output->image_msg_header->frame_id);
  }

  pub_data->set__fps(smart_fps);

  std::unordered_map<int32_t,
  std::vector<ai_msgs::msg::Roi>> rois;
  std::vector<ai_msgs::msg::Point> body_kps;

  for (const auto& idx : box_outputs_index_) {
    auto filter2d_result =
              dynamic_cast<Filter2DResult *>(
                outputs[idx].get());
    if (!filter2d_result) {
      continue;
    }

    if (box_outputs_index_type_.find(idx) == box_outputs_index_type_.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
        "Invalid output index: %d", idx);
      return -1;
    }
    std::string roi_type = box_outputs_index_type_[idx];

    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
      "Output box type: %s, rect size: %d",
      roi_type.data(), filter2d_result->boxes.size());

    for (auto &rect : filter2d_result->boxes) {
      if (rect.left < 0) rect.left = 0;
      if (rect.top < 0) rect.top = 0;
      if (rect.right > model_input_width_) {
        rect.right = model_input_width_;
      }
      if (rect.bottom > model_input_height_) {
        rect.bottom = model_input_height_;
      }
      std::stringstream ss;
      ss << "rect: " << rect.left << " " << rect.top
          << " " << rect.right << " " << rect.bottom;
      RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
      "%s", ss.str().c_str());

      ai_msgs::msg::Roi roi;
      roi.type = roi_type;
      roi.rect.set__x_offset(rect.left);
      roi.rect.set__y_offset(rect.top);
      roi.rect.set__width(rect.right - rect.left);
      roi.rect.set__height(rect.bottom - rect.top);
      rois[idx].emplace_back(roi);
    }
  }

  auto lmk_result = dynamic_cast<LandmarksResult *>(
    outputs[kps_output_index_].get());
  if (lmk_result) {
    std::stringstream ss;
    for (const auto& value : lmk_result->values) {
      ai_msgs::msg::Point target_point;
      target_point.set__type("body_kps");
      ss << "kps point: ";
      for (const auto &lmk : value) {
        ss << "\n" << lmk.x << "," << lmk.y << "," << lmk.score;
        geometry_msgs::msg::Point32 pt;
        pt.set__x(lmk.x);
        pt.set__y(lmk.y);
        target_point.point.emplace_back(pt);
      }
      ss << "\n";
      RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
        "FasterRcnnKpsOutputParser parse kps: %s", ss.str().c_str());
      body_kps.emplace_back(target_point);
    }
  }

  for (const auto& roi : rois) {
    for (size_t idx = 0; idx < roi.second.size(); idx++) {
      ai_msgs::msg::Target target;
      target.set__type("person");
      target.rois.emplace_back(roi.second.at(idx));
      if (roi.first == body_box_output_index_ &&
      roi.second.size() == body_kps.size()) {
        target.points.emplace_back(body_kps.at(idx));
      }
      pub_data->targets.emplace_back(std::move(target));
    }
  }

  clock_gettime(CLOCK_REALTIME, &time_start);
  perf.stamp_end.sec = time_start.tv_sec;
  perf.stamp_end.nanosec = time_start.tv_nsec;
  pub_data->perfs.emplace_back(perf);

  if (!pub_data->targets.empty()) {
    for (const auto target : pub_data->targets) {
      RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
      "Publish target.rois.size: %d", target.rois.size());
      for (const auto& roi : target.rois) {
        RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
        "roi.type: %s", roi.type.c_str());
      }
    }
  }

  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int Mono2dBodyDetNode::Predict(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois,
  std::shared_ptr<DnnNodeOutput> dnn_output) {
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "task_num: %d",
  dnn_node_para_ptr_->task_num);
  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

void Mono2dBodyDetNode::RosImgProcess(
  const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
  << ", h: " << img_msg->height
  << ", w: " << img_msg->width
  << ", step: " << img_msg->step
  << ", frame_id: " << img_msg->header.frame_id
  << ", stamp: " << img_msg->header.stamp.sec
  << "." << img_msg->header.stamp.nanosec
  << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

  // dump recved img msg
  // std::ofstream ofs("img." + img_msg->encoding);
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data.size());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img = cv_bridge::cvtColorForDisplay(
      cv_bridge::toCvShare(img_msg),
      "bgr8");
    // dump recved img msg after convert
    // cv::imwrite("dump_raw_" +
    //     std::to_string(img_msg->header.stamp.sec) + "." +
    //     std::to_string(img_msg->header.stamp.nanosec) + ".jpg",
    //     cv_img->image);

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start).count();
      RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
        "after cvtColorForDisplay cost ms: %d", interval);
    }

    pyramid = ImageUtils::GetNV12Pyramid(cv_img->image,
      model_input_height_, model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()),
      img_msg->height, img_msg->width,
      model_input_height_, model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
      "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
      "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Run predict failed!");
    return;
  }
}

#ifdef SHARED_MEM_ENABLED
void Mono2dBodyDetNode::SharedMemImgProcess(
  const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg) {
    return;
  }

  if (!rclcpp::ok()) {
    return;
  }

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
  std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(img_msg->data.data()),
      img_msg->height, img_msg->width,
      model_input_height_, model_input_width_);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
    "Unsupported img encoding: %s", img_msg->encoding);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
      "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
      "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
      "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    return;
  }
}
#endif
