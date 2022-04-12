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

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node.h"
#include "include/mono2d_body_det_node.h"
#include "include/image_utils.h"
#include "include/fasterrcnn_kps_output_parser.h"
#include "include/image_subscriber.h"
#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif

Mono2dBodyDetNode::Mono2dBodyDetNode(
    const std::string & node_name,
    const NodeOptions & options) :
    DnnNode(node_name, options) {
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);

  std::stringstream ss;
  ss << "Parameter:"
  << "\n is_sync_mode_: " << is_sync_mode_
  << "\n model_file_name_: " << model_file_name_;
  RCLCPP_WARN(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  RCLCPP_INFO(rclcpp::get_logger("msg pub"),
    "msg_pub_topic_name: %s", msg_pub_topic_name.data());
  msg_publisher_ =
      this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        msg_pub_topic_name, 10);
}

Mono2dBodyDetNode::~Mono2dBodyDetNode() {
}

int Mono2dBodyDetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("example"), "Set node para.");
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
  RCLCPP_INFO(rclcpp::get_logger("example"), "Set output parser.");
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
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
  model_manage->SetOutputParser(box_output_index_, box_out_parser);

  // 2 set kps paser
  // 2.1 set dependencies
  auto output_desc =
      std::make_shared<OutputDescription>(model_manage, kps_output_index_,
      "body_kps_branch");
  output_desc->GetDependencies().push_back(box_output_index_);
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
  RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

  // 2.3 create and set kps parser
  std::shared_ptr<OutputParser> kps_out_parser =
      std::make_shared<FasterRcnnKpsOutputParser>(parser_para);
  model_manage->SetOutputParser(kps_output_index_, kps_out_parser);

  return 0;
}

int Mono2dBodyDetNode::PreProcess(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const TaskId& task_id,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois) {
  std::shared_ptr<ModelInferTask> infer_task =
    std::dynamic_pointer_cast<ModelInferTask>(GetTask(task_id));
  if (!infer_task) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid infer task");
    return -1;
  }
  uint32_t ret = infer_task->SetInputs(inputs);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Failed to set inputs");
    return ret;
  }
  if (rois) {
    // set roi
  }
  return ret;
}

int Mono2dBodyDetNode::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &node_output) {
  if (!msg_publisher_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid msg_publisher_");
    return -1;
  }

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
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());
  }

  const auto& outputs = node_output->outputs;
  RCLCPP_INFO(rclcpp::get_logger("example"),
    "outputs size: %d", outputs.size());
  if (outputs.empty() ||
    static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid outputs");
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
      RCLCPP_WARN(rclcpp::get_logger("example"),
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

  // outputs.front().size() is 3
  // 0: invalid, 1: box, 2: kps
  // box
  auto *filter2d_result =
    dynamic_cast<Filter2DResult *>(outputs[box_output_index_].get());
  if (!filter2d_result) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "invalid Filter2DResult cast");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("example"), "out box size: %d",
      filter2d_result->boxes.size());
  }

  // kps
  auto *lmk_result =
    dynamic_cast<LandmarksResult *>(outputs[kps_output_index_].get());
  if (!lmk_result) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "invalid LandmarksResult cast");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("example"), "out kps size: %d",
      lmk_result->values.size());
  }

  if (filter2d_result->boxes.size() != lmk_result->values.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "out box size: %d is unmatch with kps size: %d",
      filter2d_result->boxes.size(), lmk_result->values.size());
    return -1;
  }

  std::vector<ai_msgs::msg::Roi> rois;
  std::vector<ai_msgs::msg::Point> points;

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
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

    ai_msgs::msg::Roi roi;
    roi.type = "body";
    roi.rect.set__x_offset(rect.left);
    roi.rect.set__y_offset(rect.top);
    roi.rect.set__width(rect.right - rect.left);
    roi.rect.set__height(rect.bottom - rect.top);
    rois.emplace_back(roi);
  }

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
    RCLCPP_DEBUG(rclcpp::get_logger("example"),
      "FasterRcnnKpsOutputParser parse kps: %s", ss.str().c_str());
    points.emplace_back(target_point);
  }

  if (!rois.empty() && rois.size() == points.size()) {
    for (size_t idx = 0; idx < rois.size(); idx++) {
      ai_msgs::msg::Target target;
      target.set__type("person");
      target.rois.emplace_back(rois.at(idx));
      target.points.emplace_back(points.at(idx));
      pub_data->targets.emplace_back(std::move(target));
    }
  }

  clock_gettime(CLOCK_REALTIME, &time_start);
  perf.stamp_end.sec = time_start.tv_sec;
  perf.stamp_end.nanosec = time_start.tv_nsec;
  pub_data->perfs.emplace_back(perf);

  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int Mono2dBodyDetNode::Predict(
  std::vector<std::shared_ptr<DNNInput>> &inputs,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois,
  std::shared_ptr<DnnNodeOutput> dnn_output) {
  RCLCPP_INFO(rclcpp::get_logger("example"), "task_num: %d",
  dnn_node_para_ptr_->task_num);
  // 1. 申请预测task
  auto task_id = AllocTask();
  uint32_t ret = 0;
  // 2. 将准备好的数据通过前处理接口输入给模型
  ret = PreProcess(inputs, task_id, rois);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run PreProcess failed!");
    return ret;
  }

  // 3. 模型推理
  ret = RunInferTask(dnn_output, task_id, is_sync_mode_);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run RunInferTask failed!");
    ReleaseTask(task_id);
    return ret;
  }
  if (is_sync_mode_) {
    ReleaseTask(task_id);
    // 4. 通过后处理接口处理模型输出
    ret = PostProcess(dnn_output);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Run PostProcess failed!");
    }
  }
  return ret;
}

int Mono2dBodyDetNode::Feed() {
  RCLCPP_INFO(rclcpp::get_logger("example"),
    "Dnn node feed with img subscriber");
  if (!image_subscriber_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid img subscriber");
    return -1;
  }

  while (rclcpp::ok()) {
    RCLCPP_INFO(rclcpp::get_logger("example"), "Get img start");
    // 1. 订阅图片消息，如果无publisher，阻塞在GetImg调用
    auto img_msg = image_subscriber_->GetImg();
    if (!img_msg) {
      continue;
    }

    if (!rclcpp::ok()) {
      return 0;
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
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss.str().c_str());

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
        RCLCPP_DEBUG(rclcpp::get_logger("example"),
          "after cvtColorForDisplay cost ms: %d", interval);
      }

      pyramid = ImageUtils::GetNV12Pyramid(cv_img->image,
        model_input_height_, model_input_width_);
#else
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Unsupport cv bridge");
#endif
    } else if ("nv12" == img_msg->encoding) {
      pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height, img_msg->width,
        model_input_height_, model_input_width_);
    }

    if (!pyramid) {
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Get Nv12 pym fail");
      continue;
    }

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start).count();
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
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
      RCLCPP_DEBUG(rclcpp::get_logger("example"),
        "after Predict cost ms: %d", interval);
    }

    // 4. 处理预测结果，如渲染到图片或者发布预测结果
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
      return ret;
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("example"), "Feed done");
  return 0;
}

#ifdef SHARED_MEM_ENABLED
void Mono2dBodyDetNode::SharedMemImgProcess(
  const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr &img_msg) {
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
    RCLCPP_INFO(rclcpp::get_logger("example"),
    "Unsupported img encoding: %s", img_msg->encoding);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("example"),
      "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("example"),
      "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  // dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);
  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);
  RCLCPP_INFO(rclcpp::get_logger("example"), "===%d", __LINE__);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - tp_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("example"),
      "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Run predict failed!");
    return;
  }
}
#endif

int Mono2dBodyDetNode::Run() {
  RCLCPP_INFO(rclcpp::get_logger("example"),
    "Dnn node feed with subscription");

  rclcpp::executors::SingleThreadedExecutor exec;
#ifdef SHARED_MEM_ENABLED
  image_subscriber_ = std::make_shared<ImageSubscriber>(
    std::bind(&Mono2dBodyDetNode::SharedMemImgProcess, this,
              std::placeholders::_1));
  exec.add_node(image_subscriber_);
#else
  image_subscriber_ = std::make_shared<ImageSubscriber>();
  exec.add_node(image_subscriber_);
  auto predict_task = std::make_shared<std::thread>(
    std::bind(&Mono2dBodyDetNode::Feed, this));
  exec.spin();
  if (predict_task && predict_task->joinable()) {
    predict_task.reset();
  }
#endif

  exec.spin();

  return 0;
}
