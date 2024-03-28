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

#include "include/mono2d_body_det_node.h"

#include <fstream>
#include <memory>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/detail/time__struct.h"
#include "rcpputils/env.hpp"
#include "rcutils/env.h"

#ifdef PLATFORM_X86

enum class DataState {
  /// valid
  VALID = 0,
  /// filtered
  FILTERED = 1,
  /// invisible
  INVISIBLE = 2,
  /// disappeared
  DISAPPEARED = 3,
  /// invalid
  INVALID = 4,
};

struct box_s {
  box_s() {}
  box_s(int x1_, int y1_, int x2_, int y2_) {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
  }
  box_s(int x1_, int y1_, int x2_, int y2_, float score_) {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    score = score_;
  }
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  float score = 1;
  int id = 0;
  DataState state_ = DataState::VALID;
};

using MotBox = box_s;

#endif
builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

void NodeOutputManage::Feed(uint64_t ts_ms) {
  RCLCPP_DEBUG(
      rclcpp::get_logger("mono2d_body_det"), "feed frame ts: %llu", ts_ms);

  std::unique_lock<std::mutex> lk(mtx_);
  cache_frame_.insert(ts_ms);
  if (cache_frame_.size() > cache_size_limit_) {
    cache_frame_.erase(cache_frame_.begin());
  }
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
               "cache_frame_.size(): %d",
               cache_frame_.size());
}

std::vector<std::shared_ptr<DnnNodeOutput>> NodeOutputManage::Feed(
    const std::shared_ptr<DnnNodeOutput>& in_node_output) {
  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs{};
  auto fasterRcnn_output =
      std::dynamic_pointer_cast<FasterRcnnOutput>(in_node_output);
  if (!fasterRcnn_output || !fasterRcnn_output->image_msg_header) {
    return node_outputs;
  }

  uint64_t ts_ms =
      fasterRcnn_output->image_msg_header->stamp.sec * 1000 +
      fasterRcnn_output->image_msg_header->stamp.nanosec / 1000 / 1000;
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"), "feed ts: %llu", ts_ms);

  uint8_t loop_num = cache_size_limit_;
  {
    std::unique_lock<std::mutex> lk(mtx_);
    cache_node_output_[ts_ms] = in_node_output;
    if (cache_node_output_.size() > cache_size_limit_) {
      cache_node_output_.erase(cache_node_output_.begin());
    }
    if (cache_frame_.empty()) {
      return node_outputs;
    }

    loop_num = cache_node_output_.size();
  }

  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
               "node_outputs.size(): %d",
               node_outputs.size());

  // 按照时间戳顺序输出推理结果
  for (uint8_t idx = 0; idx < loop_num; idx++) {
    std::shared_ptr<DnnNodeOutput> node_output = nullptr;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      if (cache_frame_.empty() || cache_node_output_.empty()) {
        break;
      }

      auto first_frame = cache_frame_.begin();
      auto first_output = cache_node_output_.begin();
      if (*first_frame == first_output->first) {
        RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                     "push ts: %llu",
                     *first_frame);

        node_output = in_node_output;
        cache_frame_.erase(first_frame);
        cache_node_output_.erase(first_output);
      } else {
        if (first_output->first > *first_frame) {
          // 首个推理结果时间戳大于首帧图像数据的时间戳
          uint64_t time_ms_diff = first_output->first - *first_frame;
          if (time_ms_diff > smart_output_timeout_ms_) {
            // 首个推理结果和首帧图像数据的时间戳相差大于阈值，说明存在推理丢帧，删除首帧图像数据
            cache_frame_.erase(first_frame);
          }
        } else if (*first_frame > first_output->first) {
          // 首帧图像数据的时间戳大于首个推理结果时间戳
          uint64_t time_ms_diff = *first_frame - first_output->first;
          if (time_ms_diff > smart_output_timeout_ms_) {
            // 首帧图像数据和首个推理结果的时间戳相差大于阈值，删除首帧图像数据，理论上不应该出现这种case
            cache_node_output_.erase(first_output);
          }
        } else {
          // 时间戳相等的情况，不会走到这里
          break;
        }
      }
    }

    if (node_output) {
      node_outputs.emplace_back(node_output);
    }
  }

  return node_outputs;
}

void NodeOutputManage::Erase(uint64_t ts_ms) {
  std::unique_lock<std::mutex> lk(mtx_);
  if (cache_frame_.find(ts_ms) != cache_frame_.end()) {
    cache_frame_.erase(ts_ms);
  }
  if (cache_node_output_.find(ts_ms) != cache_node_output_.end()) {
    cache_node_output_.erase(ts_ms);
  }
}

Mono2dBodyDetNode::Mono2dBodyDetNode(const std::string& node_name,
                                     const NodeOptions& options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ros_img_topic_name",
                                       ros_img_topic_name_);   
  this->declare_parameter<int>("image_gap", image_gap_);      

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<int>("is_shared_mem_sub", is_shared_mem_sub_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("ros_img_topic_name",
                                       ros_img_topic_name_); 
  this->get_parameter<int>("image_gap", image_gap_);  
  {
    std::stringstream ss;
    ss << "Parameter:"
      << "\n is_sync_mode_: " << is_sync_mode_
      << "\n model_file_name_: " << model_file_name_
      << "\n is_shared_mem_sub: " << is_shared_mem_sub_
      << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
      << "\n ros_img_topic_name: " << ros_img_topic_name_ 
      << "\n image_gap: " << image_gap_;
    RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
  }

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Init failed!");
    rclcpp::shutdown();
    return;
  }

  // Init()之后模型已经加载成功，查询kps解析参数
  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Invalid model");
    rclcpp::shutdown();
    return;
  }
  parser_para_ = std::make_shared<FasterRcnnKpsParserPara>();
  hbDNNTensorProperties tensor_properties;
  model_manage->GetOutputTensorProperties(tensor_properties, kps_output_index_);
  parser_para_->aligned_kps_dim.clear();
  parser_para_->kps_shifts_.clear();
  for (int i = 0; i < tensor_properties.alignedShape.numDimensions; i++) {
    parser_para_->aligned_kps_dim.push_back(
        tensor_properties.alignedShape.dimensionSize[i]);
  }
  for (int i = 0; i < tensor_properties.shift.shiftLen; i++) {
    parser_para_->kps_shifts_.push_back(
        static_cast<uint8_t>(tensor_properties.shift.shiftData[i]));
  }
  {
    std::stringstream ss;
    ss << "aligned_kps_dim:";
    for (const auto& val : parser_para_->aligned_kps_dim) {
      ss << " " << val;
    }
    ss << "\nkps_shifts: ";
    for (const auto& val : parser_para_->kps_shifts_) {
      ss << " " << val;
    }
    ss << "\n";
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
  }

  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      ai_msg_pub_topic_name_, 10);

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
                 "Get model input size fail!");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
                "The model input width is %d and height is %d",
                model_input_width_,
                model_input_height_);
  }

  if (is_shared_mem_sub_) {
    std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
    if (ros_zerocopy_env.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "Launching with zero-copy, but env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is not set. "
        << "Transporting data without zero-copy!");
    } else {
      if ("1" == ros_zerocopy_env) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Enabling zero-copy");
      } else {
        RCLCPP_ERROR_STREAM(this->get_logger(),
          "env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is [" << ros_zerocopy_env
          << "], which should be set to 1. "
          << "Data transporting without zero-copy!");
      }
    }
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
                "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ =
        this->create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
            sharedmem_img_topic_name_,
            10,
            std::bind(&Mono2dBodyDetNode::SharedMemImgProcess,
                      this,
                      std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Unsupport shared mem");
#endif
  } else {
    RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
                "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_,
        10,
        std::bind(
            &Mono2dBodyDetNode::RosImgProcess, this, std::placeholders::_1));
  }
#ifndef PLATFORM_X86
  for (const auto& config : hobot_mot_configs_) {
    hobot_mots_[config.first] = std::make_shared<HobotMot>(config.second);
  }
#endif
}

Mono2dBodyDetNode::~Mono2dBodyDetNode() {}

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

int Mono2dBodyDetNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  if (!msg_publisher_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
                 "Invalid msg_publisher_");
    return -1;
  }

  std::vector<std::shared_ptr<DnnNodeOutput>> node_outputs{};
  if (node_output_manage_ptr_) {
    // 启用了输出排序功能
    node_outputs = node_output_manage_ptr_->Feed(output);
    if (node_outputs.empty()) {
      // 排序后的输出为空
      auto fasterRcnn_output =
          std::dynamic_pointer_cast<FasterRcnnOutput>(output);
      if (!fasterRcnn_output && !fasterRcnn_output->image_msg_header) {
        // 由于没有消息头（时间戳）导致的排序后的输出为空
        // 直接使用当前帧输出
        node_outputs.push_back(output);
      }
    }
  } else {
    // 未启用输出排序功能，直接使用当前帧输出
    node_outputs.push_back(output);
  }

  for (const auto node_output : node_outputs) {
    if (!node_output) {
      continue;
    }

    auto fasterRcnn_output =
        std::dynamic_pointer_cast<FasterRcnnOutput>(node_output);
    {
      std::stringstream ss;
      ss << "Output from";
      ss << ", frame_id: " << fasterRcnn_output->image_msg_header->frame_id
         << ", stamp: " << fasterRcnn_output->image_msg_header->stamp.sec << "_"
         << fasterRcnn_output->image_msg_header->stamp.nanosec
         << ", infer time ms: " << node_output->rt_stat->infer_time_ms;
      RCLCPP_INFO(
          rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
    }

    // 创建解析输出数据，检测框和关键点数据
    // results的维度等于检测出来的目标类别数
    std::vector<std::shared_ptr<hobot::dnn_node::parser_fasterrcnn::Filter2DResult>>
        results;
    std::shared_ptr<LandmarksResult> lmk_result = nullptr;
    
    // 使用hobot dnn内置的Parse解析方法，解析算法输出的DNNTensor类型数据
    if (hobot::dnn_node::parser_fasterrcnn::Parse(node_output, parser_para_,
    box_outputs_index_, kps_output_index_, body_box_output_index_, results, lmk_result) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("dnn_node_sample"),
                  "Parse node_output fail!");
      return -1;
    }

    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);

    ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
        new ai_msgs::msg::PerceptionTargets());
    if (fasterRcnn_output->image_msg_header) {
      pub_data->header.set__stamp(fasterRcnn_output->image_msg_header->stamp);
      pub_data->header.set__frame_id(
          fasterRcnn_output->image_msg_header->frame_id);
    }
    if (output->rt_stat) {
      pub_data->set__fps(round(output->rt_stat->output_fps));
    }

    // key is model output index
    std::unordered_map<int32_t, std::vector<MotBox>> rois;
    std::vector<ai_msgs::msg::Point> body_kps;

    for (const auto& idx : box_outputs_index_) {
      if (idx >= results.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
                     "Output index: %d exceeds results size %d",
                     idx, results.size());
        return -1;
      }

      auto filter2d_result = results.at(idx);
      if (!filter2d_result) {
        continue;
      }

      if (box_outputs_index_type_.find(idx) == box_outputs_index_type_.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"),
                     "Invalid output index: %d",
                     idx);
        return -1;
      }

      rois[idx].resize(0);
      std::string roi_type = box_outputs_index_type_[idx];

      RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
                  "Output box type: %s, rect size: %d",
                  roi_type.data(),
                  filter2d_result->boxes.size());

      for (auto& rect : filter2d_result->boxes) {
        rect.left /= width_scale_;
        rect.right /= width_scale_;
        rect.top /= height_scale_;
        rect.bottom /= height_scale_;
        if (rect.left < 0) rect.left = 0;
        if (rect.top < 0) rect.top = 0;
        if (rect.right > model_input_width_ / width_scale_) {
          rect.right = model_input_width_ / width_scale_;
        }
        if (rect.bottom > model_input_height_ / height_scale_) {
          rect.bottom = model_input_height_ / height_scale_;
        }

        std::stringstream ss;
        ss << "rect: " << rect.left << " " << rect.top << " " << rect.right
           << " " << rect.bottom << ", " << rect.conf;
        RCLCPP_INFO(
            rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

        rois[idx].emplace_back(
            MotBox(rect.left, rect.top, rect.right, rect.bottom, rect.conf));
      }
    }

    if (lmk_result) {
      std::stringstream ss;
      for (const auto& value : lmk_result->values) {
        ai_msgs::msg::Point target_point;
        target_point.set__type("body_kps");
        ss << "kps point: ";
        for (const auto& lmk : value) {
          ss << "\n" << lmk.x << "," << lmk.y << "," << lmk.score;
          geometry_msgs::msg::Point32 pt;
          pt.set__x(lmk.x / width_scale_);
          pt.set__y(lmk.y / height_scale_);
          target_point.point.emplace_back(pt);
          target_point.confidence.push_back(lmk.score);
        }
        ss << "\n";
        RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                     "FasterRcnnKpsOutputParser parse kps: %s",
                     ss.str().c_str());
        body_kps.emplace_back(target_point);
      }
    }

    std::unordered_map<int32_t, std::vector<MotBox>> out_rois;
#ifndef PLATFORM_X86
    std::unordered_map<int32_t, std::vector<std::shared_ptr<MotTrackId>>>
        out_disappeared_ids;

    uint64_t ts_ms =
        fasterRcnn_output->image_msg_header->stamp.sec * 1000 +
        fasterRcnn_output->image_msg_header->stamp.nanosec / 1000 / 1000;
    time_t time_stamp = ts_ms;

    DoMot(time_stamp, rois, out_rois, out_disappeared_ids);
#endif
#ifndef PLATFORM_X86
    for (const auto& out_roi : out_rois) 
#else
    for (const auto& out_roi : rois) 
#endif
    {
      std::string roi_type = "";
      if (box_outputs_index_type_.find(out_roi.first) !=
          box_outputs_index_type_.end()) {
        roi_type = box_outputs_index_type_.at(out_roi.first);
      }
      for (size_t idx = 0; idx < out_roi.second.size(); idx++) {
        const auto& rect = out_roi.second.at(idx);
#ifndef PLATFORM_X86
        if (rect.id < 0 || hobot_mot::DataState::INVALID == rect.state_) 
#else
        if (rect.id < 0 || DataState::INVALID == rect.state_) 
#endif
        {
          std::stringstream ss;
          ss << "invalid id, rect: " << rect.x1 << " " << rect.y1 << " "
             << rect.x2 << " " << rect.y2 << ", score: " << rect.score
             << ", state_: " << static_cast<int>(rect.state_);
          RCLCPP_INFO(
              rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
          continue;
        }
        ai_msgs::msg::Target target;
        target.set__type("person");
        target.set__track_id(rect.id);
        ai_msgs::msg::Roi roi;
        roi.type = roi_type;
        roi.rect.set__x_offset(rect.x1);
        roi.rect.set__y_offset(rect.y1);
        roi.rect.set__width(rect.x2 - rect.x1);
        roi.rect.set__height(rect.y2 - rect.y1);
        target.rois.emplace_back(roi);
        if (out_roi.first == body_box_output_index_ &&
            out_roi.second.size() == body_kps.size()) {
          target.points.emplace_back(body_kps.at(idx));
        }
        pub_data->targets.emplace_back(std::move(target));
      }
    }
#ifndef PLATFORM_X86
    for (const auto& disappeared_id : out_disappeared_ids) {
      std::string roi_type = "";
      if (box_outputs_index_type_.find(disappeared_id.first) !=
          box_outputs_index_type_.end()) {
        roi_type = box_outputs_index_type_.at(disappeared_id.first);
      }
      for (size_t idx = 0; idx < disappeared_id.second.size(); idx++) {
        auto id_info = disappeared_id.second.at(idx);
        if (id_info->value < 0 ||
            hobot_mot::DataState::INVALID == id_info->state_) {
          continue;
        }
        ai_msgs::msg::Target target;
        target.set__type("person");
        target.set__track_id(id_info->value);
        ai_msgs::msg::Roi roi;
        roi.type = roi_type;
        target.rois.emplace_back(roi);
        pub_data->disappeared_targets.emplace_back(std::move(target));
      }
    }
#endif
    struct timespec time_now = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_now);

    // preprocess
    ai_msgs::msg::Perf perf_preprocess;
    perf_preprocess.set__type(model_name_ + "_preprocess");
    perf_preprocess.set__stamp_start(
        ConvertToRosTime(fasterRcnn_output->preprocess_timespec_start));
    perf_preprocess.set__stamp_end(
        ConvertToRosTime(fasterRcnn_output->preprocess_timespec_end));
    perf_preprocess.set__time_ms_duration(CalTimeMsDuration(
        perf_preprocess.stamp_start, perf_preprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_preprocess);

    // predict
    if (output->rt_stat) {
      ai_msgs::msg::Perf perf;
      perf.set__type(model_name_ + "_predict_infer");
      perf.set__stamp_start(
          ConvertToRosTime(output->rt_stat->infer_timespec_start));
      perf.set__stamp_end(
          ConvertToRosTime(output->rt_stat->infer_timespec_end));
      perf.set__time_ms_duration(output->rt_stat->infer_time_ms);
      pub_data->perfs.push_back(perf);

      perf.set__type(model_name_ + "_predict_parse");
      perf.set__stamp_start(
          ConvertToRosTime(output->rt_stat->parse_timespec_start));
      perf.set__stamp_end(
          ConvertToRosTime(output->rt_stat->parse_timespec_end));
      perf.set__time_ms_duration(output->rt_stat->parse_time_ms);
      pub_data->perfs.push_back(perf);
    }

    // postprocess
    ai_msgs::msg::Perf perf_postprocess;
    perf_postprocess.set__type(model_name_ + "_postprocess");
    perf_postprocess.set__stamp_start(ConvertToRosTime(time_start));
    clock_gettime(CLOCK_REALTIME, &time_now);
    perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
    perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
        perf_postprocess.stamp_start, perf_postprocess.stamp_end));
    pub_data->perfs.emplace_back(perf_postprocess);

    // 从发布图像到发布AI结果的延迟
    ai_msgs::msg::Perf perf_pipeline;
    perf_pipeline.set__type(model_name_ + "_pipeline");
    perf_pipeline.set__stamp_start(pub_data->header.stamp);
    perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
    perf_pipeline.set__time_ms_duration(
        CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
    pub_data->perfs.push_back(perf_pipeline);

    {
      std::stringstream ss;
      ss << "Publish frame_id: "
         << fasterRcnn_output->image_msg_header->frame_id
         << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec)
         << "_" << std::to_string(pub_data->header.stamp.nanosec) << "\n";
      RCLCPP_INFO(
          rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
    }

    std::stringstream ss;
    ss << "Publish frame_id: " << fasterRcnn_output->image_msg_header->frame_id
       << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec) << "_"
       << std::to_string(pub_data->header.stamp.nanosec) << "\n";
    ss << "targets.size: " << pub_data->targets.size() << "\n";

    if (!pub_data->targets.empty()) {
      for (const auto target : pub_data->targets) {
        ss << "target track_id: " << target.track_id
           << ", rois.size: " << target.rois.size();
        for (const auto& roi : target.rois) {
          ss << ", " << roi.type.c_str();
        }
        ss << ", points.size: " << target.points.size();
        for (const auto& point : target.points) {
          ss << ", " << point.type.c_str();
        }
        ss << "\n";
      }
    }

    ss << "disappeared_targets.size: " << pub_data->disappeared_targets.size()
       << "\n";
    if (!pub_data->disappeared_targets.empty()) {
      for (const auto& target : pub_data->disappeared_targets) {
        ss << "disappeared target track_id: " << target.track_id
           << ", rois.size: " << target.rois.size();
        for (const auto& roi : target.rois) {
          ss << ", " << roi.type.c_str();
        }
        ss << "\n";
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

    if (node_output->rt_stat->fps_updated) {
      RCLCPP_WARN(rclcpp::get_logger("mono2d_body_det"),
                  "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms,
                  static_cast<int>(perf_postprocess.time_ms_duration));
    }

    msg_publisher_->publish(std::move(pub_data));
  }
  return 0;
}

int Mono2dBodyDetNode::Predict(
    std::vector<std::shared_ptr<DNNInput>>& inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output) {
  RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
               "task_num: %d",
               dnn_node_para_ptr_->task_num);
  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1 ? true : false);
}

void Mono2dBodyDetNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }
  static int gap_cnt = 0;
  if (++gap_cnt < image_gap_) {
    return;
  }
  gap_cnt = 0;
  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "_"
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());
  width_scale_ = static_cast<double>(model_input_width_) / img_msg->width;
  height_scale_ = static_cast<double>(model_input_height_) / img_msg->height;
  // dump recved img msg
  // std::ofstream ofs("img." + img_msg->encoding);
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data.size());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding || "bgr8" == img_msg->encoding) {
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    // dump recved img msg after convert
    // cv::imwrite("dump_raw_" +
    //     std::to_string(img_msg->header.stamp.sec) + "." +
    //     std::to_string(img_msg->header.stamp.nanosec) + ".jpg",
    //     cv_img->image);

    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                   "after cvtColorForDisplay cost ms: %d",
                   interval);
    }
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromBGRImg(
        cv_img->image, model_input_height_, model_input_width_);
  } else if ("nv12" == img_msg->encoding) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                 "after GetNV12Pyramid cost ms: %d",
                 interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header->set__stamp(img_msg->header.stamp);

  if (node_output_manage_ptr_) {
    node_output_manage_ptr_->Feed(img_msg->header.stamp.sec * 1000 +
                                  img_msg->header.stamp.nanosec / 1000 / 1000);
  }

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                 "after Predict cost ms: %d",
                 interval);
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
  if (!img_msg || !rclcpp::ok()) {
    return;
  }
  static int gap_cnt = 0;
  if (++gap_cnt < image_gap_) {
    return;
  }
  gap_cnt = 0;
  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step << ", index: " << img_msg->index
     << ", stamp: " << img_msg->time_stamp.sec << "_"
     << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "%s", ss.str().c_str());

  rclcpp::Time msg_ts = img_msg->time_stamp;
  rclcpp::Duration dura = this->now() - msg_ts;
  float duration_ms = dura.nanoseconds() / 1000.0 / 1000.0;
  RCLCPP_WARN_THROTTLE(this->get_logger(),
    *this->get_clock(), 3000,
    "%s, comm delay [%.4f]ms",
    ss.str().c_str(), duration_ms);

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"),
                "Unsupported img encoding: %s",
                img_msg->encoding);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                 "after GetNV12Pyramid cost ms: %d",
                 interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<FasterRcnnOutput>();
  dnn_output->image_msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header->set__stamp(img_msg->time_stamp);

  if (node_output_manage_ptr_) {
    node_output_manage_ptr_->Feed(img_msg->time_stamp.sec * 1000 +
                                  img_msg->time_stamp.nanosec / 1000 / 1000);
  }

  dnn_output->preprocess_timespec_start = time_start;
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->preprocess_timespec_end = time_now;

  uint32_t ret = 0;
  // 3. 开始预测
  ret = Predict(inputs, nullptr, dnn_output);

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("mono2d_body_det"),
                 "after Predict cost ms: %d",
                 interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    return;
  }
}
#endif
#ifndef PLATFORM_X86
int Mono2dBodyDetNode::DoMot(
    const time_t& time_stamp,
    const std::unordered_map<int32_t, std::vector<MotBox>>& in_rois,
    std::unordered_map<int32_t, std::vector<MotBox>>& out_rois,
    std::unordered_map<int32_t, std::vector<std::shared_ptr<MotTrackId>>>&
        out_disappeared_ids) {
  if (hobot_mots_.empty()) {
    return -1;
  }
  for (auto& roi : in_rois) {
    std::shared_ptr<HobotMot> hobot_mot = nullptr;
    if (box_outputs_index_type_.find(roi.first) !=
        box_outputs_index_type_.end()) {
      if (hobot_mots_.find(box_outputs_index_type_.at(roi.first)) !=
          hobot_mots_.end()) {
        hobot_mot = hobot_mots_.at(box_outputs_index_type_.at(roi.first));
      } else {
        continue;
      }
    }
    if (!hobot_mot) {
      continue;
    }

    std::vector<MotBox> out_box_list;
    std::vector<std::shared_ptr<MotTrackId>> disappeared_ids;

    if (hobot_mot->DoProcess(roi.second,
                             out_box_list,
                             disappeared_ids,
                             time_stamp,
                             model_input_width_,
                             model_input_height_) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Do mot fail");
      continue;
    }

    out_rois[roi.first] = out_box_list;
    out_disappeared_ids[roi.first] = disappeared_ids;
  }
  return 0;
}
#endif
