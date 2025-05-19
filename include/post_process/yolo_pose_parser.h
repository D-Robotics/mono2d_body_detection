// Copyright (c) 2025，D-Robotics.
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

#ifndef LLAMA_OUTPUT_PARSER_H_
#define LLAMA_OUTPUT_PARSER_H_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"
#include "dnn_node/util/output_parser/detection/nms.h"
#include "std_msgs/msg/string.hpp"

#include "dnn_node/util/output_parser/detection/fasterrcnn_output_parser.h"

using hobot::dnn_node::parser_fasterrcnn::Filter2DResult;
using hobot::dnn_node::parser_fasterrcnn::LandmarksResult;
using hobot::dnn_node::parser_fasterrcnn::PerceptionRect;
using hobot::dnn_node::parser_fasterrcnn::Landmarks;
using hobot::dnn_node::parser_fasterrcnn::Point;
using hobot::dnn_node::output_parser::Bbox;
using hobot::dnn_node::output_parser::Detection;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Perception;
using hobot::dnn_node::DNNTensor;

#define NMS_MAX_INPUT (400)
#define REG 16
// 分数阈值, 默认0.25
// Score threshold, default is 0.25
#define SCORE_THRESHOLD 0.5
#define input_H 640
#define input_W 640
#define CLASSES_NUM 1
#define KPT_NUM 17
#define KPT_ENCODE 3

int32_t FieldParse(const std::shared_ptr<DNNTensor> &boxes_tensor,
                   const std::shared_ptr<DNNTensor> &cls_tensor,
                   const std::shared_ptr<DNNTensor> &kpts_tensor,
                   const int32_t stride,
                   std::vector<PerceptionRect>& boxes,
                   std::vector<Landmarks>& values);

void nms(std::vector<PerceptionRect> &input,
         std::vector<Landmarks> &input_kps,
         float iou_threshold,
         int top_k,
         std::vector<PerceptionRect> &result,
         std::vector<Landmarks> &result_kps,
         bool suppress);

int32_t YoloPoseParse(
    const std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
    std::vector<std::shared_ptr<Filter2DResult>> &outputs,
    std::shared_ptr<LandmarksResult> &output_body_kps);

#endif  // LLAMA_OUTPUT_PARSER_H_
