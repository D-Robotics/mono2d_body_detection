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

#include <algorithm>

#include "include/post_process/yolo_pose_parser.h"

int32_t FieldParse(const std::shared_ptr<DNNTensor> &boxes_tensor,
                   const std::shared_ptr<DNNTensor> &cls_tensor,
                   const std::shared_ptr<DNNTensor> &kpts_tensor,
                   const int32_t stride,
                   std::vector<PerceptionRect>& boxes,
                   std::vector<Landmarks>& values) {

  int32_t H = input_H / stride;
  int32_t W = input_W / stride;

  float CONF_THRES_RAW = -log(1 / SCORE_THRESHOLD - 1);         // 利用反函数作用阈值，利用单调性筛选

  boxes_tensor->CACHE_INVALIDATE();
  cls_tensor->CACHE_INVALIDATE();
  kpts_tensor->CACHE_INVALIDATE();

  // 将BPU推理完的内存地址转换为对应类型的指针
  auto *bbox_raw = boxes_tensor->GetTensorData<float>();
  auto *cls_raw = cls_tensor->GetTensorData<float>();
  auto *kpts_raw = kpts_tensor->GetTensorData<float>();

  for (int h = 0; h < H; h++)
  {
      for (int w = 0; w < W; w++)
      {
          // 1. 取对应H和W位置的C通道, 记为数组的形式
          // cls对应CLASSES_NUM个分数RAW值, 也就是Sigmoid计算之前的值，这里利用函数单调性先筛选, 再计算
          // bbox对应4个坐标乘以REG的RAW值, 也就是DFL计算之前的值, 仅仅分数合格了, 才会进行这部分的计算
          float *cur_cls_raw = cls_raw;
          float *cur_bbox_raw = bbox_raw;
          float *cur_kpts_raw = kpts_raw;
          cls_raw += CLASSES_NUM;
          bbox_raw += REG * 4;
          kpts_raw += KPT_NUM * KPT_ENCODE; 

          // 2. 找到分数的最大值索引, 如果最大值小于阈值，则舍去
          int cls_id = 0;
          for (int i = 1; i < CLASSES_NUM; i++)
          {
              if (cur_cls_raw[i] > cur_cls_raw[cls_id])
              {
                  cls_id = i;
              }
          }

          // 3. 不合格则直接跳过, 避免无用的反量化, DFL和dist2bbox计算
          if (cur_cls_raw[cls_id] < CONF_THRES_RAW)
          {
              continue;
          }

          // 4. 计算这个目标的分数
          float score = 1 / (1 + std::exp(-cur_cls_raw[cls_id]));

          // 5. 对bbox_raw信息进行反量化, DFL计算
          float ltrb[4], sum, dfl;
          for (int i = 0; i < 4; i++)
          {
              ltrb[i] = 0.;
              sum = 0.;
              for (int j = 0; j < REG; j++)
              {
                  dfl = std::exp(cur_bbox_raw[REG * i + j]);
                  ltrb[i] += dfl * j;
                  sum += dfl;
              }
              ltrb[i] /= sum;
          }

          // 6. 剔除不合格的框   if(x1 >= x2 || y1 >=y2) continue;
          if (ltrb[2] + ltrb[0] <= 0 || ltrb[3] + ltrb[1] <= 0)
          {
              continue;
          }

          // 7. dist 2 bbox (ltrb 2 xyxy)
          float x1 = (w + 0.5 - ltrb[0]) * static_cast<float>(stride);
          float y1 = (h + 0.5 - ltrb[1]) * static_cast<float>(stride);
          float x2 = (w + 0.5 + ltrb[2]) * static_cast<float>(stride);
          float y2 = (h + 0.5 + ltrb[3]) * static_cast<float>(stride);

          // 8. kpts的处理
          Landmarks skeleton;
          skeleton.resize(KPT_NUM + 2);
          for (int j = 0; j < KPT_NUM; j++)
          {
              float x = (cur_kpts_raw[KPT_ENCODE * j] * 2.0 + w) * static_cast<float>(stride);
              float y = (cur_kpts_raw[KPT_ENCODE * j + 1] * 2.0 + h) * static_cast<float>(stride);
              Point point;
              point.x = x;
              point.y = y;
              point.score = cur_kpts_raw[KPT_ENCODE * j + 2];
              skeleton[j] = point;
          }
          skeleton[KPT_NUM] = skeleton[KPT_NUM - 2];
          skeleton[KPT_NUM + 1] = skeleton[KPT_NUM - 1];

          // 9. 对应类别加入到对应的std::vector中
          PerceptionRect roi{};
          roi.left = x1;
          roi.top = y1;
          roi.right = x2;
          roi.bottom = y2;
          roi.conf = score;
          boxes.push_back(roi);
          values.emplace_back(skeleton);
      }
  }
  return 0;
}

int32_t YoloPoseParse(
    const std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
    std::vector<std::shared_ptr<Filter2DResult>> &outputs,
    std::shared_ptr<LandmarksResult> &output_body_kps) {
  
  if (!output_body_kps) {
    output_body_kps = std::make_shared<LandmarksResult>();
    output_body_kps->Reset();
  }

  outputs.resize(2);
  if (!outputs[1]) {
    outputs[1] = std::make_shared<Filter2DResult>();
  }

  std::vector<PerceptionRect> boxes;
  std::vector<Landmarks> values;

  std::vector<std::vector<int32_t>> orders = {{1, 0, 2}, {4, 3, 5}, {7, 6, 8}};

  std::vector<int32_t> strides = {8, 16, 32};

  for (int i = 0; i < orders.size(); i++) {
    FieldParse(output_tensors[orders[i][0]], output_tensors[orders[i][1]], output_tensors[orders[i][2]], strides[i], boxes, values);
  }
  nms(boxes, values, 0.45, 50, outputs[1]->boxes, output_body_kps->values, false);

  return 0;
}

void nms(std::vector<PerceptionRect> &input,
         std::vector<Landmarks> &input_kps,
         float iou_threshold,
         int top_k,
         std::vector<PerceptionRect> &result,
         std::vector<Landmarks> &result_kps,
         bool suppress) {
  // sort order by score desc
  // std::stable_sort(input.begin(), input.end(), std::greater<PerceptionRect>());
  if (input.size() > NMS_MAX_INPUT) {
    input.resize(NMS_MAX_INPUT);
  }

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].right - input[i].left;
    float height = input[i].bottom - input[i].top;
    areas.push_back(width * height);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].type != input[j].type) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].left, input[j].left);
      float yy1 = std::max(input[i].top, input[j].top);
      float xx2 = std::min(input[i].right, input[j].right);
      float yy2 = std::min(input[i].bottom, input[j].bottom);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }
    result.push_back(input[i]);
    result_kps.push_back(input_kps[i]);
  }
}
